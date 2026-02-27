
#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <vector>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>
#include <unordered_map>
#include <cmath>
#include <Wire.h>

enum waveform_t {SINE, SQUARE, TRIANGLE, SAWTOOTH};

//Constants
  const uint32_t interval = 100; //Display update interval
  const uint32_t sampling_frequency = 22000;
  constexpr uint8_t kNumVoices = 4;

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;
  const int KNOB_INT_PIN = PA10;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int KNOB_MODE = 2;
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;
  const int ALL_BITS = 0x7;

  uint32_t ID = 0x123;
  uint32_t stepSizes[12];
  double frequencies[12];
  int32_t sineLUT[4096];
  
  volatile uint32_t currentStepSize = 0;
  volatile uint8_t knob3Rotation = 8;
  volatile uint8_t knob2Rotation = 0;
  volatile uint8_t knob1Rotation = 4;
  volatile uint8_t TX_Message[8] = {0};
  volatile uint8_t octave = 4;
  volatile waveform_t currentWaveform = SAWTOOTH;

struct Voice{
uint8_t octave;
uint8_t key;
uint32_t phase;
uint32_t step;
uint32_t value;
bool active;
};

QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;
HardwareTimer sampleTimer(TIM1);
SemaphoreHandle_t CAN_TX_Semaphore;
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t knobIntSem;

volatile Voice voices[kNumVoices];

struct {
std::bitset<32> inputs; 
uint8_t RX_Message[8] = {0};
SemaphoreHandle_t mutex;
} sysState;

//Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// README matrix map: index = row*4 + col
const char * const INPUT_MAP[32] = {
  "Key C",        "Key C#",       "Key D",        "Key D#",
  "Key E",        "Key F",        "Key F#",       "Key G",
  "Key G#",       "Key A",        "Key A#",       "Key B",
  "Knob 3 A",     "Knob 3 B",     "Knob 2 A",     "Knob 2 B",
  "Knob 1 A",     "Knob 1 B",     "Knob 0 A",     "Knob 0 B",
  "Knob 2 S",     "Knob 3 S",     "Joystick S",   "West Detect",
  "Knob 0 S",     "Knob 1 S",     "Unused",       "East Detect",
  "Unused",       "Unused",       "Unused",       "Unused"
};

uint8_t pcalReadInputs() {
  constexpr uint8_t kPcalAddr = 0x21;
  constexpr uint8_t kInputReg = 0x00;

  Wire.beginTransmission(kPcalAddr);
  Wire.write(kInputReg);
  if (Wire.endTransmission(false) != 0) {
    return 0xFF;
  }

  if (Wire.requestFrom(kPcalAddr, static_cast<uint8_t>(1)) != 1) {
    return 0xFF;
  }

  return Wire.read();
}

void pcalWriteReg(const uint8_t reg, const uint8_t value) {
  constexpr uint8_t kPcalAddr = 0x21;
  Wire.beginTransmission(kPcalAddr);
  Wire.write(reg);
  Wire.write(value);
  (void)Wire.endTransmission();
}

void knobIntISR() {
  xSemaphoreGiveFromISR(knobIntSem, NULL);
}



//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

std::bitset<4> readCols(){

  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);
  return result;
}

void setRow(const uint8_t rowIdx){
  digitalWrite(REN_PIN, LOW);
  if(rowIdx == 2){
    digitalWrite(OUT_PIN, LOW);
  }else{
    digitalWrite(OUT_PIN, HIGH);
  }
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN, HIGH);
}

std::string wave2String(const waveform_t wave){
  if (wave == SINE){
    return "SINE";
  }else if(wave == SAWTOOTH){
    return "SAWTOOTH";
  }else if(wave == SQUARE){
    return "SQUARE";
  }else if(wave == TRIANGLE){
    return "TRIANGLE";
  }
  return "UNKNOWN";
}

void applyKeyEvent(const uint8_t command, const uint8_t octaveValue, const uint8_t key){
  xSemaphoreTake(sysState.mutex, portMAX_DELAY);
  sysState.RX_Message[0] = command;
  sysState.RX_Message[1] = octaveValue;
  sysState.RX_Message[2] = key;

  if(command == 'P'){
    for(int i = 0; i<kNumVoices; i++){
      if(voices[i].active){
        continue;
      }else{
        voices[i].octave = octaveValue;
        voices[i].key = key;
        voices[i].phase = 0;
        voices[i].step = static_cast<uint32_t>(
          (static_cast<double>(frequencies[key]) *
           pow(2.0, static_cast<double>(static_cast<int>(octaveValue) - 4)) *
           4294967296.0) / sampling_frequency
        );
        voices[i].active = true;
        break;
      }
    }
  }else if(command == 'R'){
    for(int i = 0; i<kNumVoices; i++){
      if(voices[i].active && voices[i].octave == octaveValue && voices[i].key == key){
        voices[i].active = false;
        break;
      }
    }
  }
  xSemaphoreGive(sysState.mutex);
}

void sampleISR(){
  static int32_t phaseAcc = 0;
  phaseAcc = 0;
  uint8_t count = 0;

  if(currentWaveform == SINE){
    for(int i = 0; i<kNumVoices; i++){
      if(voices[i].active){
        int32_t sample = sineLUT[voices[i].phase >> 20];
        phaseAcc += sample >> (8-knob3Rotation);
        voices[i].phase += voices[i].step;
        count++;
      }
    }
  }else if(currentWaveform == SAWTOOTH){
    for(int i = 0; i<kNumVoices; i++){
      if(voices[i].active){
        int32_t sample = static_cast<int32_t>(voices[i].phase - 0x80000000u);
        phaseAcc += sample >> (2+(8-knob3Rotation));
        voices[i].phase += voices[i].step;
        count++;
      }
    }
  }else if(currentWaveform == SQUARE){
    for(int i = 0; i<kNumVoices; i++){
      if(voices[i].active){
        int32_t sample = (voices[i].phase & 0x80000000u) ? 2147483647 : -2147483647;
        phaseAcc += sample >> (2+(8-knob3Rotation));
        voices[i].phase += voices[i].step;
        count++;
      }
    }
  }else if(currentWaveform == TRIANGLE){
    for(int i = 0; i<kNumVoices; i++){
      if(voices[i].active){
        uint32_t ramp = voices[i].phase >> 15; // 17-bit ramp [0..131071]
        uint32_t tri = (ramp & 0x10000u) ? (0x1FFFFu - ramp) : ramp; // Fold into triangle
        int32_t sample = (static_cast<int32_t>(tri) << 16) - 2147483647;
        phaseAcc += sample >> (2+(8-knob3Rotation));
        voices[i].phase += voices[i].step;
        count++;
      }
    }
  }

  if(count != 0){  
    int32_t Vout = (phaseAcc >> 20);
    //Vout = Vout >> (8-knob3Rotation); //Apply volume control
    analogWrite(OUTR_PIN, Vout + 2048);
  }

}

void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}


void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void scanKeysTask(void * pvParameters){
  static uint32_t next = millis();
  static uint32_t count = 0;
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  static std::bitset<32> inputs;
  static std::bitset<32> old_inputs;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while(true){
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    old_inputs = sysState.inputs;
    for(int i = 0; i<8; i++){
      setRow(i);
      std::bitset<4> col = readCols();
      for(int j = 0; j<4; j++){
        inputs[i*4+j] = col[j];
      }
    }
    sysState.inputs = inputs;
    xSemaphoreGive(sysState.mutex);
    
    std::bitset<32> changed = inputs ^ old_inputs;

    for(int i = 0; i<12; i++){
      if(changed[i]){
        if(!inputs[i]){
          Serial.print("Pushed: ");
          Serial.println(i);
          TX_Message[0] = 'P';
          TX_Message[1] = octave;
          TX_Message[2] = i;
          applyKeyEvent('P', octave, i);
        }else{
          Serial.print("Released: ");
          Serial.println(i);
          TX_Message[0] = 'R';
          TX_Message[1] = octave;
          TX_Message[2] = i;
          applyKeyEvent('R', octave, i);
        }
        uint8_t* msg_ptr = (uint8_t*) TX_Message;
        xQueueSend(msgOutQ, msg_ptr, 0);
      }
    }

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void displayKeysTask(void * pvParameters){
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(true){
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.setCursor(2,10);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    u8g2.print((char) sysState.RX_Message[0]);
    u8g2.print(sysState.RX_Message[1]);
    u8g2.print(sysState.RX_Message[2]);
    xSemaphoreGive(sysState.mutex);

    u8g2.setCursor(50,10);
    u8g2.print(wave2String(currentWaveform).c_str());

    u8g2.setCursor(2,20);
    std::string key_states = "";
    std::string volume_str;
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    for(int i = 0; i<12; i++){
      key_states += ' ';
      key_states += (sysState.inputs[i] ? '1' : '0');
    }
    xSemaphoreGive(sysState.mutex);
    u8g2.print(key_states.c_str());

    u8g2.setCursor(2,30);
    u8g2.print("Oct: ");
    u8g2.print(octave);

    u8g2.setCursor(50,30);
    volume_str = "Volume: " + std::to_string(knob3Rotation);
    u8g2.print(volume_str.c_str());
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    u8g2.sendBuffer();          // transfer internal memory to the display
    xSemaphoreGive(i2cMutex);

    //Toggle LED
    digitalToggle(LED_BUILTIN);
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void knobTask(void * pvParameters){
  uint8_t prev = 0xFF;
  while(true){
    xSemaphoreTake(knobIntSem, portMAX_DELAY);
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    uint8_t now = pcalReadInputs();

    xSemaphoreGive(i2cMutex);

    // Quadrature decode from PCAL inputs:
    // knob3 A/B: bits 6/7, knob2 A/B: bits 4/5, knob1 A/B: bits 2/3.
    uint8_t prevAB3 = (prev >> 6) & 0x03;
    uint8_t nowAB3 = (now >> 6) & 0x03;
    uint8_t t3 = (prevAB3 << 2) | nowAB3;
    if ((t3 == 0b0001 || t3 == 0b1110) && knob3Rotation < 8) knob3Rotation++;
    if ((t3 == 0b1011 || t3 == 0b0100) && knob3Rotation > 0) knob3Rotation--;

    uint8_t prevAB2 = (prev >> 4) & 0x03;
    uint8_t nowAB2 = (now >> 4) & 0x03;
    uint8_t t2 = (prevAB2 << 2) | nowAB2;
    if (t2 == 0b0001 || t2 == 0b1110) {
      knob2Rotation = (knob2Rotation < 3) ? (knob2Rotation + 1) : 0;
    }
    if (t2 == 0b1011 || t2 == 0b0100) {
      knob2Rotation = (knob2Rotation > 0) ? (knob2Rotation - 1) : 3;
    }

    uint8_t prevAB1 = (prev >> 2) & 0x03;
    uint8_t nowAB1 = (now >> 2) & 0x03;
    uint8_t t1 = (prevAB1 << 2) | nowAB1;
    if ((t1 == 0b0001 || t1 == 0b1110) && knob1Rotation < 8) knob1Rotation++;
    if ((t1 == 0b1011 || t1 == 0b0100) && knob1Rotation > 0) knob1Rotation--;

    switch (knob2Rotation) {
      case 0: currentWaveform = SAWTOOTH; break;
      case 1: currentWaveform = SINE;     break;
      case 2: currentWaveform = SQUARE;   break;
      case 3: currentWaveform = TRIANGLE; break;
    }
    octave = knob1Rotation;

    prev = now;
  }
}

void decodeTask(void * pvParameters){
  uint8_t RX_Message[8] = {0};
  while(true){
    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);
    uint8_t command = RX_Message[0];
    uint8_t octave = RX_Message[1];
    uint8_t key = RX_Message[2];
    applyKeyEvent(command, octave, key);
  }
}

void CAN_TX_Task(void * pvParameters){
	uint8_t msgOut[8];
	while (true) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}
}



void setup() {
  // put your setup code here, to run once:
  analogWriteResolution(12);
  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);
  sysState.mutex = xSemaphoreCreateMutex();

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply
  setOutMuxBit(KNOB_MODE, LOW);  //Read knobs through key matrix

  setOutMuxBit(ALL_BITS, HIGH);  //Enable headphone output
  digitalWrite(REN_PIN, HIGH);
  
  //Initialise UART
  Serial.begin(9600);
  Serial.println("Serial initialized");

  for (int i = 0; i < 4096; i++) {
    double phase = (2.0 * PI * static_cast<double>(i)) / 4096;
    double s = sin(phase);
    double value = 2147483647.0 * s;
    sineLUT[i] = static_cast<int32_t>(value);
  }

  // NCO tuning: A (index 9) = 440 Hz, fs = 22 kHz
  // constexpr double sampleRate = 22000.0;
  // constexpr double semitone = 1.0594630943592953; // 2^(1/12)
  // constexpr double ncoScale = 4294967296.0 / sampleRate; // 2^32 / fs
  // double freq = 440.0 * pow(semitone, -9.0); // index 0 is C
  // for (int i = 0; i < 12; i++) {
  //   stepSizes[i] = static_cast<uint32_t>(freq * ncoScale);
  //   freq *= semitone;
  // }

  constexpr double c4Hz = 261.6255653005986; // Middle C (C4)
  for (int i = 0; i < 12; i++) {
    frequencies[i] = c4Hz * pow(2.0, static_cast<double>(i) / 12.0);
  }

  sampleTimer.setOverflow(sampling_frequency, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &scanKeysHandle );	/* Pointer to store the task handle */

  TaskHandle_t displayKeysHandle = NULL;
  xTaskCreate(
  displayKeysTask,		/* Function that implements the task */
  "displayKeys",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  2,			/* Task priority */
  &displayKeysHandle );	/* Pointer to store the task handle */

  TaskHandle_t decodeHandle = NULL;
  xTaskCreate(
  decodeTask,		/* Function that implements the task */
  "decode",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &decodeHandle );	/* Pointer to store the task handle */

  TaskHandle_t CANTXHandle = NULL;
  xTaskCreate(CAN_TX_Task,"CAN_TX",64,NULL,1,&CANTXHandle );	
  TaskHandle_t knobHandle = NULL;
  xTaskCreate(knobTask,"knob",64,NULL,1,&knobHandle );	

  Wire.begin();
  i2cMutex = xSemaphoreCreateMutex();
  knobIntSem = xSemaphoreCreateBinary();

  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  pcalWriteReg(0x03, 0xFF); // Configuration: all pins input
  pcalWriteReg(0x43, 0xFF); // Pull-up/down enable
  pcalWriteReg(0x44, 0xFF); // Pull-up select
  pcalWriteReg(0x42, 0xFF); // Input latch enable (all pins)
  pcalWriteReg(0x45, 0x00); // Interrupt mask: enable all input interrupts
  (void)pcalReadInputs();   // Clear pending input-change state
  xSemaphoreGive(i2cMutex);
  pinMode(KNOB_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KNOB_INT_PIN), knobIntISR, LOW);

  CAN_Init(false);
  setCANFilter(0x123,0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_Start();


  vTaskStartScheduler();
}

void loop() {

  // put your main code here, to run repeatedly:
}
