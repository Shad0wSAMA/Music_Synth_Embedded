
#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <vector>
#include <STM32FreeRTOS.h>

//Constants
  const uint32_t interval = 100; //Display update interval

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

  uint32_t stepSizes[12];
  volatile uint32_t currentStepSize = 0;
  volatile uint32_t knob3Rotation = 5;

HardwareTimer sampleTimer(TIM1);

struct {
std::bitset<32> inputs; 
std::bitset<32> old_inputs;
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
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN, HIGH);
}

void sampleISR(){
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8-knob3Rotation); //Apply volume control
  analogWrite(OUTR_PIN, Vout + 128);
}

void scanKeysTask(void * pvParameters){
  static uint32_t next = millis();
  static uint32_t count = 0;
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  static std::bitset<32> inputs;
  static std::bitset<32> old_inputs;
  static std::bitset<2> knob3State;
  static std::bitset<2> old_knob3State;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while(true){
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    for(int i = 0; i<8; i++){
      setRow(i);
      std::bitset<4> col = readCols();
      for(int j = 0; j<4; j++){
        sysState.inputs[i*4+j] = col[j];
      }
    }
    inputs = sysState.inputs;
    xSemaphoreGive(sysState.mutex);
    
    std::bitset<32> changed = inputs ^ old_inputs;
    old_inputs = inputs;
    int last_key_pressed;

    bool key_pressed = false;

    for(int i = 0; i<12; i++){
      if(!inputs[i]) key_pressed = true;
      if(changed[i] && !inputs[i]){
        Serial.println(i);
        currentStepSize = stepSizes[i];
      }
    }
    if(!key_pressed) currentStepSize = 0;

    knob3State[0] = inputs[12];
    knob3State[1] = inputs[13];

    switch (knob3State.to_ulong()){
      case 0b00:
        if(old_knob3State == 0b01 && knob3Rotation > 0) knob3Rotation -= 1;
        break;
      case 0b01:
        if(old_knob3State == 0b00 && knob3Rotation < 8) knob3Rotation += 1;
        break;
      case 0b10:
        if(old_knob3State == 0b11 && knob3Rotation < 8) knob3Rotation += 1;
        break;
      case 0b11:
        if(old_knob3State == 0b10 && knob3Rotation > 0) knob3Rotation -= 1;
        break;
    }
    old_knob3State = knob3State;
    

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void displayKeysTask(void * pvParameters){
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(true){
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(1,10,"Hello World!");  // write something to the internal memory
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
    u8g2.print(currentStepSize);
    u8g2.setCursor(50,30);
    volume_str = "Volume: " + std::to_string(knob3Rotation);
    u8g2.print(volume_str.c_str());
    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}




void setup() {
  // put your setup code here, to run once:

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
  setOutMuxBit(KNOB_MODE, HIGH);  //Read knobs through key matrix

  setOutMuxBit(ALL_BITS, HIGH);  //Enable headphone output
  digitalWrite(REN_PIN, HIGH);
  
  //Initialise UART
  Serial.begin(9600);
  // NCO tuning: A (index 9) = 440 Hz, fs = 22 kHz
  constexpr double sampleRate = 22000.0;
  constexpr double semitone = 1.0594630943592953; // 2^(1/12)
  constexpr double ncoScale = 4294967296.0 / sampleRate; // 2^32 / fs
  double freq = 440.0 * pow(semitone, -9.0); // index 0 is C
  for (int i = 0; i < 12; i++) {
    stepSizes[i] = static_cast<uint32_t>(freq * ncoScale);
    freq *= semitone;
  }

  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

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
  1,			/* Task priority */
  &displayKeysHandle );	/* Pointer to store the task handle */

  sysState.mutex = xSemaphoreCreateMutex();



  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
}
