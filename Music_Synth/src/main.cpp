#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <cstddef>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

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

//Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

HardwareTimer sampleTimer(TIM1);
volatile uint32_t currentStepSize = 0;
/// volatile uint8_t TX_Message[8] = {0};

QueueHandle_t msgInQ;
volatile uint8_t RX_Message[8] = {0};
QueueHandle_t msgOutQ;
SemaphoreHandle_t CAN_TX_Semaphore;
SemaphoreHandle_t recordMutex;
uint32_t SEND_ID = 0x123;
uint32_t RECEIVE_ID = 0x123;

// ---- Loudness compensation ----
volatile int8_t currentNoteIdx = -1;   // -1 = no key pressed

// C..B 的补偿（Q8.8：256=1.0倍）
// 低音更大，高音更小（你可以后面微调）
const uint16_t noteGainQ88[12] = {
  460, 450, 440, 430,
  420, 410, 400, 390,
  380, 360, 340, 320
};

// octaveShift: -3..+3 的补偿（Q8.8）
// 下移八度更大，上移八度更小（更符合你“低频太小”的现象）
const uint16_t octGainQ88[7] = {
  640, 520, 400, 256, 224, 208, 192
  // os=-3  -2   -1    0    +1   +2   +3
};

enum Waveform : uint8_t {
  W_SAW=0,
  W_SQUARE=1,
  W_TRI=2,
  W_SINE=3,
  W_SAW_SINE=4,
  W_SQUARE_SINE=5,
  W_DETUNE_SQUARE=6,
  W_DETUNE_SAW=7,
  W_SUPER_SAW=8
};

// Per-wave loudness compensation (Q8.8; 256=1.0x)
const uint16_t waveGainQ88[9] = {
  256, // W_SAW
  200, // W_SQUARE  (方波 RMS 高，适当小一点)
  330, // W_TRI     (三角 RMS 低，拉大一点)
  320, // W_SINE    (正弦 RMS 更低，再大一点)
  280, // W_SAW_SINE
  240, // W_SQUARE_SINE
  220, // W_DETUNE_SQUARE
  260, // W_DETUNE_SAW
  220  // W_SUPER_SAW (叠加更厚，别太大)
};

constexpr int NVOICE = 4;


struct Voice {
  uint32_t step;    // 已经包含 octave + bend（scan task 算好）
  uint32_t phase;   // 主相位
  uint32_t phase2;  // 仅主音用 detune/supersaw
  uint32_t phase3;  // 仅主音用 detune/supersaw
};

volatile Voice voices[NVOICE];

volatile uint16_t joyYRaw = 520;   // 你测到中心是520

volatile uint8_t waveSel   = W_SAW;   // 0..3
volatile uint8_t toneK     = 0;       // 0..8 (滤波强度/闷度)
volatile int8_t octaveShift = 0;  // 以 1 为单位：+1=上移一八度，-1=下移一八度
volatile uint16_t joyXRaw = 520;
volatile uint8_t joyMode = 0;     // 0=vibrato, 1=portamento
volatile uint8_t knob3Rotation = 7; // volume 0..8
volatile uint8_t knob2Rotation = 0; // tone/filter 0..8
volatile uint8_t knob1Rotation = 4; // duty 0..8
volatile uint8_t knob0Rotation = 0; // wave select 0..8
volatile uint16_t localPressedMaskByOct[8] = {0};  // local key states by octave
volatile uint16_t remotePressedMaskByOct[8] = {0}; // bit i = remote key i pressed at octave index
// （你已有 joyXRaw）

const char* noteNames[12] = {
  "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};

constexpr uint8_t INPUT_KNOB2S = 20;
constexpr uint8_t INPUT_KNOB3S = 21;
constexpr size_t MAX_RECORDED_EVENTS = 1024;

enum RecordSource : uint8_t {
  RECORD_FROM_TX = 0,
  RECORD_FROM_RX = 1
};

struct RecordedCanEvent {
  uint32_t relTimeMs;
  uint8_t msg[3];
  uint8_t source;
};

RecordedCanEvent recordedEvents[MAX_RECORDED_EVENTS];
size_t recordedEventCount = 0;
size_t playbackEventIndex = 0;
volatile bool isRecording = false;
volatile bool isPlayingBack = false;
volatile uint32_t recordStartMs = 0;
volatile uint32_t playbackStartMs = 0;

static inline void applyKeyEvent(volatile uint16_t *maskByOct, const uint8_t command, const uint8_t octave, const uint8_t key);
bool queueCANCommand(const uint8_t command, const uint8_t arg1, const uint8_t arg2);

static inline void handleCommandMessage(const uint8_t *msg) {
  char type = (char)msg[0];
  if (type == 'P' || type == 'R') {
    applyKeyEvent(remotePressedMaskByOct, msg[0], msg[1], msg[2]);
  } else if (type == 'V') {
    uint8_t v = msg[1];
    if (v > 8) v = 8;
    __atomic_store_n(&knob3Rotation, v, __ATOMIC_RELAXED);
  } else if (type == 'W') {
    uint8_t t = msg[1];
    if (t > 8) t = 8;
    __atomic_store_n(&knob2Rotation, t, __ATOMIC_RELAXED);
    __atomic_store_n(&toneK, t, __ATOMIC_RELAXED);
  } else if (type == 'D') {
    uint8_t d = msg[1];
    if (d > 8) d = 8;
    __atomic_store_n(&knob1Rotation, d, __ATOMIC_RELAXED);
  } else if (type == 'M') {
    uint8_t m = msg[1];
    if (m > 8) m = 8;
    __atomic_store_n(&knob0Rotation, m, __ATOMIC_RELAXED);
    __atomic_store_n(&waveSel, m, __ATOMIC_RELAXED);
  } else if (type == 'J') {
    uint8_t jm = msg[1] & 0x01;
    __atomic_store_n(&joyMode, jm, __ATOMIC_RELAXED);
  }
}

static void recordCanEvent(const uint8_t *msg, const RecordSource source) {
  if (!recordMutex) return;

  xSemaphoreTake(recordMutex, portMAX_DELAY);
  if (isRecording && !isPlayingBack && recordedEventCount < MAX_RECORDED_EVENTS) {
    RecordedCanEvent &evt = recordedEvents[recordedEventCount++];
    evt.relTimeMs = millis() - recordStartMs;
    evt.msg[0] = msg[0];
    evt.msg[1] = msg[1];
    evt.msg[2] = msg[2];
    evt.source = (uint8_t)source;
  }
  xSemaphoreGive(recordMutex);
}

static void startRecordingSession() {
  xSemaphoreTake(recordMutex, portMAX_DELAY);
  bool wasPlaying = isPlayingBack;
  recordedEventCount = 0;
  playbackEventIndex = 0;
  recordStartMs = millis();
  isPlayingBack = false;
  isRecording = true;
  xSemaphoreGive(recordMutex);

  if (wasPlaying) {
    for (uint8_t oct = 0; oct < 8; oct++) {
      __atomic_store_n(&remotePressedMaskByOct[oct], 0, __ATOMIC_RELAXED);
    }
    Serial.println("[PLAYBACK STOP] reason=recording_started");
  }
}

static void stopRecordingSession() {
  xSemaphoreTake(recordMutex, portMAX_DELAY);
  isRecording = false;
  xSemaphoreGive(recordMutex);
}

static void stopPlaybackSession(const char *reason) {
  xSemaphoreTake(recordMutex, portMAX_DELAY);
  uint32_t elapsed = millis() - playbackStartMs;
  size_t idx = playbackEventIndex;
  size_t total = recordedEventCount;
  isPlayingBack = false;
  playbackEventIndex = 0;
  for (uint8_t oct = 0; oct < 8; oct++) {
    __atomic_store_n(&remotePressedMaskByOct[oct], 0, __ATOMIC_RELAXED);
  }
  xSemaphoreGive(recordMutex);

  Serial.print("[PLAYBACK STOP] reason=");
  Serial.print(reason);
  Serial.print(" elapsed_ms=");
  Serial.print((unsigned long)elapsed);
  Serial.print(" idx=");
  Serial.print((unsigned long)idx);
  Serial.print("/");
  Serial.println((unsigned long)total);
}

static bool startPlaybackSession() {
  bool started = false;
  size_t total = 0;
  xSemaphoreTake(recordMutex, portMAX_DELAY);
  total = recordedEventCount;
  if (!isRecording && recordedEventCount > 0) {
    isPlayingBack = true;
    playbackEventIndex = 0;
    playbackStartMs = millis();
    started = true;
  }
  xSemaphoreGive(recordMutex);
  if (started) {
    Serial.print("[PLAYBACK START] events=");
    Serial.println((unsigned long)total);
  } else {
    Serial.print("[PLAYBACK NOT STARTED] isRecording=");
    Serial.print((int)__atomic_load_n(&isRecording, __ATOMIC_RELAXED));
    Serial.print(" events=");
    Serial.println((unsigned long)total);
  }
  return started;
}

static void processPlaybackEvents() {
  if (!recordMutex) return;

  while (true) {
    RecordedCanEvent evt;
    bool hasDueEvent = false;
    bool done = false;
    bool playbackFinished = false;

    xSemaphoreTake(recordMutex, portMAX_DELAY);
    if (!isPlayingBack) {
      xSemaphoreGive(recordMutex);
      return;
    }

    uint32_t elapsed = millis() - playbackStartMs;
    if (playbackEventIndex < recordedEventCount &&
        recordedEvents[playbackEventIndex].relTimeMs <= elapsed) {
      evt = recordedEvents[playbackEventIndex++];
      hasDueEvent = true;
    } else if (playbackEventIndex >= recordedEventCount) {
      playbackFinished = true;
      done = true;
    }
    xSemaphoreGive(recordMutex);

    if (done) {
      if (playbackFinished) {
        stopPlaybackSession("sequence_end");
      }
      return;
    }
    if (!hasDueEvent) return;

    handleCommandMessage(evt.msg);
    queueCANCommand(evt.msg[0], evt.msg[1], evt.msg[2]);
  }
}

bool queueCANCommand(const uint8_t command, const uint8_t arg1, const uint8_t arg2) {
  uint8_t msg[8] = {0};
  msg[0] = command;
  msg[1] = arg1;
  msg[2] = arg2;
  bool queued = (xQueueSend(msgOutQ, msg, 0) == pdTRUE);
  if (queued) {
    recordCanEvent(msg, RECORD_FROM_TX);
  }
  return queued;
}

std::bitset<4> readCols() {

  std::bitset<4> result;
  /*
  // ---- Select Row 0 ----
  digitalWrite(REN_PIN, LOW);   // Disable row select while changing address

  digitalWrite(RA0_PIN, LOW);   // RA2 RA1 RA0 = 000
  digitalWrite(RA1_PIN, LOW);
  digitalWrite(RA2_PIN, LOW);

  digitalWrite(REN_PIN, HIGH);  // Enable row (drives R0 low)
  */
  
  // ---- Read columns ----
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);

  return result;
}
constexpr float FS = 22000.0f;
constexpr double TWO32 = 4294967296.0; // 2^32


struct {
    std::bitset<32> inputs;
    SemaphoreHandle_t mutex;
} sysState;

constexpr float noteFreq[12] = {
  261.6256f, 277.1826f, 293.6648f, 311.1270f,
  329.6276f, 349.2282f, 369.9944f, 391.9954f,
  415.3047f, 440.0000f, 466.1638f, 493.8833f
};

void setRow(uint8_t rowIdx) {
  digitalWrite(REN_PIN, LOW);            // disable to avoid glitches

  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, (rowIdx >> 1) & 0x01);
  digitalWrite(RA2_PIN, (rowIdx >> 2) & 0x01);

  digitalWrite(REN_PIN, HIGH);           // enable again
}

/*
static inline int16_t renderWave(uint8_t mode, uint32_t step, Voice &v) {
  v.phase += step;
  uint8_t p = (uint8_t)(v.phase >> 24);
  int16_t x = 0;

  switch (mode) {
    case W_SAW: {
      int16_t s = (int16_t)p - 128;
      s = s * 2;
      x = clamp8(s);
      break;
    }
    case W_SQUARE:
      x = (v.phase & 0x80000000u) ? 127 : -128;
      break;

    case W_TRI: {
      uint8_t triU = (p < 128) ? (p << 1) : ((255 - p) << 1);
      int16_t s = (int16_t)triU - 128;
      s = s * 2;
      x = clamp8(s);
      break;
    }

    case W_SINE: {
      int16_t s = sineTable[p];
      s = s * 3;
      x = clamp8(s);
      break;
    }

    case W_SAW_SINE: {
      int16_t saw = (int16_t)p - 128;
      int16_t sin = (int16_t)sineTable[p];
      x = (3*saw + sin) / 4;
      break;
    }

    case W_SQUARE_SINE: {
      int16_t sq  = (v.phase & 0x80000000u) ? 127 : -128;
      int16_t sin = (int16_t)sineTable[p];
      x = (sq + sin) / 2;
      break;
    }

    // detune/supersaw 先不做，或降级
    default:
      x = (int16_t)p - 128;
      break;
  }
  return x;
}
*/

const uint32_t stepSizes[12] = {
  (uint32_t)(TWO32 * noteFreq[0]  / FS),
  (uint32_t)(TWO32 * noteFreq[1]  / FS),
  (uint32_t)(TWO32 * noteFreq[2]  / FS),
  (uint32_t)(TWO32 * noteFreq[3]  / FS),
  (uint32_t)(TWO32 * noteFreq[4]  / FS),
  (uint32_t)(TWO32 * noteFreq[5]  / FS),
  (uint32_t)(TWO32 * noteFreq[6]  / FS),
  (uint32_t)(TWO32 * noteFreq[7]  / FS),
  (uint32_t)(TWO32 * noteFreq[8]  / FS),
  (uint32_t)(TWO32 * noteFreq[9]  / FS),   // A4 index 9
  (uint32_t)(TWO32 * noteFreq[10] / FS),
  (uint32_t)(TWO32 * noteFreq[11] / FS)

};

// 256-point sine table, range -128..127
#include <math.h>

int8_t sineTable[256];

void initSineTable() {
    for (int i = 0; i < 256; i++) {
        float angle = 2.0f * M_PI * i / 256.0f;
        sineTable[i] = (int8_t)(127.0f * sinf(angle));
    }
};

static inline int16_t clamp8(int16_t s){
  if (s > 127) return 127;
  if (s < -128) return -128;
  return s;
}

static inline int16_t softClip8(int16_t s) {
  int16_t a = (s < 0) ? -s : s;
  int16_t denom = 128 + a;
  return (s * 128) / denom;
}

static inline void applyKeyEvent(volatile uint16_t *maskByOct, const uint8_t command, const uint8_t octave, const uint8_t key) {
  if (octave >= 8 || key >= 12) return;
  uint16_t bit = (uint16_t)(1u << key);
  uint16_t mask = __atomic_load_n(&maskByOct[octave], __ATOMIC_RELAXED);
  if (command == 'P') {
    mask |= bit;
  } else if (command == 'R') {
    mask &= (uint16_t)(~bit);
  } else {
    return;
  }
  __atomic_store_n(&maskByOct[octave], mask, __ATOMIC_RELAXED);
}

void scanKeysTask(void * pvParameters) {

  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  std::bitset<12> prevKeys;
  prevKeys.set();   // 未按=1，按下=0

  static std::bitset<32> prevInputs;
  static bool prevInputsValid = false;
  static uint8_t localKeyOctave[12] = {4,4,4,4,4,4,4,4,4,4,4,4};

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // ---- scan keys rows 0..2 ----
    std::bitset<32> localInputs;
    localInputs.reset();

    for (uint8_t row = 0; row < 7; row++) {   // 0..6
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> cols = readCols();
      for (uint8_t c = 0; c < 4; c++) {
        localInputs[row * 4 + c] = cols[c];
      }
    }

    // ---- joystick press (matrix row5 col2 -> index 22) : toggle mode ----
    static bool lastJoyPressed = false;
    static TickType_t lastToggleTick = 0;

    bool joyPressed = (localInputs[22] == 0);  // active low

    if (!lastJoyPressed && joyPressed) {       // falling edge: not pressed -> pressed
      TickType_t now = xTaskGetTickCount();
      if (now - lastToggleTick > (200 / portTICK_PERIOD_MS)) { // debounce 200ms
        lastToggleTick = now;
        uint8_t m = __atomic_load_n(&joyMode, __ATOMIC_RELAXED);
        m ^= 1;
        __atomic_store_n(&joyMode, m, __ATOMIC_RELAXED);
        queueCANCommand('J', m, 0);
      }
    }
    lastJoyPressed = joyPressed;

    // ---- record / playback button handling ----
    std::bitset<32> changedInputs;
    changedInputs.reset();
    if (prevInputsValid) {
      changedInputs = localInputs ^ prevInputs;
    }

    bool playbackWasActive = __atomic_load_n(&isPlayingBack, __ATOMIC_RELAXED);
    bool playbackKeyPressedEdge = prevInputsValid &&
      changedInputs[INPUT_KNOB3S] &&
      (localInputs[INPUT_KNOB3S] == 0);

    if (prevInputsValid && playbackWasActive && playbackKeyPressedEdge) {
      stopPlaybackSession("playback_key_pressed");
    }

    if (prevInputsValid && changedInputs[INPUT_KNOB2S] && (localInputs[INPUT_KNOB2S] == 0)) {
      bool recNow = __atomic_load_n(&isRecording, __ATOMIC_RELAXED);
      if (recNow) {
        stopRecordingSession();
      } else {
        startRecordingSession();
      }
    }

    if (prevInputsValid && playbackKeyPressedEdge && !playbackWasActive) {
      startPlaybackSession();
    }

    // ---- compare + send CAN ----
    for (int k = 0; k < 12; k++) {
      bool prev = prevKeys[k];
      bool curr = localInputs[k];

      if (prev != curr) {
        if (curr == 0) {
          int8_t osForTx = __atomic_load_n(&octaveShift, __ATOMIC_RELAXED);
          uint8_t octaveForTx = (uint8_t)(4 + osForTx); // transmit real current octave
          localKeyOctave[k] = octaveForTx;
          applyKeyEvent(localPressedMaskByOct, 'P', octaveForTx, (uint8_t)k);
          queueCANCommand('P', octaveForTx, (uint8_t)k);
        } else {
          applyKeyEvent(localPressedMaskByOct, 'R', localKeyOctave[k], (uint8_t)k);
          queueCANCommand('R', localKeyOctave[k], (uint8_t)k);
        }
      }
    }

    for (int k = 0; k < 12; k++) prevKeys[k] = localInputs[k];

    // ---- knobs (统一方向) ----
    int k3 = (int)__atomic_load_n(&knob3Rotation, __ATOMIC_RELAXED);   // volume
    int k2 = (int)__atomic_load_n(&knob2Rotation, __ATOMIC_RELAXED);   // tone
    int k1 = (int)__atomic_load_n(&knob1Rotation, __ATOMIC_RELAXED);   // duty
    int k0 = (int)__atomic_load_n(&knob0Rotation, __ATOMIC_RELAXED);   // wave

    int oldK3 = k3;
    int oldK2 = k2;
    int oldK1 = k1;
    int oldK0 = k0;

    if (prevInputsValid) {
      uint8_t prevAB3 = (static_cast<uint8_t>(prevInputs[12]) << 1) | static_cast<uint8_t>(prevInputs[13]);
      uint8_t nowAB3 = (static_cast<uint8_t>(localInputs[12]) << 1) | static_cast<uint8_t>(localInputs[13]);
      uint8_t t3 = (prevAB3 << 2) | nowAB3;
      if ((t3 == 0b0001 || t3 == 0b1110) && k3 > 0) k3--;
      if ((t3 == 0b1011 || t3 == 0b0100) && k3 < 8) k3++;

      uint8_t prevAB2 = (static_cast<uint8_t>(prevInputs[14]) << 1) | static_cast<uint8_t>(prevInputs[15]);
      uint8_t nowAB2 = (static_cast<uint8_t>(localInputs[14]) << 1) | static_cast<uint8_t>(localInputs[15]);
      uint8_t t2 = (prevAB2 << 2) | nowAB2;
      if ((t2 == 0b0001 || t2 == 0b1110) && k2 > 0) k2--;
      if ((t2 == 0b1011 || t2 == 0b0100) && k2 < 8) k2++;

      uint8_t prevAB1 = (static_cast<uint8_t>(prevInputs[16]) << 1) | static_cast<uint8_t>(prevInputs[17]);
      uint8_t nowAB1 = (static_cast<uint8_t>(localInputs[16]) << 1) | static_cast<uint8_t>(localInputs[17]);
      uint8_t t1 = (prevAB1 << 2) | nowAB1;
      if ((t1 == 0b0001 || t1 == 0b1110) && k1 > 0) k1--;
      if ((t1 == 0b1011 || t1 == 0b0100) && k1 < 8) k1++;

      uint8_t prevAB0 = (static_cast<uint8_t>(prevInputs[18]) << 1) | static_cast<uint8_t>(prevInputs[19]);
      uint8_t nowAB0 = (static_cast<uint8_t>(localInputs[18]) << 1) | static_cast<uint8_t>(localInputs[19]);
      uint8_t t0 = (prevAB0 << 2) | nowAB0;
      if ((t0 == 0b0001 || t0 == 0b1110) && k0 > 0) k0--;
      if ((t0 == 0b1011 || t0 == 0b0100) && k0 < 8) k0++;
    }

    prevInputs = localInputs;
    prevInputsValid = true;

    __atomic_store_n(&knob3Rotation, (uint8_t)k3, __ATOMIC_RELAXED);
    __atomic_store_n(&knob2Rotation, (uint8_t)k2, __ATOMIC_RELAXED);
    __atomic_store_n(&knob1Rotation, (uint8_t)k1, __ATOMIC_RELAXED);
    __atomic_store_n(&knob0Rotation, (uint8_t)k0, __ATOMIC_RELAXED);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = localInputs;
    xSemaphoreGive(sysState.mutex);
 
    // ---- map knobs -> synth params ----
    uint8_t ws = (uint8_t)k0;   // k0 已经 clamp 到 0..8
    __atomic_store_n(&waveSel, ws, __ATOMIC_RELAXED);


    __atomic_store_n(&toneK, (uint8_t)k2, __ATOMIC_RELAXED);

    if (k3 != oldK3) {
      queueCANCommand('V', (uint8_t)k3, 0);
    }
    if (k2 != oldK2) {
      queueCANCommand('W', (uint8_t)k2, 0);
    }
    if (k1 != oldK1) {
      queueCANCommand('D', (uint8_t)k1, 0);
    }
    if (k0 != oldK0) {
      queueCANCommand('M', (uint8_t)k0, 0);
    }

    // ---- joystick octave shift (discrete) ----
    static uint8_t joyState = 0; // 0=center, 1=left, 2=right

    // ---- joystick analog ----
    uint16_t xRaw = (uint16_t)analogRead(JOYX_PIN);
    uint16_t yRaw = (uint16_t)analogRead(JOYY_PIN);

    __atomic_store_n(&joyXRaw, xRaw, __ATOMIC_RELAXED);
    __atomic_store_n(&joyYRaw, yRaw, __ATOMIC_RELAXED);

    // ---- pitch bend (X): map to [-2, +2] semitone, with deadzone ----
    const int32_t JOY_CENTER = 520;
    const int32_t JOY_HALFSPAN = 520;   // 大约到边界
    const int32_t DEAD = 25;            // 先给大一点，避免抖

    int32_t dx = (int32_t)xRaw - JOY_CENTER;
    if (dx > -DEAD && dx < DEAD) dx = 0;

    float bend = (float)dx / (float)JOY_HALFSPAN * 2.0f;   // ±2 semitone
    if (bend >  2.0f) bend =  2.0f;
    if (bend < -2.0f) bend = -2.0f;

    float bendRatio = powf(2.0f, bend / 12.0f);

    const uint16_t LEFT_TH  = 760;
    const uint16_t RIGHT_TH = 260;

    uint8_t newState = 0;
    if (xRaw <= RIGHT_TH) newState = 2;      // ✅ right
    else if (xRaw >= LEFT_TH) newState = 1;  // ✅ left
    else newState = 0;

    // center -> left/right 触发一次
    if (joyState == 0 && newState == 1) {
      int8_t os = __atomic_load_n(&octaveShift, __ATOMIC_RELAXED);
      if (os > -3) os -= 1;   // 左：降八度
      __atomic_store_n(&octaveShift, os, __ATOMIC_RELAXED);
    }
    if (joyState == 0 && newState == 2) {
      int8_t os = __atomic_load_n(&octaveShift, __ATOMIC_RELAXED);
      if (os < 3) os += 1;    // 右：升八度
      __atomic_store_n(&octaveShift, os, __ATOMIC_RELAXED);
    }

    joyState = newState;

    // ---- choose note ----
    // ---- collect pressed keys -> voices (polyphony) ----
    uint32_t steps[NVOICE] = {0};

    // 这里用“从左到右”取最多 NVOICE 个键，你也可以反过来
    int n = 0;
    int8_t mainNoteIdx = -1;

    uint16_t combinedMasks[8];
    for (uint8_t oct = 0; oct < 8; oct++) {
      uint16_t localMask = __atomic_load_n(&localPressedMaskByOct[oct], __ATOMIC_RELAXED);
      uint16_t remoteMask = __atomic_load_n(&remotePressedMaskByOct[oct], __ATOMIC_RELAXED);
      combinedMasks[oct] = (uint16_t)(localMask | remoteMask);
    }

    for (uint8_t oct = 0; oct < 8 && n < NVOICE; oct++) {
      uint16_t mask = combinedMasks[oct];
      if (mask == 0) continue;

      for (int i = 0; i < 12 && n < NVOICE; i++) {
        if ((mask & (uint16_t)(1u << i)) == 0) continue;

        uint32_t st = stepSizes[i];
        mainNoteIdx = (mainNoteIdx < 0) ? (int8_t)i : mainNoteIdx;

        if (oct > 4) st <<= (oct - 4);
        else if (oct < 4) st >>= (4 - oct);

        st = (uint32_t)((float)st * bendRatio);
        steps[n++] = st;
      }
    }

    __atomic_store_n(&currentNoteIdx, mainNoteIdx, __ATOMIC_RELAXED);

    // 原子写入 voices steps（phase 保持不动，避免“相位跳”）
    for (int v = 0; v < NVOICE; v++) {
      __atomic_store_n(&voices[v].step, steps[v], __ATOMIC_RELAXED);
    }

    processPlaybackEvents();
  }
}


void sampleISR() {
  static int32_t lp = 0;                // master filter state
  static uint32_t smoothStep0 = 0;      // portamento only for voice0
  static uint32_t vibPhase0 = 0;        // vibrato LFO only for voice0

  uint8_t mode = __atomic_load_n(&waveSel, __ATOMIC_RELAXED);
  uint8_t k    = __atomic_load_n(&toneK, __ATOMIC_RELAXED);

  // joystick Y (10-bit, center=520)
  uint16_t yRaw = __atomic_load_n(&joyYRaw, __ATOMIC_RELAXED);
  int32_t dy = (int32_t)yRaw - 520;
  if (dy > -60 && dy < 60) dy = 0;      // deadzone

  uint32_t ady = (dy < 0) ? -dy : dy;
  uint32_t depth = 0;
  const uint32_t START = 60;            // 门槛：静止不抖
  if (ady > START) {
    depth = (ady - START) * 255 / (520 - START);
    if (depth > 255) depth = 255;
  }

  uint8_t jm = __atomic_load_n(&joyMode, __ATOMIC_RELAXED);

  int32_t mix = 0;
  int activeCount = 0;

  // ========= Voice loop =========
  for (int v = 0; v < NVOICE; v++) {
    uint32_t st = __atomic_load_n(&voices[v].step, __ATOMIC_RELAXED);
    if (st == 0) continue;

    activeCount++;

    // ----- only voice0 has port/vib + detune/supersaw -----
    if (v == 0) {
      // portamento on step
      if (jm == 1) {
        if (smoothStep0 == 0) smoothStep0 = st;

        // Y down = slower glide, Y up = faster glide
        uint8_t sh;
        if (dy >= 0) sh = 8 + (uint8_t)((uint32_t)dy * 8 / 503);   // 8..16 slow
        else         sh = 3 + (uint8_t)((uint32_t)(-dy) * 5 / 520); // 3..8 fast

        int32_t err = (int32_t)st - (int32_t)smoothStep0;
        smoothStep0 += (uint32_t)(err >> sh);
        st = smoothStep0;
      } else {
        smoothStep0 = st; // vib 模式下别让 smoothStep0 漂着
      }

      // vibrato on step
      if (jm == 0 && depth > 0) {
        constexpr uint32_t VIB_STEP = (uint32_t)(4294967296.0 * 6.5 / 22000.0);
        vibPhase0 += VIB_STEP;
        int16_t s = sineTable[(uint8_t)(vibPhase0 >> 24)];
        int32_t mod = (int32_t)st * (int32_t)s * (int32_t)depth;
        st = (uint32_t)((int32_t)st + (mod >> 20));  // 明显一点
      }

      // store back step used this sample (optional)
      // (not storing to voices to avoid extra atomics)
    }

    // ----- waveform render -----
    // phase update
    voices[v].phase += st;
    uint8_t p = (uint8_t)(voices[v].phase >> 24);

    int16_t x = 0;

    // detune/supersaw only for v==0, otherwise downgrade
    if (v == 0 && (mode == W_DETUNE_SAW || mode == W_DETUNE_SQUARE || mode == W_SUPER_SAW)) {
      uint32_t det = st >> 7;

      if (mode == W_DETUNE_SAW) {
        voices[v].phase2 += (st + det);
        uint8_t p2 = (uint8_t)(voices[v].phase2 >> 24);

        int16_t s1 = (int16_t)p  - 128;
        int16_t s2 = (int16_t)p2 - 128;
        x = (s1 + s2) / 2;

      } else if (mode == W_DETUNE_SQUARE) {
        voices[v].phase2 += (st + det);
        int16_t sq1 = (voices[v].phase  & 0x80000000u) ? 127 : -128;
        int16_t sq2 = (voices[v].phase2 & 0x80000000u) ? 127 : -128;
        x = (sq1 + sq2) / 2;

      } else { // W_SUPER_SAW
        voices[v].phase2 += st + det;
        voices[v].phase3 += st - det;

        uint8_t p2 = (uint8_t)(voices[v].phase2 >> 24);
        uint8_t p3 = (uint8_t)(voices[v].phase3 >> 24);

        int16_t s1 = (int16_t)p  - 128;
        int16_t s2 = (int16_t)p2 - 128;
        int16_t s3 = (int16_t)p3 - 128;
        x = (s1 + s2 + s3) / 3;
      }
    } else {
      // normal modes for all voices
      switch (mode) {
        case W_SAW: {
          int16_t s = (int16_t)p - 128;
          s = s * 2;
          x = clamp8(s);
          break;
        }
        case W_SQUARE:
          x = (voices[v].phase & 0x80000000u) ? 127 : -128;
          break;

        case W_TRI: {
          uint8_t triU = (p < 128) ? (p << 1) : ((255 - p) << 1);
          int16_t s = (int16_t)triU - 128;
          s = s * 2;
          x = clamp8(s);
          break;
        }
        case W_SINE: {
          int16_t s = sineTable[p];
          s = s * 3;
          x = clamp8(s);
          break;
        }
        case W_SAW_SINE: {
          int16_t saw = (int16_t)p - 128;
          int16_t sin = (int16_t)sineTable[p];
          x = (3*saw + sin) / 4;
          break;
        }
        case W_SQUARE_SINE: {
          int16_t sq  = (voices[v].phase & 0x80000000u) ? 127 : -128;
          int16_t sin = (int16_t)sineTable[p];
          x = (sq + sin) / 2;
          break;
        }
        default: {
          int16_t s = (int16_t)p - 128;
          x = s;
          break;
        }
      }
    }

    // apply per-wave loudness gain BEFORE mixing
    uint16_t gW = waveGainQ88[mode];      // Q8.8
    int32_t x2 = (int32_t)x * (int32_t)gW;
    x = (int16_t)(x2 >> 8);              // back to int16
    x = clamp8(x);

    mix += x;
  }

  // ========= Output =========
  if (activeCount == 0) {
    lp = 0;
    smoothStep0 = 0;
    // vibPhase0 不清也行
    analogWrite(OUTR_PIN, 128);
    return;
  }

  int32_t mixScaled = (mix >> 2);
  if (mixScaled > 127) mixScaled = 127;
  if (mixScaled < -128) mixScaled = -128;
  int16_t x = (int16_t)mixScaled;

  // master tone low-pass (your original)
  
  if (k > 0) {
    uint8_t alpha = 255 - k * 28;
    lp = lp + ((alpha * (x - lp)) >> 8);
    x = lp;

    int16_t s = x * (int16_t)(8 + k);
    x = s / 8;
    if (x > 127) x = 127;
    if (x < -128) x = -128;
  } 

  // volume + clip
  int volume = (int)__atomic_load_n(&knob3Rotation, __ATOMIC_RELAXED);
  x = x >> (8-volume);
  x = softClip8(x);

  analogWrite(OUTR_PIN, (uint8_t)(x + 128));
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

void displayUpdateTask(void * pvParameters) {

  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // ✅ 1) 先在锁内拷贝一份 inputs
    std::bitset<32> inputsCopy;
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    inputsCopy = sysState.inputs;
    xSemaphoreGive(sysState.mutex);
    // display
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);


    // show inputs (12-bit hex)
    u8g2.setCursor(0, 20);
    u8g2.print((unsigned long)(inputsCopy.to_ulong() & 0xFFF), HEX);

    // show selected note name
    int selectedKey = -1;
    for (int i = 0; i < 12; i++) {
      if (sysState.inputs[i] == 0) selectedKey = i;
    }

    u8g2.setCursor(0, 30);
    u8g2.print("Note=");
    if (selectedKey >= 0) u8g2.print(noteNames[selectedKey]);
    else u8g2.print("--");


    int knobCopy = (int)__atomic_load_n(&knob3Rotation, __ATOMIC_RELAXED);

    u8g2.setCursor(95, 30);
    u8g2.print("Vol=");
    u8g2.print(knobCopy);


    int8_t os = __atomic_load_n(&octaveShift, __ATOMIC_RELAXED);
    u8g2.setCursor(90, 10);
    u8g2.print("OCT=");
    u8g2.print((int)os);

    bool recOn = __atomic_load_n(&isRecording, __ATOMIC_RELAXED);
    bool playOn = __atomic_load_n(&isPlayingBack, __ATOMIC_RELAXED);

    u8g2.setCursor(50, 20);
    if (playOn) u8g2.print("PLAY");
    else if (recOn) u8g2.print("REC ");
    else u8g2.print("    ");

    uint8_t jm = __atomic_load_n(&joyMode, __ATOMIC_RELAXED);
    u8g2.setCursor(50, 30);
    u8g2.print(jm ? "PORT" : "VIB ");

    
    u8g2.setCursor(100, 20);
    u8g2.print((char)RX_Message[0]);
    u8g2.print(RX_Message[1]);
    u8g2.print(RX_Message[2]);

    
    // ---- Show waveform / duty / tone ----
    u8g2.setCursor(0, 10);   // 你原来(0,10)在写Hello World，这里你也可以换位置
    // 如果不想覆盖 Hello World，就用别的位置，比如 (2, 10) 之前你写了 drawStr
    // 建议用 (2, 10) 这行放 Wave 信息，把 Hello World 去掉更清爽

    uint8_t m = __atomic_load_n(&waveSel, __ATOMIC_RELAXED);
    uint8_t t = __atomic_load_n(&toneK, __ATOMIC_RELAXED);

    ///// u8g2.print("W=");
    if (m == W_SAW) u8g2.print("SAWTOOTH");
    else if (m == W_SQUARE) u8g2.print("SQUARE");
    else if (m == W_TRI) u8g2.print("TRIANGLE");
    else if (m == W_SINE) u8g2.print("SINE");
    else if (m == W_SAW_SINE) u8g2.print("SAW+SIN");
    else if (m == W_SQUARE_SINE) u8g2.print("SQR+SIN");
    else if (m == W_DETUNE_SAW) u8g2.print("detSAW");
    else if (m == W_DETUNE_SQUARE) u8g2.print("detSQR");
    else if (m == W_SUPER_SAW) u8g2.print("SUPERSAW");
    else u8g2.print("RES");


    u8g2.print(" T=");
    u8g2.print(t);

    u8g2.sendBuffer();

    // LED toggle (讲义要求保留)
    digitalToggle(LED_BUILTIN);
  }
}

void CAN_RX_ISR(void) {
    uint8_t rx_isr[8];
    uint32_t ID;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    CAN_RX(ID, rx_isr);
    xQueueSendFromISR(msgInQ, rx_isr, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void CAN_TX_ISR(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(CAN_TX_Semaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void CAN_TX_Task(void *pvParameters) {
  uint8_t msgOut[8];

  while (1) {
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    CAN_TX(SEND_ID, msgOut);
  }
}

void decodeTask(void *pvParameters) {
    uint8_t localMsg[8];

    while (1) {
        xQueueReceive(msgInQ, localMsg, portMAX_DELAY);

        // 拷贝到全局 RX_Message（简单版，先不加 mutex）
        for (int i = 0; i < 8; i++) {
            RX_Message[i] = localMsg[i];
        }
        recordCanEvent(localMsg, RECORD_FROM_RX);
        handleCommandMessage(localMsg);
    }
}

void setup() {
  // put your setup code here, to run once:
  sysState.mutex = xSemaphoreCreateMutex();
  recordMutex = xSemaphoreCreateMutex();
  msgInQ = xQueueCreate(36, 8);
  msgOutQ = xQueueCreate(36, 8);
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3, 3);
  __atomic_store_n(&knob3Rotation, 7, __ATOMIC_RELAXED);   // 默认最大音量
  __atomic_store_n(&knob0Rotation, 0, __ATOMIC_RELAXED);
  __atomic_store_n(&knob1Rotation, 4, __ATOMIC_RELAXED);
  __atomic_store_n(&knob2Rotation, 0, __ATOMIC_RELAXED);
  initSineTable();

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

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  //Timer setup
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  2,			/* Task priority */
  &scanKeysHandle );	/* Pointer to store the task handle */

  TaskHandle_t displayHandle = NULL;
  xTaskCreate(
    displayUpdateTask,
    "display",
    1024,       // stack words
    NULL,
    1,         // lower priority
    &displayHandle
  );

  xTaskCreate(
    decodeTask,
    "decode",
    256,
    NULL,
    2,
    NULL
  );
  
  xTaskCreate(
    CAN_TX_Task,
    "canTX",
    256,
    NULL,
    3,     // 比 scanKeys 高一点也可以
    NULL
  );

  CAN_Init(false);
  setCANFilter(RECEIVE_ID, 0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_Start();

  __atomic_store_n(&knob3Rotation, 7, __ATOMIC_RELAXED);  // volume 默认最大（你原来）
  __atomic_store_n(&knob0Rotation, 0, __ATOMIC_RELAXED);  // wave 默认 saw
  __atomic_store_n(&knob1Rotation, 4, __ATOMIC_RELAXED);  // duty 默认中间
  __atomic_store_n(&knob2Rotation, 0, __ATOMIC_RELAXED);  // tone 默认最亮/不过滤


  vTaskStartScheduler();
}

void loop() {

}
