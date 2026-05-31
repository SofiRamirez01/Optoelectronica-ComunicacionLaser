#include <Arduino.h>

namespace {

constexpr int PIN_TX2 = 17;
// Pin donde conectaremos el pulsador (el otro extremo a GND)
constexpr int PIN_BUTTON = 4;

// Manchester a 20 bps => 1 bit = 50 ms, medio bit = 25 ms
constexpr unsigned long BIT_HALF_MS = 25;    
// El tiempo bajó a 20s gracias a la compresión binaria
constexpr unsigned long TX_PERIOD_MS = 20000;   

constexpr uint8_t PREAMBLE_BYTE = 0x55;   
constexpr uint8_t PREAMBLE_LEN  = 4;
constexpr uint8_t SYNC_BYTE     = 0x7E;

// ESTRUCTURA BINARIA (34 bytes en total)
#pragma pack(push, 1)
struct TelemetryData {
  uint32_t seq;
  uint32_t ts;
  uint16_t F0;
  uint16_t F1;
  uint16_t F2;
  uint16_t F3;
  int16_t errAz;
  int16_t errEl;
  uint8_t motAz;
  uint8_t motEl;
  float vdc;
  float idc;
  float pw;
};
#pragma pack(pop)

hw_timer_t *txTimer = nullptr;
volatile bool txPending = false;

// Variables para el Modo Apuntado y Anti-rebote
bool targetingMode = false;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
constexpr unsigned long DEBOUNCE_DELAY = 50; 

void IRAM_ATTR onTxTimer() {
  txPending = true;
}

uint32_t crc32_calc(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint32_t>(data[i]);
    for (uint8_t j = 0; j < 8; ++j) {
      const uint32_t mask = -(crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return ~crc;
}

unsigned long seqCounter = 0;

int clampAdc(float x) {
  if (x < 0) return 0;
  if (x > 4095) return 4095;
  return static_cast<int>(x);
}

void buildMockPayload(TelemetryData &data) {
  const float t = millis() / 1000.0f;

  const float desbalAz = 400.0f * sinf(2.0f * PI * t / 30.0f);
  const float desbalEl = 400.0f * cosf(2.0f * PI * t / 45.0f);

  data.F0 = clampAdc(2400.0f - desbalAz - desbalEl + random(-20, 21));
  data.F1 = clampAdc(2380.0f + desbalAz - desbalEl + random(-20, 21));
  data.F2 = clampAdc(2420.0f - desbalAz + desbalEl + random(-20, 21));
  data.F3 = clampAdc(2410.0f + desbalAz + desbalEl + random(-20, 21));

  data.errAz = (data.F1 + data.F3 - data.F0 - data.F2) / 2;
  data.errEl = (data.F2 + data.F3 - data.F0 - data.F1) / 2;
  data.motAz = abs(data.errAz) > 80 ? 1 : 0;
  data.motEl = abs(data.errEl) > 80 ? 1 : 0;

  float vdc = 12.5f + 0.6f * sinf(2.0f * PI * t / 20.0f);
  float phase = fmodf(t, 60.0f) / 60.0f;
  float curve = sinf(PI * phase);
  float idc = max(0.0f, 3.8f * curve + (random(-10, 11) / 100.0f));
  
  data.vdc = roundf(vdc * 100.0f) / 100.0f;
  data.idc = roundf(idc * 100.0f) / 100.0f;
  data.pw = roundf((vdc * idc) * 100.0f) / 100.0f;

  ++seqCounter;
  data.seq = seqCounter;
  data.ts = millis() / 1000;
}

void IRAM_ATTR manchesterSendByte(uint8_t b) {
  for (int i = 7; i >= 0; --i) {
    const bool bit = (b >> i) & 0x01;
    if (bit) {
      digitalWrite(PIN_TX2, HIGH); delay(BIT_HALF_MS);
      digitalWrite(PIN_TX2, LOW);  delay(BIT_HALF_MS);
    } else {
      digitalWrite(PIN_TX2, LOW);  delay(BIT_HALF_MS);
      digitalWrite(PIN_TX2, HIGH); delay(BIT_HALF_MS);
    }
  }
}

void manchesterSendFrame(const uint8_t *data, uint16_t len) {
  for (uint8_t i = 0; i < PREAMBLE_LEN; ++i) manchesterSendByte(PREAMBLE_BYTE);
  manchesterSendByte(SYNC_BYTE);
  manchesterSendByte((len >> 8) & 0xFF);
  manchesterSendByte(len & 0xFF);
  for (uint16_t i = 0; i < len; ++i) manchesterSendByte(data[i]);
  digitalWrite(PIN_TX2, LOW);
}

void sendFrame() {
  TelemetryData payload;
  buildMockPayload(payload);

  // Calculamos el tamaño total: 34 bytes de datos + 4 bytes de CRC
  uint16_t frameLen = sizeof(TelemetryData) + sizeof(uint32_t);
  uint8_t frameBuffer[frameLen];

  // Copiamos los datos a un arreglo de bytes
  memcpy(frameBuffer, &payload, sizeof(TelemetryData));
  
  // Calculamos el CRC solo de los datos
  uint32_t crc = crc32_calc(frameBuffer, sizeof(TelemetryData));
  
  // Pegamos el CRC al final del arreglo
  memcpy(frameBuffer + sizeof(TelemetryData), &crc, sizeof(uint32_t));

  manchesterSendFrame(frameBuffer, frameLen);

  Serial.printf("[TX] Trama binaria enviada. Seq: %lu | Tamaño: %d bytes\n", payload.seq, frameLen);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("[TX] ESP32 FSO TX iniciado (Modo Binario)");

  randomSeed(esp_random());

  pinMode(PIN_TX2, OUTPUT);
  digitalWrite(PIN_TX2, LOW);   

  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Serial.printf("[TX] Manchester GPIO%d  bit=%lums  half=%lums\n",
                PIN_TX2, BIT_HALF_MS * 2, BIT_HALF_MS);

  txTimer = timerBegin(0, 80, true);                               
  timerAttachInterrupt(txTimer, &onTxTimer, true);
  timerAlarmWrite(txTimer, (uint64_t)TX_PERIOD_MS * 1000ULL, true);
  timerAlarmEnable(txTimer);
  Serial.printf("[TX] Timer hw periodo=%lu ms\n", TX_PERIOD_MS);
}

void loop() {
  // 1. LECTURA DEL BOTÓN CON ANTI-REBOTE
  int reading = digitalRead(PIN_BUTTON);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    static int buttonState = HIGH;
    if (reading != buttonState) {
      buttonState = reading;
      
      if (buttonState == LOW) {
        targetingMode = !targetingMode;
        
        if (targetingMode) {
          Serial.println("\n[TX] >>> MODO APUNTADO ACTIVADO <<< (Laser FIJO)");
          timerAlarmDisable(txTimer); 
          digitalWrite(PIN_TX2, HIGH); 
        } else {
          Serial.println("\n[TX] <<< MODO NORMAL ACTIVADO >>> (Transmision habilitada)");
          digitalWrite(PIN_TX2, LOW); 
          txPending = false;          
          timerWrite(txTimer, 0);     
          timerAlarmEnable(txTimer);  
        }
      }
    }
  }
  lastButtonState = reading;

  // 2. LÓGICA DE TRANSMISIÓN (Solo si NO estamos en modo apuntado)
  if (!targetingMode) {
    if (txPending) {
      txPending = false;
      sendFrame();
    }
    if (Serial.available() > 0) {
      Serial.readString(); 
      Serial.println("[TX] *** Disparo manual forzado por Serie ***");
      sendFrame();
    }
  }
}