#include <Arduino.h>
#include <ArduinoJson.h>

namespace {

constexpr int PIN_TX2 = 17;
constexpr int PIN_RX2 = 16;
// Manchester a 300 baud => 1 bit = 3.33 ms, medio bit = 1.66 ms
// Bit 0 = LOW->HIGH (subida luz)   Bit 1 = HIGH->LOW (bajada luz)
constexpr unsigned long BIT_HALF_US = 1666;
constexpr unsigned long TX_PERIOD_MS = 7000;

// Bytes de sincronizacion
constexpr uint8_t PREAMBLE_BYTE = 0x55;   // 01010101 = onda cuadrada en Manchester
constexpr uint8_t PREAMBLE_LEN  = 4;
constexpr uint8_t SYNC_BYTE     = 0x7E;

// Timer de hardware para el periodo de transmision
hw_timer_t *txTimer = nullptr;
volatile bool txPending = false;

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

void buildMockPayload(JsonDocument &doc) {
  const float t = millis() / 1000.0f;

  const float desbalAz = 400.0f * sinf(2.0f * PI * t / 30.0f);
  const float desbalEl = 400.0f * cosf(2.0f * PI * t / 45.0f);

  const int f0 = clampAdc(2400.0f - desbalAz - desbalEl + random(-20, 21));
  const int f1 = clampAdc(2380.0f + desbalAz - desbalEl + random(-20, 21));
  const int f2 = clampAdc(2420.0f - desbalAz + desbalEl + random(-20, 21));
  const int f3 = clampAdc(2410.0f + desbalAz + desbalEl + random(-20, 21));

  const int errAz = (f1 + f3 - f0 - f2) / 2;
  const int errEl = (f2 + f3 - f0 - f1) / 2;
  const int motAz = abs(errAz) > 80 ? 1 : 0;
  const int motEl = abs(errEl) > 80 ? 1 : 0;

  const float vdc = 12.5f + 0.6f * sinf(2.0f * PI * t / 20.0f);
  const float phase = fmodf(t, 60.0f) / 60.0f;
  const float curve = sinf(PI * phase);
  const float idc = max(0.0f, 3.8f * curve + (random(-10, 11) / 100.0f));
  const float pw = vdc * idc;

  ++seqCounter;

  doc["seq"] = seqCounter;
  doc["ts"] = millis() / 1000;
  doc["F0"] = f0;
  doc["F1"] = f1;
  doc["F2"] = f2;
  doc["F3"] = f3;
  doc["Err_Az"] = errAz;
  doc["Err_El"] = errEl;
  doc["Mot_Az"] = motAz;
  doc["Mot_El"] = motEl;
  doc["Estado"] = (motAz || motEl) ? "BUSCANDO" : "TRACK";
  doc["v_dc"] = roundf(vdc * 100.0f) / 100.0f;
  doc["i_dc"] = roundf(idc * 100.0f) / 100.0f;
  doc["p_w"] = roundf(pw * 100.0f) / 100.0f;
}

// --- Manchester bit-banging --------------------------------------------
// Bit 0 = LOW->HIGH (encender laser en el centro del bit)
// Bit 1 = HIGH->LOW (apagar laser en el centro del bit)
// IEEE 802.3 convention
void IRAM_ATTR manchesterSendByte(uint8_t b) {
  for (int i = 7; i >= 0; --i) {
    const bool bit = (b >> i) & 0x01;
    if (bit) {
      digitalWrite(PIN_TX2, HIGH);
      delayMicroseconds(BIT_HALF_US);
      digitalWrite(PIN_TX2, LOW);
      delayMicroseconds(BIT_HALF_US);
    } else {
      digitalWrite(PIN_TX2, LOW);
      delayMicroseconds(BIT_HALF_US);
      digitalWrite(PIN_TX2, HIGH);
      delayMicroseconds(BIT_HALF_US);
    }
  }
}

void manchesterSendFrame(const uint8_t *data, uint16_t len) {
  // Preambulo: secuencia 0x55 = onda cuadrada perfecta para sincronizar el RX
  for (uint8_t i = 0; i < PREAMBLE_LEN; ++i) manchesterSendByte(PREAMBLE_BYTE);
  // Sync: rompe el patron y marca inicio de datos
  manchesterSendByte(SYNC_BYTE);
  // Longitud big-endian
  manchesterSendByte((len >> 8) & 0xFF);
  manchesterSendByte(len & 0xFF);
  // Payload
  for (uint16_t i = 0; i < len; ++i) manchesterSendByte(data[i]);
  // Idle final: laser apagado
  digitalWrite(PIN_TX2, LOW);
}

void sendFrame() {
  StaticJsonDocument<512> doc;
  buildMockPayload(doc);

  String payload;
  serializeJson(doc, payload);

  const uint32_t crc = crc32_calc(reinterpret_cast<const uint8_t *>(payload.c_str()), payload.length());
  String frame = payload + "|" + String(crc);    // SIN '\n', usamos LEN explicito

  manchesterSendFrame(reinterpret_cast<const uint8_t *>(frame.c_str()), frame.length());

  Serial.print("[TX] ");
  Serial.println(frame);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("[TX] ESP32 FSO TX iniciado");

  randomSeed(esp_random());

  // Bit-banging Manchester por GPIO17 (sin UART)
  pinMode(PIN_TX2, OUTPUT);
  digitalWrite(PIN_TX2, LOW);   // estado idle: laser apagado

  Serial.printf("[TX] Manchester GPIO%d  bit=%luus  half=%luus\n",
                PIN_TX2, BIT_HALF_US * 2, BIT_HALF_US);

  // Timer 0: dispara cada TX_PERIOD_MS exactos, independiente del loop()
  txTimer = timerBegin(0, 80, true);                               // 1 tick = 1 us
  timerAttachInterrupt(txTimer, &onTxTimer, true);
  timerAlarmWrite(txTimer, (uint64_t)TX_PERIOD_MS * 1000ULL, true);
  timerAlarmEnable(txTimer);
  Serial.printf("[TX] Timer hw periodo=%lu ms\n", TX_PERIOD_MS);
}

void loop() {
  if (txPending) {
    txPending = false;
    sendFrame();
  }
}
