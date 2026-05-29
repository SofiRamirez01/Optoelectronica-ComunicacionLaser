#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WebServer.h>

namespace {

constexpr int PIN_TX2 = 17;       // (sin uso, libre)
constexpr int PIN_RX_DATA = 16;   // entrada digital del receptor (LDR via NPN)
// Manchester 20 bps: T = 50ms, T/2 = 25ms
// Decodificacion por flancos: corto(<37ms)=frontera, largo(>=37ms)=centro

// Trama: [PREAMBLE 4x0x55][SYNC 0x7E][LEN 2B BE][PAYLOAD N B]
constexpr uint8_t  SYNC_BYTE     = 0x7E;
constexpr uint16_t MAX_PAYLOAD   = 600;

// GPIO34 = D34: lectura analogica del divisor LDR-10k (solo diagnostico)
// Conectar punto medio del divisor directamente a D34
constexpr int PIN_ADC_LDR = 34;

// LED RGB (catodo comun): HIGH = encendido
// Cambiar a LOW=encendido si tu modulo es de anodo comun
constexpr int PIN_LED_R = 25;
constexpr int PIN_LED_G = 26;
constexpr int PIN_LED_B = 27;
// Si no hay trama ok en 200 s (~3 frames a 20bps) se considera enlace perdido
constexpr unsigned long LINK_TIMEOUT_MS = 200000;

constexpr const char *AP_SSID = "FSO-RX";
constexpr const char *AP_PASS = "fso12345";
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_GW(192, 168, 4, 1);
const IPAddress AP_MASK(255, 255, 255, 0);

// Decoder Manchester por detección de flancos (edge-triggered, auto-sincronizante)
// T/2 = 50ms = 50000µs  T = 100ms
// Flanco corto (<75000µs): transición de frontera de bit  -> no genera bit
// Flanco largo (>=75000µs): transición del CENTRO del bit -> bit determinado por dirección
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;
constexpr unsigned long MAN_SHORT_US  = 37500;   // umbral corto vs largo (75% de T=50ms)
constexpr unsigned long MAN_TIMEOUT_US = 300000; // >300ms = resincronizar (>6 bits silencio a 20bps)

enum RxState : uint8_t {
  ST_SEARCH_PREAMBLE,
  ST_READ_LEN,
  ST_READ_PAYLOAD,
  ST_COMPLETE
};

volatile RxState rxState = ST_SEARCH_PREAMBLE;
volatile uint8_t  rxByte = 0;
volatile uint8_t  rxBitCount = 0;
volatile uint16_t rxLen = 0;
volatile uint16_t rxIdx = 0;
uint8_t  rxBuffer[MAX_PAYLOAD];
volatile bool framePending = false;
volatile uint16_t framePendingLen = 0;

volatile unsigned long manchLastEdgeUs = 0;
volatile bool manchLastWasShort = false;

WebServer server(80);

String lastPayload = "{}";
unsigned long rxOk = 0;
unsigned long lastRxOkMs = 0;   // timestamp del ultimo frame valido
unsigned long crcErr = 0;
unsigned long frameErr = 0;
unsigned long jsonErr = 0;
unsigned long seqGap = 0;
long lastSeq = -1;
int lastQuality = 0;

struct LastData {
  unsigned long seq = 0;
  unsigned long ts = 0;
  int F0 = 0, F1 = 0, F2 = 0, F3 = 0;
  int errAz = 0, errEl = 0;
  int motAz = 0, motEl = 0;
  String estado = "--";
  float vdc = 0.0f, idc = 0.0f, pw = 0.0f;
} lastData;

void setLed(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, r ? HIGH : LOW);
  digitalWrite(PIN_LED_G, g ? HIGH : LOW);
  digitalWrite(PIN_LED_B, b ? HIGH : LOW);
}

void updateLed() {
  const bool timeout = (rxOk == 0) || (millis() - lastRxOkMs > LINK_TIMEOUT_MS);
  if (timeout) {
    setLed(true, false, false);   // ROJO: sin datos
  } else if (lastQuality >= 70) {
    setLed(false, true, false);   // VERDE: enlace bueno
  } else {
    setLed(true, true, false);    // AMARILLO: enlace debil
  }
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

void updateQuality() {
  const unsigned long total = rxOk + crcErr + frameErr + jsonErr;
  if (total == 0) {
    lastQuality = 0;
    return;
  }
  lastQuality = static_cast<int>((100UL * rxOk) / total);
}

void parsePayloadFields(const String &payload) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    ++jsonErr;
    return;
  }

  const long seq = doc["seq"] | 0;
  if (lastSeq >= 0 && seq > lastSeq + 1) {
    seqGap += static_cast<unsigned long>(seq - lastSeq - 1);
  }
  lastSeq = seq;

  lastData.seq = static_cast<unsigned long>(seq);
  lastData.ts = doc["ts"] | 0;
  lastData.F0 = doc["F0"] | 0;
  lastData.F1 = doc["F1"] | 0;
  lastData.F2 = doc["F2"] | 0;
  lastData.F3 = doc["F3"] | 0;
  lastData.errAz = doc["Err_Az"] | 0;
  lastData.errEl = doc["Err_El"] | 0;
  lastData.motAz = doc["Mot_Az"] | 0;
  lastData.motEl = doc["Mot_El"] | 0;
  lastData.estado = String(static_cast<const char *>(doc["Estado"] | "--"));
  lastData.vdc = doc["v_dc"] | 0.0f;
  lastData.idc = doc["i_dc"] | 0.0f;
  lastData.pw = doc["p_w"] | 0.0f;

  ++rxOk;
  lastRxOkMs = millis();
  lastPayload = payload;
}

void handleFrame(const String &line) {
  const int sep = line.lastIndexOf('|');
  if (sep <= 0 || sep >= static_cast<int>(line.length()) - 1) {
    ++frameErr;
    return;
  }

  const String payload = line.substring(0, sep);
  const String crcStr = line.substring(sep + 1);

  char *endp = nullptr;
  const uint32_t rxCrc = strtoul(crcStr.c_str(), &endp, 10);
  if (endp == crcStr.c_str() || *endp != '\0') {
    ++frameErr;
    return;
  }

  const uint32_t calc = crc32_calc(reinterpret_cast<const uint8_t *>(payload.c_str()), payload.length());
  if (calc != rxCrc) {
    ++crcErr;
    return;
  }

  parsePayloadFields(payload);
}

void readUartFrames() {
  // Si la ISR completó una trama, procesarla
  bool ready = false;
  uint16_t len = 0;
  portENTER_CRITICAL(&rxMux);
  if (framePending) {
    framePending = false;
    ready = true;
    len = framePendingLen;
  }
  portEXIT_CRITICAL(&rxMux);

  if (!ready) return;

  String line;
  line.reserve(len);
  for (uint16_t i = 0; i < len; ++i) line += (char)rxBuffer[i];
  handleFrame(line);
}

// Recibe 1 bit decodificado: lo acumula en byte y avanza maquina de estados
inline void IRAM_ATTR rxPushBit(uint8_t bit) {
  rxByte = (rxByte << 1) | (bit & 0x01);
  ++rxBitCount;
  if (rxBitCount < 8) return;
  const uint8_t b = rxByte;
  rxBitCount = 0;
  rxByte = 0;

  switch (rxState) {
    case ST_SEARCH_PREAMBLE:
      // Esperamos el byte de sync DESPUES de cualquier secuencia de 0x55
      if (b == SYNC_BYTE) {
        rxState = ST_READ_LEN;
        rxIdx = 0;
        rxLen = 0;
      }
      // bytes 0x55 (preambulo) los ignoramos; cualquier otro byte tambien
      // (la ISR esta alineada al patron del preambulo via half-bit sampling)
      break;
    case ST_READ_LEN:
      if (rxIdx == 0) { rxLen = (uint16_t)b << 8; rxIdx = 1; }
      else {
        rxLen |= b;
        if (rxLen == 0 || rxLen > MAX_PAYLOAD) {
          rxState = ST_SEARCH_PREAMBLE;
        } else {
          rxState = ST_READ_PAYLOAD;
          rxIdx = 0;
        }
      }
      break;
    case ST_READ_PAYLOAD:
      rxBuffer[rxIdx++] = b;
      if (rxIdx >= rxLen) {
        framePendingLen = rxLen;
        framePending = true;
        rxState = ST_SEARCH_PREAMBLE;
      }
      break;
    default: break;
  }
}

// ISR: se activa en cada flanco (CHANGE) del GPIO16.
// Clasifica el flanco como CORTO (frontera de bit) o LARGO (centro = dato).
// IEEE 802.3: flanco de subida en el centro = bit 0, flanco de bajada = bit 1.
void IRAM_ATTR onEdge() {
  const unsigned long now = micros();
  const unsigned long dt  = now - manchLastEdgeUs;
  manchLastEdgeUs = now;

  // nivel actual POST-flanco: HIGH significa flanco de subida = bit 0
  const uint8_t bit = digitalRead(PIN_RX_DATA) ? 0 : 1;

  portENTER_CRITICAL_ISR(&rxMux);

  if (dt > MAN_TIMEOUT_US) {
    // Silencio prolongado: resincronizar. Este flanco es el centro del 1er bit.
    rxState = ST_SEARCH_PREAMBLE;
    rxByte = 0; rxBitCount = 0; rxIdx = 0;
    manchLastWasShort = false;
    rxPushBit(bit);
  } else if (manchLastWasShort) {
    // Anterior fue frontera -> este flanco ES el centro del bit
    rxPushBit(bit);
    manchLastWasShort = false;
  } else if (dt < MAN_SHORT_US) {
    // Flanco corto: es frontera de bit, no genera dato
    manchLastWasShort = true;
  } else {
    // Flanco largo: centro del bit sin frontera previa
    rxPushBit(bit);
    manchLastWasShort = false;
  }

  portEXIT_CRITICAL_ISR(&rxMux);
}

String stateJson() {
  StaticJsonDocument<1024> out;
  JsonObject last = out["last"].to<JsonObject>();
  last["seq"] = lastData.seq;
  last["ts"] = lastData.ts;
  last["F0"] = lastData.F0;
  last["F1"] = lastData.F1;
  last["F2"] = lastData.F2;
  last["F3"] = lastData.F3;
  last["Err_Az"] = lastData.errAz;
  last["Err_El"] = lastData.errEl;
  last["Mot_Az"] = lastData.motAz;
  last["Mot_El"] = lastData.motEl;
  last["Estado"] = lastData.estado;
  last["v_dc"] = lastData.vdc;
  last["i_dc"] = lastData.idc;
  last["p_w"] = lastData.pw;

  JsonObject metrics = out["metrics"].to<JsonObject>();
  metrics["rx_ok"] = rxOk;
  metrics["crc_err"] = crcErr;
  metrics["frame_err"] = frameErr;
  metrics["json_err"] = jsonErr;
  metrics["seq_gap"] = seqGap;
  metrics["link_quality_pct"] = lastQuality;

  String s;
  serializeJson(out, s);
  return s;
}

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="es"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>FSO RX Dashboard</title>
<style>
body{font-family:Segoe UI,Arial,sans-serif;background:#0f172a;color:#e2e8f0;margin:0;padding:16px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px}
.card{background:#111827;border:1px solid #334155;border-radius:10px;padding:12px}
.big{font-size:28px;font-weight:700}
.small{font-size:12px;color:#94a3b8}
.row{display:flex;gap:12px;flex-wrap:wrap}
.badge{padding:2px 8px;border-radius:999px;background:#1f2937}
.ok{color:#22c55e}.bad{color:#ef4444}
.bar{height:12px;background:#1f2937;border-radius:6px;overflow:hidden}
.fill{height:100%;background:linear-gradient(90deg,#ef4444,#f59e0b,#22c55e);width:0%}
</style>
</head><body>
<h2>FSO Telemetria Solar (RX)</h2>
<div class="grid">
<div class="card"><div class="small">Potencia</div><div class="big" id="p">--</div></div>
<div class="card"><div class="small">Tension</div><div class="big" id="v">--</div></div>
<div class="card"><div class="small">Corriente</div><div class="big" id="i">--</div></div>
<div class="card"><div class="small">Estado</div><div class="big" id="st">--</div></div>
<div class="card"><div class="small">Seq</div><div class="big" id="seq">--</div></div>
<div class="card"><div class="small">ErrAz / ErrEl</div><div class="big" id="err">-- / --</div></div>
</div>
<div class="card" style="margin-top:12px">
<div class="small">LDR F0 F1 F2 F3</div>
<div class="row" id="ldr">--</div>
</div>
<div class="card" style="margin-top:12px">
<div class="small">Calidad enlace</div>
<div class="bar"><div class="fill" id="qf"></div></div>
<div class="row"><span id="q">0%</span><span>OK:<b id="ok">0</b></span><span>CRC:<b id="crc">0</b></span><span>FRAME:<b id="fr">0</b></span><span>GAP:<b id="gp">0</b></span></div>
</div>
<script>
async function tick(){
  try{
    const r=await fetch('/api/state',{cache:'no-store'});
    const s=await r.json();
    const d=s.last||{},m=s.metrics||{};
    document.getElementById('p').textContent=(d.p_w??0).toFixed?d.p_w.toFixed(2)+' W':d.p_w+' W';
    document.getElementById('v').textContent=(d.v_dc??0).toFixed?d.v_dc.toFixed(2)+' V':d.v_dc+' V';
    document.getElementById('i').textContent=(d.i_dc??0).toFixed?d.i_dc.toFixed(2)+' A':d.i_dc+' A';
    document.getElementById('st').textContent=d.Estado??'--';
    document.getElementById('seq').textContent=d.seq??'--';
    document.getElementById('err').textContent=(d.Err_Az??'--')+' / '+(d.Err_El??'--');
    document.getElementById('ldr').textContent='F0:'+ (d.F0??0)+'  F1:'+ (d.F1??0)+'  F2:'+ (d.F2??0)+'  F3:'+ (d.F3??0);
    const q = Number(m.link_quality_pct??0);
    document.getElementById('q').textContent=q+'%';
    document.getElementById('qf').style.width=q+'%';
    document.getElementById('ok').textContent=m.rx_ok??0;
    document.getElementById('crc').textContent=m.crc_err??0;
    document.getElementById('fr').textContent=(m.frame_err??0)+(m.json_err??0);
    document.getElementById('gp').textContent=m.seq_gap??0;
  }catch(e){}
}
setInterval(tick,1000); tick();
</script>
</body></html>
)HTML";

void handleRoot() {
  server.send(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleApiState() {
  server.send(200, "application/json; charset=utf-8", stateJson());
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("[RX] ESP32 FSO RX iniciado");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Serial.printf("[RX] ADC LDR en GPIO%d\n", PIN_ADC_LDR);

  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  setLed(true, false, false);   // rojo hasta recibir primer dato

  pinMode(PIN_RX_DATA, INPUT);
  Serial.printf("[RX] Manchester GPIO%d edge-triggered (umbral=%lums timeout=%lums)\n",
                PIN_RX_DATA, MAN_SHORT_US/1000, MAN_TIMEOUT_US/1000);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  const bool apOk = WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[RX] AP %s ssid=%s ip=%s\n", apOk ? "OK" : "FAIL", AP_SSID,
                WiFi.softAPIP().toString().c_str());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/state", HTTP_GET, handleApiState);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();

  Serial.println("[RX] Dashboard en http://192.168.4.1/");

  // Interrupt por flanco: auto-sincronizante, sin deriva de timer
  manchLastEdgeUs = micros();
  attachInterrupt(digitalPinToInterrupt(PIN_RX_DATA), onEdge, CHANGE);
  Serial.println("[RX] Edge decoder Manchester activo (IRQ CHANGE GPIO16)");
}

void loop() {
  // La ISR del sample-timer publica framePending cuando hay trama completa.
  readUartFrames();
  updateQuality();
  updateLed();          // millis(), sin cambios
  server.handleClient();

  static unsigned long tLog = 0;
  if (millis() - tLog > 5000) {
    tLog = millis();
    Serial.printf("[RX][LINK] ok=%lu crc=%lu frame=%lu json=%lu gap=%lu q=%d%%\n",
                  rxOk, crcErr, frameErr, jsonErr, seqGap, lastQuality);
    int ldrRaw = analogRead(PIN_ADC_LDR);
    Serial.printf("[RX][LDR]  adc=%d/4095 (%d%%)\n", ldrRaw, map(ldrRaw, 0, 4095, 0, 100));
  }
}
