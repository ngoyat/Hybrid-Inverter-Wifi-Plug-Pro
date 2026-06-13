/*
 * DESS Logger Emulator with MQTT Live Data
 * =================================================
 * Board  : ESP32 (ESP32-CAM or any ESP32 variant)
 * IDE    : Arduino / PlatformIO
 * Library: PubSubClient by Nick O'Leary (install via Library Manager)
 *
 * MQTT topics — publish plain float string, retain=true recommended
 * ──────────────────────────────────────────────────────────────────
 *  desstransmitter/data/mains/voltage           V     (< 100 = mains absent)
 *  desstransmitter/data/mains/frequency         Hz
 *  desstransmitter/data/battery/voltage         V
 *  desstransmitter/data/battery/charging_amp    A     (> 0 = charging)
 *  desstransmitter/data/battery/discharging_amp A     (> 0 = discharging)
 *  desstransmitter/data/battery/soc             %
 *  desstransmitter/data/pv/voltage              V
 *  desstransmitter/data/pv/current              A
 *  desstransmitter/data/pv/power_w              W     (> 10 = PV active)
 *  desstransmitter/data/load/ac_w               W
 *
 * Derived logic (applied fresh on every DESS poll):
 *  pv.power_w  > 10   → pvStatus    = PV_DISCHARGE (1)   else UNDERVOLTAGE (0)
 *  mains.v     >= 100 → mainsStatus = MAINS_DISCHARGE (2), workingMode = Line Mode (4)
 *  mains.v     < 100  → mainsStatus = MAINS_ABNORMALITY (0), workingMode = Invert (3)
 *                        outV forced to 2300 (230.0V), outHz forced to 500 (50.0Hz)
 *  batt.chgA   > 0    → battStatus  = BATT_CHARGING (2)
 *  batt.disA   > 0    → battStatus  = BATT_DISCHARGING (1)
 *  both == 0          → battStatus  = BATT_FLOAT (3)
 *  load always        → loadStatus  = LOAD_ON (1)
 *
 * Load calculations (W-based, PF 0.95, 6200W capacity):
 *  outVA   = load_ac_w / 0.95
 *  loadPct = (load_ac_w / 6200.0) * 100   clamped 0–100
 *
 * appShows = (loadStatus<<6)|(mainsStatus<<4)|(pvStatus<<2)|battStatus
 *
 * All nameplate/settings registers hardcoded from dessnormal.xlsx 2026-03-01.
 * Modbus RTU: Slave ID 5, 2400 baud 8N1, little-endian register payload.
 */

// ============================================================
//  WiFi credentials  — EDIT THESE
// ============================================================
#define WIFI_SSID       "YOUR_SSID"
#define WIFI_PASS       "YOUR_PASSWORD"

// ============================================================
//  MQTT broker  — EDIT THESE
// ============================================================
#define MQTT_BROKER     "192.168.1.100"   // broker IP or hostname
#define MQTT_PORT       1883
#define MQTT_CLIENT_ID  "dess_emulator"
#define MQTT_USER       ""                // leave blank if no auth
#define MQTT_PASS_STR   ""

// ============================================================
//  Modbus hardware
// ============================================================
#define SLAVE_ID        5
#define BAUD_RATE       2400
#define LOGGER_RX_PIN   15
#define LOGGER_TX_PIN   14

// ============================================================
//  MQTT base topic
// ============================================================
#define BASE_TOPIC      "desstransmitter/data/"

#include <WiFi.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <string.h>
#include <stdlib.h>

HardwareSerial loggerSerial(2);
WiFiClient     wifiClient;
PubSubClient   mqttClient(wifiClient);

// ============================================================
//  Live values — updated by MQTT, read on every DESS poll
// ============================================================
volatile float lv_mainsV    = 245.7f;   // V
volatile float lv_mainsHz   = 50.0f;    // Hz
volatile float lv_battV     = 54.0f;    // V
volatile float lv_battChgA  = 0.0f;     // A
volatile float lv_battDisA  = 0.0f;     // A
volatile float lv_battSOC   = 100.0f;   // %
volatile float lv_pvV       = 280.7f;   // V
volatile float lv_pvA       = 0.0f;     // A  (stored, not used in registers)
volatile float lv_pvW       = 2309.0f;  // W
volatile float lv_loadW     = 911.0f;   // W

// ============================================================
//  Nameplate / fixed constants  (dessnormal.xlsx 2026-03-01)
// ============================================================
// --- inverter capacity ---
#define NOM_W           6200            // 6.2 kW active — load% base

static const uint16_t MACHINE_TYPE     = 2;     // DESS shows "--"
static const uint16_t CPU_VER          = 7411;
static const uint16_t SEC_CPU_VER      = 1;
static const uint16_t BATT_PIECE       = 2;     // 48V(5KW)
static const uint16_t NOM_VA           = 6200;
static const uint16_t NOM_AC_V         = 230;
static const uint16_t NOM_AC_A         = 26;
static const uint16_t RATED_BATT_V     = 480;   // 48.0V raw
static const uint16_t NOM_OUT_V        = 230;
static const uint16_t NOM_OUT_HZ       = 500;   // 50.0Hz raw
static const uint16_t NOM_OUT_A        = 26;

// --- settings flags ---
// 0x011D = BattEq(b0)+Buzzer(b2)+PowerSave(b3)+LCD(b4)+RecordFault(b8)
static const uint16_t SETTING_STATE    = 0x011D;
static const uint16_t CHARGER_PRI      = 2;    // Solar and mains
static const uint16_t OUT_PRI          = 1;    // Solar
static const uint16_t AC_RANGE         = 1;    // UPS
static const uint16_t BATT_TYPE        = 2;    // User
static const uint16_t OUT_FREQ_SET     = 0;    // 50Hz
static const uint16_t MAX_CHG_A        = 30;
static const uint16_t OUT_V_SET        = 230;
static const uint16_t MAX_UTIL_A       = 30;
static const uint16_t CB_UTIL_V        = 460;  // 46.0V
static const uint16_t CB_BATT_V        = 540;  // 54.0V

// --- charge voltage settings ---
static const uint16_t BULK_CHG_V       = 574;  // 57.4V
static const uint16_t FLOAT_CHG_V      = 540;  // 54.0V
static const uint16_t LOW_BATT_V       = 420;  // 42.0V
static const uint16_t BATT_EQ_V        = 600;  // 60.0V
static const uint16_t BATT_EQ_TIME     = 60;   // min
static const uint16_t BATT_EQ_TIMEOUT  = 120;  // min
static const uint16_t BATT_EQ_INTERVAL = 30;   // days

// --- single-register mirrors ---
static const uint16_t REG_5001         = 0;
static const uint16_t REG_5020         = 2;    // User battery

// ============================================================
//  appShows enum constants
// ============================================================
#define BATT_NOT_CONNECTED  0
#define BATT_DISCHARGING    1
#define BATT_CHARGING       2
#define BATT_FLOAT          3

#define PV_UNDERVOLTAGE     0
#define PV_DISCHARGE        1

#define MAINS_ABNORMALITY   0
#define MAINS_DISCHARGE     2

#define LOAD_ON             1

// ============================================================
//  Register arrays (built fresh on every DESS poll)
// ============================================================
uint16_t reg4501[45];
uint16_t reg4546_block[16];

// ============================================================
//  Derive functions
// ============================================================
uint8_t deriveBattStatus() {
  // discharging_amp == 0  → Charging (covers float/idle too)
  // discharging_amp  > 0  → Discharging
  if (lv_battDisA > 0.0f) return BATT_DISCHARGING;
  return BATT_CHARGING;
}

uint8_t derivePvStatus() {
  return (lv_pvW > 10.0f) ? PV_DISCHARGE : PV_UNDERVOLTAGE;
}

uint8_t deriveMainsStatus() {
  return (lv_mainsV > 140.0f) ? MAINS_DISCHARGE : MAINS_ABNORMALITY;
}

uint16_t deriveAppShows() {
  return (uint16_t)(
    (LOAD_ON              << 6) |
    (deriveMainsStatus()  << 4) |
    (derivePvStatus()     << 2) |
    (deriveBattStatus()        )
  );
}

uint16_t deriveWorkingMode() {
  return (lv_mainsV > 140.0f) ? 4 : 3;  // 4=Line Mode  3=Invert
}

// Output voltage/freq: track mains when present, else nominal
uint16_t deriveOutV() {
  // mains > 140V  → output tracks mains
  // mains <= 140V → inverter running, output = nominal 230V
  return (lv_mainsV > 140.0f) ? (uint16_t)(lv_mainsV * 10.0f) : 2300;
}
uint16_t deriveOutHz() {
  return (lv_mainsV > 140.0f) ? (uint16_t)(lv_mainsHz * 10.0f) : 500;
}

// VA from W using PF 0.95
uint16_t deriveOutVA() {
  return (uint16_t)(lv_loadW / 0.95f);
}

// Load % based on active W vs 6200W capacity
uint16_t deriveLoadPct() {
  uint16_t pct = (uint16_t)((lv_loadW / (float)NOM_W) * 100.0f);
  if (pct > 100) pct = 100;
  return pct;
}

// ============================================================
//  Build reg4501[45] — called on every incoming DESS poll
// ============================================================
void buildReg4501() {
  memset(reg4501, 0, sizeof(reg4501));

  // --- live data ---
  reg4501[0]  = deriveWorkingMode();
  reg4501[1]  = (lv_mainsV > 140.0f) ? (uint16_t)(lv_mainsV  * 10.0f) : 0;
  reg4501[2]  = (lv_mainsV > 140.0f) ? (uint16_t)(lv_mainsHz * 10.0f) : 0;
  reg4501[3]  = (uint16_t)(lv_pvV  * 10.0f);
  reg4501[4]  = (uint16_t)lv_pvW;
  reg4501[5]  = (uint16_t)(lv_battV * 10.0f);
  reg4501[6]  = (uint16_t)lv_battSOC;
  reg4501[7]  = (uint16_t)lv_battChgA;
  reg4501[8]  = (uint16_t)lv_battDisA;
  reg4501[9]  = deriveOutV();
  reg4501[10] = deriveOutHz();
  reg4501[11] = deriveOutVA();          // VA = W / 0.95
  reg4501[12] = (uint16_t)lv_loadW;    // active W direct from MQTT
  reg4501[13] = deriveLoadPct();        // W / 6200 * 100
  // [14][15] reserved — zero (memset covers these)

  // --- nameplate ---
  reg4501[16] = MACHINE_TYPE;
  reg4501[17] = CPU_VER;
  reg4501[18] = SEC_CPU_VER;
  reg4501[19] = BATT_PIECE;
  reg4501[20] = NOM_VA;
  reg4501[21] = NOM_W;
  reg4501[22] = NOM_AC_V;
  reg4501[23] = NOM_AC_A;
  reg4501[24] = RATED_BATT_V;
  reg4501[25] = NOM_OUT_V;
  reg4501[26] = NOM_OUT_HZ;
  reg4501[27] = NOM_OUT_A;

  // --- fault / alarm ---
  reg4501[28] = 0;   // errorCode1 — no fault
  reg4501[29] = 0;   // alarmCode1 — no alarm
  reg4501[30] = 0;   // devID1
  reg4501[31] = 0;   // devID2
  reg4501[32] = 0;   // devID3
  reg4501[33] = 0;   // devID4

  // --- settings ---
  reg4501[34] = SETTING_STATE;
  reg4501[35] = CHARGER_PRI;
  reg4501[36] = OUT_PRI;
  reg4501[37] = AC_RANGE;
  reg4501[38] = BATT_TYPE;
  reg4501[39] = OUT_FREQ_SET;
  reg4501[40] = MAX_CHG_A;
  reg4501[41] = OUT_V_SET;
  reg4501[42] = MAX_UTIL_A;
  reg4501[43] = CB_UTIL_V;
  reg4501[44] = CB_BATT_V;
}

// ============================================================
//  Build reg4546_block[16] — called on every incoming poll
// ============================================================
void buildReg4546() {
  memset(reg4546_block, 0, sizeof(reg4546_block));
  reg4546_block[0] = BULK_CHG_V;
  reg4546_block[1] = FLOAT_CHG_V;
  reg4546_block[2] = LOW_BATT_V;
  reg4546_block[3] = BATT_EQ_V;
  reg4546_block[4] = BATT_EQ_TIME;
  reg4546_block[5] = BATT_EQ_TIMEOUT;
  reg4546_block[6] = BATT_EQ_INTERVAL;
  reg4546_block[7] = deriveAppShows();  // energy-flow animation
  // [8]..[15] energy counters — keep zero
}

// ============================================================
//  CRC16 Modbus
// ============================================================
uint16_t crc16(const uint8_t *buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

// ============================================================
//  Send Modbus response frame (little-endian CRC)
// ============================================================
void sendFrame(uint8_t *data, uint16_t len) {
  uint16_t crc = crc16(data, len);
  loggerSerial.write(data, len);
  loggerSerial.write((uint8_t)(crc & 0xFF));
  loggerSerial.write((uint8_t)((crc >> 8) & 0xFF));
  loggerSerial.flush();
  Serial.print("TX: ");
  for (uint16_t i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
  Serial.printf("%02X %02X\n", crc & 0xFF, (crc >> 8) & 0xFF);
}

// ============================================================
//  FC03 handler
// ============================================================
void handleFC03(uint16_t startReg, uint16_t qty) {
  Serial.printf("FC03 reg=%u qty=%u  [mainsV=%.1f pvW=%.1f battChg=%.1f battDis=%.1f loadW=%.1f]\n",
    startReg, qty, (float)lv_mainsV, (float)lv_pvW,
    (float)lv_battChgA, (float)lv_battDisA, (float)lv_loadW);

  // --- reg4501 qty=45 (live data + nameplate + settings) ---
  if (startReg == 4501 && qty == 45) {
    buildReg4501();
    uint8_t resp[3 + 90];
    resp[0] = SLAVE_ID; resp[1] = 0x03; resp[2] = 90;
    for (int i = 0; i < 45; i++) {
      resp[3 + i*2]     =  reg4501[i] & 0xFF;         // Lo byte first
      resp[3 + i*2 + 1] = (reg4501[i] >> 8) & 0xFF;   // Hi byte second
    }
    sendFrame(resp, sizeof(resp));
    return;
  }

  // --- reg4546_block qty=16 (charge settings + appShows) ---
  if (startReg == 4546 && qty == 16) {
    buildReg4546();
    uint8_t resp[3 + 32];
    resp[0] = SLAVE_ID; resp[1] = 0x03; resp[2] = 32;
    for (int i = 0; i < 16; i++) {
      resp[3 + i*2]     =  reg4546_block[i] & 0xFF;
      resp[3 + i*2 + 1] = (reg4546_block[i] >> 8) & 0xFF;
    }
    sendFrame(resp, sizeof(resp));
    return;
  }

  // --- single-register polls ---
  if (qty == 1) {
    uint16_t val = 0;
    if      (startReg == 5001) val = REG_5001;
    else if (startReg == 5020) val = REG_5020;
    else if (startReg == 4553) { buildReg4546(); val = reg4546_block[7]; }
    uint8_t resp[] = {
      SLAVE_ID, 0x03, 0x02,
      (uint8_t)(val & 0xFF),
      (uint8_t)(val >> 8)
    };
    sendFrame(resp, sizeof(resp));
    return;
  }

  // --- fallback: return zeros for any unrecognised request ---
  uint16_t byteCount = qty * 2;
  if (byteCount > 200) byteCount = 200;
  uint8_t resp[3 + 200];
  resp[0] = SLAVE_ID; resp[1] = 0x03; resp[2] = (uint8_t)byteCount;
  memset(&resp[3], 0, byteCount);
  sendFrame(resp, 3 + byteCount);
}

// ============================================================
//  Modbus RX buffer
// ============================================================
uint8_t       rxBuf[32];
uint8_t       rxLen    = 0;
unsigned long lastByte = 0;

void processFrame() {
  if (rxLen < 8) { rxLen = 0; return; }
  if (rxBuf[0] != SLAVE_ID) { rxLen = 0; return; }
  uint16_t rxCRC   = rxBuf[rxLen-2] | ((uint16_t)rxBuf[rxLen-1] << 8);
  uint16_t calcCRC = crc16(rxBuf, rxLen - 2);
  if (rxCRC != calcCRC) {
    Serial.printf("CRC FAIL  got=%04X  calc=%04X\n", rxCRC, calcCRC);
    rxLen = 0; return;
  }
  Serial.print("RX: ");
  for (int i = 0; i < rxLen; i++) Serial.printf("%02X ", rxBuf[i]);
  Serial.println();
  uint8_t  fc  = rxBuf[1];
  uint16_t reg = ((uint16_t)rxBuf[2] << 8) | rxBuf[3];
  uint16_t qty = ((uint16_t)rxBuf[4] << 8) | rxBuf[5];
  if (fc == 0x03) handleFC03(reg, qty);
  else Serial.printf("Unsupported FC 0x%02X\n", fc);
  rxLen = 0;
}

// ============================================================
//  MQTT callback — one topic per value, plain float string
// ============================================================
void mqttCallback(char *topic, byte *payload, unsigned int length) {
  char buf[32] = {0};
  if (length > 31) length = 31;
  memcpy(buf, payload, length);
  float val = atof(buf);
  String t(topic);
  Serial.printf("[MQTT] %s = %s\n", topic, buf);

  if      (t.endsWith("mains/voltage"))            lv_mainsV   = val;
  else if (t.endsWith("mains/frequency"))           lv_mainsHz  = val;
  else if (t.endsWith("battery/voltage"))           lv_battV    = val;
  else if (t.endsWith("battery/charging_amp"))      lv_battChgA = val;
  else if (t.endsWith("battery/discharging_amp"))   lv_battDisA = val;
  else if (t.endsWith("battery/soc"))               lv_battSOC  = val;
  else if (t.endsWith("pv/voltage"))                lv_pvV      = val;
  else if (t.endsWith("pv/current"))                lv_pvA      = val;
  else if (t.endsWith("pv/power_w"))                lv_pvW      = val;
  else if (t.endsWith("load/ac_w"))                 lv_loadW    = val;
}

// ============================================================
//  WiFi connect
// ============================================================
void connectWiFi() {
  Serial.printf("Connecting to WiFi: %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
    if (++tries > 40) { Serial.println("\nWiFi timeout — reboot"); ESP.restart(); }
  }
  Serial.printf("\nWiFi OK  IP=%s\n", WiFi.localIP().toString().c_str());
}

// ============================================================
//  MQTT connect + subscribe
// ============================================================
void connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.printf("Connecting MQTT %s:%d ...\n", MQTT_BROKER, MQTT_PORT);
    bool ok = (strlen(MQTT_USER) > 0)
      ? mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS_STR)
      : mqttClient.connect(MQTT_CLIENT_ID);
    if (ok) {
      Serial.println("MQTT connected");
      mqttClient.subscribe(BASE_TOPIC "#");
      Serial.println("Subscribed: " BASE_TOPIC "#");
    } else {
      Serial.printf("MQTT failed rc=%d  retry in 5s\n", mqttClient.state());
      delay(5000);
    }
  }
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== SRNE DESS Emulator + MQTT  v3.0 ===");
  Serial.printf("Slave ID: %d  |  Baud: %d  |  RX=GPIO%d  TX=GPIO%d\n",
                SLAVE_ID, BAUD_RATE, LOGGER_RX_PIN, LOGGER_TX_PIN);
  Serial.println("Load%%: W/6200*100  |  VA: W/0.95");

  connectWiFi();
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  connectMQTT();

  loggerSerial.begin(BAUD_RATE, SERIAL_8N1, LOGGER_RX_PIN, LOGGER_TX_PIN);
  Serial.println("Ready — waiting for DESS logger polls...");
}

// ============================================================
//  Loop
// ============================================================
void loop() {
  // Keep MQTT alive and process incoming messages
  if (!mqttClient.connected()) connectMQTT();
  mqttClient.loop();

  // Buffer incoming Modbus bytes
  while (loggerSerial.available()) {
    uint8_t b = loggerSerial.read();
    if (rxLen < sizeof(rxBuf)) rxBuf[rxLen++] = b;
    lastByte = millis();
  }

  // Process complete frame after 20 ms inter-frame gap
  if (rxLen > 0 && (millis() - lastByte) > 20) {
    processFrame();
    memset(rxBuf, 0, sizeof(rxBuf));
    rxLen = 0;
  }
}
