/*
 * DESS Logger Emulator with MQTT Live Data
 * =================================================
 * Board  : ESP8266 ESP-12F
 * IDE    : Arduino (install ESP8266 core by ESP8266 Community)
 * Library: PubSubClient by Nick O'Leary (Library Manager)
 *
 * Pin wiring (ESP-12F):
 *   GPIO1  = UART0 TX  → USB/debug Serial (do NOT use for logger)
 *   GPIO3  = UART0 RX  → USB/debug Serial
 *   GPIO14 = UART0 TX  swapped to SoftwareSerial or use UART0 swap
 *
 *   DESS logger connects to:
 *   GPIO13 (RX of ESP) ← TX of logger
 *   GPIO15 (TX of ESP) → RX of logger   [pull-down 10k to GND already needed for boot]
 *
 *   Serial1 on ESP8266 is TX-only (GPIO2).
 *   We use SoftwareSerial on GPIO13/GPIO15 for the logger bus.
 *   Debug prints go to Serial (UART0, GPIO1/GPIO3) at 115200.
 *
 * MQTT topics — publish plain float string, retain=true recommended
 * ─────────────────────────────────────────────────────────────────
 *  desstransmitter/data/mains/voltage           V     (<=140 = mains absent)
 *  desstransmitter/data/mains/frequency         Hz
 *  desstransmitter/data/battery/voltage         V
 *  desstransmitter/data/battery/charging_amp    A     (unused in logic)
 *  desstransmitter/data/battery/discharging_amp A     (>0 = discharging, ==0 = charging)
 *  desstransmitter/data/battery/soc             %
 *  desstransmitter/data/pv/voltage              V
 *  desstransmitter/data/pv/current              A
 *  desstransmitter/data/pv/power_w              W     (>10 = PV active)
 *  desstransmitter/data/load/ac_w               W
 *
 * Derived logic (applied fresh on every DESS poll):
 *  batt.disA   == 0   → battStatus = BATT_CHARGING (2)
 *  batt.disA   >  0   → battStatus = BATT_DISCHARGING (1)
 *  pv.power_w  >  10  → pvStatus   = PV_DISCHARGE (1)  else UNDERVOLTAGE (0)
 *  mains.v     > 140  → mainsStatus= MAINS_DISCHARGE (2), workingMode=Line Mode (4)
 *                        outV=mains V, outHz=mains Hz
 *  mains.v     <= 140 → mainsStatus= MAINS_ABNORMALITY (0), workingMode=Invert (3)
 *                        outV=230V, outHz=50Hz
 *  load always        → loadStatus = LOAD_ON (1)
 *
 * Load: outVA = W/0.95  |  loadPct = W/6200*100  (clamped 0-100)
 * appShows = (loadStatus<<6)|(mainsStatus<<4)|(pvStatus<<2)|battStatus
 *
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
#define MQTT_BROKER     "192.168.1.100"
#define MQTT_PORT       1883
#define MQTT_CLIENT_ID  "dess_emulator"
#define MQTT_USER       ""
#define MQTT_PASS_STR   ""

// ============================================================
//  Modbus / logger serial pins (SoftwareSerial)
//  GPIO13 = RX (D7 on NodeMCU), GPIO15 = TX (D8 on NodeMCU)
//  GPIO15 must have a 10k pull-down to GND for stable boot.
// ============================================================
#define LOGGER_RX_PIN   13
#define LOGGER_TX_PIN   15
#define SLAVE_ID        5
#define BAUD_RATE       2400

#define BASE_TOPIC      "desstransmitter/data/"

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

// SoftwareSerial for DESS logger (full-duplex at 2400 baud is fine)
SoftwareSerial loggerSerial(LOGGER_RX_PIN, LOGGER_TX_PIN);

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

// ============================================================
//  Live values — updated by MQTT callback
//  ESP8266 is single-core; no true race condition, but we
//  snapshot all values at the start of each FC03 response
//  to keep one poll self-consistent.
// ============================================================
float lv_mainsV    = 245.7f;
float lv_mainsHz   = 50.0f;
float lv_battV     = 54.0f;
float lv_battChgA  = 0.0f;   // received but not used in logic
float lv_battDisA  = 0.0f;
float lv_battSOC   = 100.0f;
float lv_pvV       = 280.7f;
float lv_pvA       = 0.0f;
float lv_pvW       = 2309.0f;
float lv_loadW     = 911.0f;

// Snapshot taken at poll time — all register builds use these
float sv_mainsV, sv_mainsHz, sv_battV, sv_battDisA;
float sv_battSOC, sv_pvV, sv_pvW, sv_loadW;

void takeSnapshot() {
  sv_mainsV   = lv_mainsV;
  sv_mainsHz  = lv_mainsHz;
  sv_battV    = lv_battV;
  sv_battDisA = lv_battDisA;
  sv_battSOC  = lv_battSOC;
  sv_pvV      = lv_pvV;
  sv_pvW      = lv_pvW;
  sv_loadW    = lv_loadW;
}

// ============================================================
//  Nameplate / fixed constants  (dessnormal.xlsx 2026-03-01)
// ============================================================
#define NOM_W           6200

static const uint16_t MACHINE_TYPE     = 2;
static const uint16_t CPU_VER          = 7411;
static const uint16_t SEC_CPU_VER      = 1;
static const uint16_t BATT_PIECE       = 2;
static const uint16_t NOM_VA_REG       = 6200;
static const uint16_t NOM_AC_V         = 230;
static const uint16_t NOM_AC_A         = 26;
static const uint16_t RATED_BATT_V     = 480;
static const uint16_t NOM_OUT_V        = 230;
static const uint16_t NOM_OUT_HZ       = 500;
static const uint16_t NOM_OUT_A        = 26;
static const uint16_t SETTING_STATE    = 0x011D;
static const uint16_t CHARGER_PRI      = 2;
static const uint16_t OUT_PRI          = 1;
static const uint16_t AC_RANGE         = 1;
static const uint16_t BATT_TYPE        = 2;
static const uint16_t OUT_FREQ_SET     = 0;
static const uint16_t MAX_CHG_A        = 30;
static const uint16_t OUT_V_SET        = 230;
static const uint16_t MAX_UTIL_A       = 30;
static const uint16_t CB_UTIL_V        = 460;
static const uint16_t CB_BATT_V        = 540;
static const uint16_t BULK_CHG_V       = 574;
static const uint16_t FLOAT_CHG_V      = 540;
static const uint16_t LOW_BATT_V       = 420;
static const uint16_t BATT_EQ_V        = 600;
static const uint16_t BATT_EQ_TIME     = 60;
static const uint16_t BATT_EQ_TIMEOUT  = 120;
static const uint16_t BATT_EQ_INTERVAL = 30;
static const uint16_t REG_5001         = 0;
static const uint16_t REG_5020         = 2;

// ============================================================
//  Enum constants
// ============================================================
#define BATT_DISCHARGING    1
#define BATT_CHARGING       2
#define PV_UNDERVOLTAGE     0
#define PV_DISCHARGE        1
#define MAINS_ABNORMALITY   0
#define MAINS_DISCHARGE     2
#define LOAD_ON             1

// ============================================================
//  Derive functions — all use snapshot values (sv_*)
// ============================================================
uint8_t deriveBattStatus()  { return (sv_battDisA > 0.0f) ? BATT_DISCHARGING : BATT_CHARGING; }
uint8_t derivePvStatus()    { return (sv_pvW      > 10.0f)? PV_DISCHARGE     : PV_UNDERVOLTAGE; }
uint8_t deriveMainsStatus() { return (sv_mainsV   > 140.0f)? MAINS_DISCHARGE : MAINS_ABNORMALITY; }

uint16_t deriveAppShows() {
  return (uint16_t)(
    (LOAD_ON              << 6) |
    (deriveMainsStatus()  << 4) |
    (derivePvStatus()     << 2) |
    (deriveBattStatus()        )
  );
}

uint16_t deriveWorkingMode() { return (sv_mainsV > 140.0f) ? 4 : 3; }
uint16_t deriveOutV()   { return (sv_mainsV > 140.0f) ? (uint16_t)(sv_mainsV  * 10.0f) : 2300; }
uint16_t deriveOutHz()  { return (sv_mainsV > 140.0f) ? (uint16_t)(sv_mainsHz * 10.0f) : 500;  }
uint16_t deriveOutVA()  { return (uint16_t)(sv_loadW / 0.95f); }
uint16_t deriveLoadPct(){
  uint16_t p = (uint16_t)((sv_loadW / (float)NOM_W) * 100.0f);
  return (p > 100) ? 100 : p;
}

// ============================================================
//  Register arrays — declared globally to avoid stack pressure
// ============================================================
uint16_t reg4501[45];
uint16_t reg4546_block[16];

void buildReg4501() {
  memset(reg4501, 0, sizeof(reg4501));
  reg4501[0]  = deriveWorkingMode();
  reg4501[1]  = (sv_mainsV > 140.0f) ? (uint16_t)(sv_mainsV  * 10.0f) : 0;
  reg4501[2]  = (sv_mainsV > 140.0f) ? (uint16_t)(sv_mainsHz * 10.0f) : 0;
  reg4501[3]  = (uint16_t)(sv_pvV  * 10.0f);
  reg4501[4]  = (uint16_t)sv_pvW;
  reg4501[5]  = (uint16_t)(sv_battV * 10.0f);
  reg4501[6]  = (uint16_t)sv_battSOC;
  reg4501[7]  = 0;                          // charging_amp not tracked in logic
  reg4501[8]  = (uint16_t)sv_battDisA;
  reg4501[9]  = deriveOutV();
  reg4501[10] = deriveOutHz();
  reg4501[11] = deriveOutVA();
  reg4501[12] = (uint16_t)sv_loadW;
  reg4501[13] = deriveLoadPct();
  // [14][15] reserved — zero
  reg4501[16] = MACHINE_TYPE;
  reg4501[17] = CPU_VER;
  reg4501[18] = SEC_CPU_VER;
  reg4501[19] = BATT_PIECE;
  reg4501[20] = NOM_VA_REG;
  reg4501[21] = NOM_W;
  reg4501[22] = NOM_AC_V;
  reg4501[23] = NOM_AC_A;
  reg4501[24] = RATED_BATT_V;
  reg4501[25] = NOM_OUT_V;
  reg4501[26] = NOM_OUT_HZ;
  reg4501[27] = NOM_OUT_A;
  reg4501[28] = 0;   // errorCode1
  reg4501[29] = 0;   // alarmCode1
  reg4501[30] = 0;   reg4501[31] = 0;
  reg4501[32] = 0;   reg4501[33] = 0;
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

void buildReg4546() {
  memset(reg4546_block, 0, sizeof(reg4546_block));
  reg4546_block[0] = BULK_CHG_V;
  reg4546_block[1] = FLOAT_CHG_V;
  reg4546_block[2] = LOW_BATT_V;
  reg4546_block[3] = BATT_EQ_V;
  reg4546_block[4] = BATT_EQ_TIME;
  reg4546_block[5] = BATT_EQ_TIMEOUT;
  reg4546_block[6] = BATT_EQ_INTERVAL;
  reg4546_block[7] = deriveAppShows();
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
//  Send frame — ESP8266 SoftwareSerial flush is blocking, fine
// ============================================================
// Response buffer declared globally to avoid stack pressure on ESP8266
uint8_t txBuf[3 + 90 + 2];   // largest needed: 3 hdr + 90 data + 2 CRC

void sendFrame(uint8_t *data, uint16_t len) {
  uint16_t crc = crc16(data, len);
  loggerSerial.write(data, len);
  loggerSerial.write((uint8_t)(crc & 0xFF));
  loggerSerial.write((uint8_t)((crc >> 8) & 0xFF));
  // No flush() on SoftwareSerial — it blocks; TX completes automatically
  Serial.print("TX: ");
  for (uint16_t i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
  Serial.printf("%02X %02X\n", crc & 0xFF, (crc >> 8) & 0xFF);
}

// ============================================================
//  FC03 handler
// ============================================================
void handleFC03(uint16_t startReg, uint16_t qty) {
  Serial.printf("FC03 reg=%u qty=%u [mV=%.1f pvW=%.1f disA=%.1f lW=%.1f]\n",
    startReg, qty, sv_mainsV, sv_pvW, sv_battDisA, sv_loadW);

  if (startReg == 4501 && qty == 45) {
    buildReg4501();
    txBuf[0] = SLAVE_ID; txBuf[1] = 0x03; txBuf[2] = 90;
    for (int i = 0; i < 45; i++) {
      txBuf[3 + i*2]     =  reg4501[i] & 0xFF;
      txBuf[3 + i*2 + 1] = (reg4501[i] >> 8) & 0xFF;
    }
    sendFrame(txBuf, 3 + 90);
    return;
  }

  if (startReg == 4546 && qty == 16) {
    buildReg4546();
    txBuf[0] = SLAVE_ID; txBuf[1] = 0x03; txBuf[2] = 32;
    for (int i = 0; i < 16; i++) {
      txBuf[3 + i*2]     =  reg4546_block[i] & 0xFF;
      txBuf[3 + i*2 + 1] = (reg4546_block[i] >> 8) & 0xFF;
    }
    sendFrame(txBuf, 3 + 32);
    return;
  }

  if (qty == 1) {
    uint16_t val = 0;
    if      (startReg == 5001) val = REG_5001;
    else if (startReg == 5020) val = REG_5020;
    else if (startReg == 4553) { buildReg4546(); val = reg4546_block[7]; }
    uint8_t r[] = { SLAVE_ID, 0x03, 0x02,
                    (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
    sendFrame(r, sizeof(r));
    return;
  }

  // fallback: zero-fill
  uint16_t bc = qty * 2;
  if (bc > 200) bc = 200;
  memset(txBuf, 0, 3 + bc);
  txBuf[0] = SLAVE_ID; txBuf[1] = 0x03; txBuf[2] = (uint8_t)bc;
  sendFrame(txBuf, 3 + bc);
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
    Serial.printf("CRC FAIL got=%04X calc=%04X\n", rxCRC, calcCRC);
    rxLen = 0; return;
  }
  Serial.print("RX: ");
  for (int i = 0; i < rxLen; i++) Serial.printf("%02X ", rxBuf[i]);
  Serial.println();
  uint8_t  fc  = rxBuf[1];
  uint16_t reg = ((uint16_t)rxBuf[2] << 8) | rxBuf[3];
  uint16_t qty = ((uint16_t)rxBuf[4] << 8) | rxBuf[5];
  takeSnapshot();            // freeze live values for this response
  if (fc == 0x03) handleFC03(reg, qty);
  else Serial.printf("Unsupported FC 0x%02X\n", fc);
  rxLen = 0;
}

// ============================================================
//  MQTT callback
// ============================================================
void mqttCallback(char *topic, byte *payload, unsigned int length) {
  char buf[32] = {0};
  if (length > 31) length = 31;
  memcpy(buf, payload, length);
  float val = atof(buf);
  String t(topic);
  Serial.printf("[MQTT] %s = %s\n", topic, buf);
  if      (t.endsWith("mains/voltage"))            lv_mainsV    = val;
  else if (t.endsWith("mains/frequency"))           lv_mainsHz   = val;
  else if (t.endsWith("battery/voltage"))           lv_battV     = val;
  else if (t.endsWith("battery/charging_amp"))      lv_battChgA  = val;
  else if (t.endsWith("battery/discharging_amp"))   lv_battDisA  = val;
  else if (t.endsWith("battery/soc"))               lv_battSOC   = val;
  else if (t.endsWith("pv/voltage"))                lv_pvV       = val;
  else if (t.endsWith("pv/current"))                lv_pvA       = val;
  else if (t.endsWith("pv/power_w"))                lv_pvW       = val;
  else if (t.endsWith("load/ac_w"))                 lv_loadW     = val;
}

// ============================================================
//  WiFi connect with watchdog-safe yield
// ============================================================
void connectWiFi() {
  Serial.printf("\nConnecting WiFi: %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    yield();                 // feed ESP8266 watchdog
    if (++tries > 40) { Serial.println("\nWiFi timeout — reboot"); ESP.restart(); }
  }
  Serial.printf("\nWiFi OK  IP=%s\n", WiFi.localIP().toString().c_str());
}

// ============================================================
//  MQTT connect + subscribe
// ============================================================
void connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.printf("MQTT connecting %s:%d ...\n", MQTT_BROKER, MQTT_PORT);
    bool ok = (strlen(MQTT_USER) > 0)
      ? mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS_STR)
      : mqttClient.connect(MQTT_CLIENT_ID);
    if (ok) {
      Serial.println("MQTT connected");
      mqttClient.subscribe(BASE_TOPIC "#");
      Serial.println("Subscribed: " BASE_TOPIC "#");
    } else {
      Serial.printf("MQTT failed rc=%d retry 5s\n", mqttClient.state());
      for (uint8_t i = 0; i < 10; i++) { delay(500); yield(); }
    }
  }
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== SRNE DESS Emulator + MQTT  v3.0 ESP-12F ===");
  Serial.printf("SlaveID=%d  Baud=%d  RX=GPIO%d  TX=GPIO%d\n",
                SLAVE_ID, BAUD_RATE, LOGGER_RX_PIN, LOGGER_TX_PIN);
  Serial.println("LoadPct = W/6200*100  |  VA = W/0.95");

  connectWiFi();
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);     // enough for retained payloads
  connectMQTT();

  loggerSerial.begin(BAUD_RATE);
  Serial.println("Ready — waiting for DESS logger polls...");
}

// ============================================================
//  Loop
// ============================================================
void loop() {
  // MQTT keep-alive
  if (!mqttClient.connected()) connectMQTT();
  mqttClient.loop();

  // Read Modbus bytes from SoftwareSerial
  while (loggerSerial.available()) {
    uint8_t b = (uint8_t)loggerSerial.read();
    if (rxLen < sizeof(rxBuf)) rxBuf[rxLen++] = b;
    lastByte = millis();
  }

  // Process on 20 ms inter-frame gap
  if (rxLen > 0 && (millis() - lastByte) > 20) {
    processFrame();
    memset(rxBuf, 0, sizeof(rxBuf));
    rxLen = 0;
  }

  yield();   // keep watchdog happy between polls
}
