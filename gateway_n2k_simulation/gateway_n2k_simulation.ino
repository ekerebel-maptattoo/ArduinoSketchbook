// MAPTATTOO — BLE Gateway Simulator (NO N2K / NO TWAI)
// Board: Seeed XIAO ESP32-C6
//
// What this version does:
// - Keeps the same BLE service UUID and per-PGN characteristic UUID scheme.
// - Keeps Option A framing: first byte of every BLE notification payload is SRC.
// - Removes all CAN/TWAI/N2K ingestion and fast-packet reassembly.
// - Generates simulated ownship data about 5 NM north of Monterey Harbor.
// - Ownship moves slowly north at 5 kt.
// - Generates 15 simulated AIS targets in Monterey Bay.
// - Simulates the other PGNs with smooth oscillating values.
// - Keeps external antenna selection, RGB LED behavior, buzzer, RSSI pseudo-PGN, heap/status summaries.
//
// AIS update rates follow ITU-R M.1371 / IEC 62287:
//   Class A dynamic: 2s (>23kt), 6s (14-23kt), 10s (<14kt), 3min (<2kt)
//   Class B dynamic: 15s (>14kt), 30s (2-14kt), 3min (<2kt)
//   AtoN:            3 min
//   Static/voyage:   6 min all types
//
// IMPORTANT:
// This sketch generates payload bytes that follow common NMEA 2000 little-endian/scaled conventions
// for the PGNs used here. If your MAPTATTOO receiver expects a custom decoded payload instead of
// raw N2K PGN payloads, tell me and I can adapt the packing exactly to your app-side decoder.

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "sim_types.h"

#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <cstring>

#include <esp_system.h>      // ESP.getEfuseMac()
#include <esp_heap_caps.h>   // heap_caps_get_largest_free_block()

extern "C" {
  #include "host/ble_gap.h"   // ble_gap_conn_rssi()
}

// -------- Tuning / Logging --------
static const bool LOG_NOTIFY_FLOW      = false;
static const bool LOG_SUBSCRIPTION     = true;
static const uint32_t LOG_SUMMARY_MS   = 5000;

// -------- Pins --------
static const uint8_t BUZZER_PIN = 20;

// External RGB LED (Dialight 5218559F = common-anode)
static const gpio_num_t LED_EXT_GREEN = GPIO_NUM_0;
static const gpio_num_t LED_EXT_BLUE  = GPIO_NUM_1;
static const gpio_num_t LED_EXT_RED   = GPIO_NUM_2;

// Simulator treats synthetic data as "N2K active" if generated recently.
static const uint32_t N2K_ACTIVE_TIMEOUT_MS = 15000;

// -------- LED brightness / PWM --------
static const float LED_BRIGHTNESS = 0.1f;
static const uint32_t LED_PWM_FREQ = 5000;
static const uint8_t  LED_PWM_RES  = 8;

// -------- Firmware identity --------
static const char* FW_VERSION = "1.0.8-sim";

// -------- BLE --------
static std::string gBleName;
static const std::string SVC_UUID = "8f000000-1a12-4a86-9e26-9f1c9f0e1234";

static NimBLEServer*       gSrv = nullptr;
static NimBLEService*      gSvc = nullptr;
static NimBLEAdvertising*  gAdv = nullptr;
static std::map<uint32_t, NimBLECharacteristic*> gCharByPGN;
static std::map<uint32_t, bool> gSubscribedByPGN;

static const uint8_t MAX_CLIENTS = 3;
static const uint8_t SRC_GATEWAY = 0xFF;

// -------- Connection pause --------
// When a new BLE client connects, pause all data notifications for this long
// to give it time to complete service discovery and subscribe to characteristics
// without being flooded by notifications during setup.
static const uint32_t CONNECT_PAUSE_MS      = 15000;
static uint32_t       gConnectingPauseUntilMs = 0;

static inline bool isConnectPaused(uint32_t nowMs) {
  return nowMs < gConnectingPauseUntilMs;
}

static const uint8_t SRC_OWNSHIP_NAV    = 0x23;
static const uint8_t SRC_OWNSHIP_ENV    = 0x24;
static const uint8_t SRC_OWNSHIP_ENGINE = 0x25;
// SRC_AIS_RECEIVER: fixed N2K source address used for ALL simulated AIS PGNs.
// On a real N2K bus, the source address identifies the AIS receiver device,
// not the individual vessel — vessels are distinguished by MMSI in the payload.
static const uint8_t SRC_AIS_RECEIVER   = 0x01;

// -------- Debug counters --------
static uint32_t gSimFramesGenerated  = 0;
static uint32_t gBleNotifyAttempts   = 0;
static uint32_t gBleNotifySent       = 0;
static uint32_t gBleNotifyFailed     = 0;
static uint32_t gBleNotifyNoChar     = 0;
static uint32_t gBleNotifyNoSub      = 0;
static uint32_t gBleNotifyThrottled  = 0;

// -------- LED / state tracking --------
static uint32_t gLastN2KRxMs = 0;

// -------- Simulation constants --------
static constexpr double DEG_TO_RAD_D  = 0.017453292519943295;
static constexpr double RAD_TO_DEG_D  = 57.29577951308232;
static constexpr double EARTH_RADIUS_M = 6371000.0;
static constexpr double KNOT_TO_MPS   = 0.514444;
static constexpr double NM_TO_M       = 1852.0;

// Monterey Harbor approximate reference point.
static constexpr double MONTEREY_HARBOR_LAT = 36.6044;
static constexpr double MONTEREY_HARBOR_LON = -121.8906;

// Ownship starts ~5 NM north of Monterey Harbor and moves north at 5 kt.
static constexpr double OWNSHIP_START_LAT = MONTEREY_HARBOR_LAT + (5.0 / 60.0);
static constexpr double OWNSHIP_START_LON = MONTEREY_HARBOR_LON;
static constexpr double OWNSHIP_SOG_KT    = 5.0;
static constexpr double OWNSHIP_COG_DEG   = 0.0;

static uint32_t gSimStartMs = 0;

// -------- Helpers --------
static void hexDump(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    Serial.printf("%02X", data[i]);
    if (i + 1 < len) Serial.print(" ");
  }
}

static float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

static double wrap360(double deg) {
  while (deg < 0.0) deg += 360.0;
  while (deg >= 360.0) deg -= 360.0;
  return deg;
}

static double degToRad(double deg) { return deg * DEG_TO_RAD_D; }
static double radToDeg(double rad) { return rad * RAD_TO_DEG_D; }

static double oscillate(double base, double amp, double periodSec, double phase, double tSec) {
  return base + amp * sin((2.0 * M_PI * tSec / periodSec) + phase);
}

static void movePoint(double latDeg, double lonDeg, double courseDeg, double distanceM,
                      double& outLatDeg, double& outLonDeg) {
  const double lat1  = degToRad(latDeg);
  const double lon1  = degToRad(lonDeg);
  const double brng  = degToRad(courseDeg);
  const double dr    = distanceM / EARTH_RADIUS_M;
  const double sinL1 = sin(lat1);
  const double cosL1 = cos(lat1);
  const double sinDr = sin(dr);
  const double cosDr = cos(dr);
  const double lat2  = asin(sinL1 * cosDr + cosL1 * sinDr * cos(brng));
  const double lon2  = lon1 + atan2(sin(brng) * sinDr * cosL1, cosDr - sinL1 * sin(lat2));
  outLatDeg = radToDeg(lat2);
  outLonDeg = radToDeg(lon2);
}

static void appendU8 (std::vector<uint8_t>& v, uint8_t  x) { v.push_back(x); }
static void appendI8 (std::vector<uint8_t>& v, int8_t   x) { v.push_back((uint8_t)x); }

static void appendU16LE(std::vector<uint8_t>& v, uint16_t x) {
  v.push_back((uint8_t)(x & 0xFF));
  v.push_back((uint8_t)((x >> 8) & 0xFF));
}
static void appendI16LE(std::vector<uint8_t>& v, int16_t x) { appendU16LE(v, (uint16_t)x); }

static void appendU32LE(std::vector<uint8_t>& v, uint32_t x) {
  v.push_back((uint8_t)(x & 0xFF));
  v.push_back((uint8_t)((x >> 8)  & 0xFF));
  v.push_back((uint8_t)((x >> 16) & 0xFF));
  v.push_back((uint8_t)((x >> 24) & 0xFF));
}
static void appendI32LE(std::vector<uint8_t>& v, int32_t x) { appendU32LE(v, (uint32_t)x); }

static void appendU64LE(std::vector<uint8_t>& v, uint64_t x) {
  for (int i = 0; i < 8; ++i) v.push_back((uint8_t)((x >> (8 * i)) & 0xFF));
}
static void appendI64LE(std::vector<uint8_t>& v, int64_t x) { appendU64LE(v, (uint64_t)x); }

static uint16_t rad1e4(double deg) {
  long val = lround(degToRad(deg) * 10000.0);
  if (val < 0) val = 0;
  if (val > 65534) val = 65534;
  return (uint16_t)val;
}

static int16_t signedRad1e4(double deg) {
  long val = lround(degToRad(deg) * 10000.0);
  if (val < -32767) val = -32767;
  if (val >  32767) val =  32767;
  return (int16_t)val;
}

static uint16_t speedCms(double knots) {
  long val = lround(knots * KNOT_TO_MPS * 100.0);
  if (val < 0) val = 0;
  if (val > 65534) val = 65534;
  return (uint16_t)val;
}

// PGN 129038/129039/129040 SOG field: 0.01 knots resolution (centikts)
static uint16_t speedCentikts(double knots) {
  long val = lround(knots * 100.0);
  if (val < 0) val = 0;
  if (val > 65534) val = 65534;
  return (uint16_t)val;
}

static uint16_t tempK_0p01C(double celsius) {
  long val = lround((celsius + 273.15) * 100.0);
  if (val < 0) val = 0;
  if (val > 65534) val = 65534;
  return (uint16_t)val;
}

static uint32_t pressurePa_0p1(double pa) {
  double val = pa * 10.0;
  if (val < 0) val = 0;
  if (val > 0xFFFFFFFEUL) val = 0xFFFFFFFEUL;
  return (uint32_t)lround(val);
}

// --- External RGB LED helpers ---
static void setExternalLed(bool redOn, bool greenOn, bool blueOn) {
  const float b = clamp01(LED_BRIGHTNESS);
  const uint32_t maxDuty = (1u << LED_PWM_RES) - 1u;
  const uint32_t dutyOff = maxDuty;
  const uint32_t dutyOn  = (uint32_t)lroundf((1.0f - b) * (float)maxDuty);
  ledcWrite((uint8_t)LED_EXT_RED,   redOn   ? dutyOn : dutyOff);
  ledcWrite((uint8_t)LED_EXT_GREEN, greenOn ? dutyOn : dutyOff);
  ledcWrite((uint8_t)LED_EXT_BLUE,  blueOn  ? dutyOn : dutyOff);
}

static void externalLedOff() { setExternalLed(false, false, false); }
static bool isBleConnected() { return (gSrv && gSrv->getConnectedCount() > 0); }

static bool isN2KActive(uint32_t nowMs) {
  return (gLastN2KRxMs != 0) && ((nowMs - gLastN2KRxMs) <= N2K_ACTIVE_TIMEOUT_MS);
}

static void updateStatusLed(uint32_t nowMs) {
  static bool blinkPhase = false;
  static uint32_t lastBlinkToggleMs = 0;
  const bool bleConnected = isBleConnected();
  const bool n2kActive    = isN2KActive(nowMs);

  if (bleConnected && !n2kActive) {
    if (nowMs - lastBlinkToggleMs >= 500) { lastBlinkToggleMs = nowMs; blinkPhase = !blinkPhase; }
    setExternalLed(false, false, blinkPhase);
    return;
  }
  if (bleConnected && n2kActive)  { setExternalLed(false, false, true);  return; }
  if (!bleConnected && n2kActive) { setExternalLed(false, true,  false); return; }
  setExternalLed(true, false, false);
}

static void selectExternalAntenna() {
  pinMode(GPIO_NUM_3,  OUTPUT); digitalWrite(GPIO_NUM_3,  LOW);
  pinMode(GPIO_NUM_14, OUTPUT); digitalWrite(GPIO_NUM_14, HIGH);
  delay(10);
}

static int8_t getClientRSSI() {
  if (!gSrv) return -128;
  const std::vector<uint16_t> peers = gSrv->getPeerDevices();
  if (peers.empty()) return -128;
  bool have = false;
  int8_t rssi_min = 127;
  for (uint16_t h : peers) {
    int8_t rssi = 0;
    if (ble_gap_conn_rssi(h, &rssi) == 0 && rssi != 127) {
      if (!have || rssi < rssi_min) rssi_min = rssi;
      have = true;
    }
  }
  return have ? rssi_min : -128;
}

static void printConnectedRSSI_1Hz(uint32_t nowMs) {
  static uint32_t lastPrintMs = 0;
  if (nowMs - lastPrintMs < 1000) return;
  lastPrintMs = nowMs;
  if (!gSrv || gSrv->getConnectedCount() == 0) return;
  const int8_t rssi = getClientRSSI();
  if (rssi != -128) Serial.printf("RSSI (connected) = %d dBm\n", (int)rssi);
}

static void printSummary(uint32_t nowMs) {
  static uint32_t lastSummaryMs = 0;
  if (nowMs - lastSummaryMs < LOG_SUMMARY_MS) return;
  lastSummaryMs = nowMs;
  Serial.printf(
    "SUMMARY simFrames=%lu notifyAttempts=%lu sent=%lu failed=%lu noChar=%lu noSub=%lu throttled=%lu connected=%u adv=%u simActive=%u\n",
    (unsigned long)gSimFramesGenerated, (unsigned long)gBleNotifyAttempts,
    (unsigned long)gBleNotifySent, (unsigned long)gBleNotifyFailed,
    (unsigned long)gBleNotifyNoChar, (unsigned long)gBleNotifyNoSub,
    (unsigned long)gBleNotifyThrottled,
    gSrv ? (unsigned)gSrv->getConnectedCount() : 0,
    gAdv ? (unsigned)gAdv->isAdvertising() : 0,
    (unsigned)isN2KActive(nowMs));
}

static void printHeap_60s(uint32_t nowMs) {
  static uint32_t lastHeapPrintMs = 0;
  if (nowMs - lastHeapPrintMs < 60000) return;
  lastHeapPrintMs = nowMs;
  Serial.printf("HEAP free=%u minFree=%u largest=%u\n",
    (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
    (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}

static void beepOnBleConnect() {
  tone(BUZZER_PIN, 800, 250);
  delay(250);
  noTone(BUZZER_PIN);
}

static const char* discReasonStr(int r) {
  switch (r) {
    case 0x08: return "Connection Timeout (often supervision timeout / RF loss)";
    case 0x13: return "Remote User Terminated (peer explicitly disconnected)";
    case 0x16: return "Local Host Terminated (gateway side explicitly disconnected)";
    case 0x3E: return "Connection Failed to be Established";
    default:   return "Other/Unknown";
  }
}

class SrvCB : public NimBLEServerCallbacks {
public:
  void onConnect(NimBLEServer* s, NimBLEConnInfo& ci) override {
    Serial.printf("BLE CONNECT from %s handle=%u (count=%u)\n",
                  ci.getAddress().toString().c_str(),
                  (unsigned)ci.getConnHandle(), (unsigned)s->getConnectedCount());
    beepOnBleConnect();
    // Pause all outbound notifications so the new client can complete
    // service discovery and subscribe without being flooded.
    gConnectingPauseUntilMs = millis() + CONNECT_PAUSE_MS;
    Serial.printf("CONNECT PAUSE: notifications paused for %lu ms\n", (unsigned long)CONNECT_PAUSE_MS);
    s->updateConnParams(ci.getConnHandle(), 40, 120, 0, 1200);
    if (s->getConnectedCount() >= MAX_CLIENTS) {
      Serial.printf("Reached MAX_CLIENTS=%u -> stop advertising\n", (unsigned)MAX_CLIENTS);
      NimBLEDevice::stopAdvertising();
    } else {
      NimBLEDevice::startAdvertising();
    }
  }

  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& ci, int reason) override {
    Serial.printf("BLE DISCONNECT from %s handle=%u reason=0x%02X (%s) (count=%u)\n",
                  ci.getAddress().toString().c_str(), (unsigned)ci.getConnHandle(),
                  reason, discReasonStr(reason), (unsigned)s->getConnectedCount());
    for (auto &kv : gSubscribedByPGN) kv.second = false;
    if (s->getConnectedCount() < MAX_CLIENTS) { delay(80); NimBLEDevice::startAdvertising(); }
  }
};

static std::string chipIdString() {
  uint64_t mac = ESP.getEfuseMac();
  char buf[32];
  snprintf(buf, sizeof(buf), "%02X%02X%02X%02X%02X%02X",
    (uint8_t)(mac >> 40), (uint8_t)(mac >> 32), (uint8_t)(mac >> 24),
    (uint8_t)(mac >> 16), (uint8_t)(mac >>  8), (uint8_t)(mac >>  0));
  return std::string(buf);
}

class CharCB : public NimBLECharacteristicCallbacks {
public:
  explicit CharCB(uint32_t pgn) : pgn_(pgn) {}
  void onSubscribe(NimBLECharacteristic*, NimBLEConnInfo& ci, uint16_t subValue) override {
    const bool subscribed = (subValue != 0);
    gSubscribedByPGN[pgn_] = subscribed;
    if (LOG_SUBSCRIPTION) {
      Serial.printf("SUBSCRIBE PGN %u from %s handle=%u -> %s (subValue=0x%04X)\n",
        (unsigned)pgn_, ci.getAddress().toString().c_str(),
        (unsigned)ci.getConnHandle(), subscribed ? "ON" : "OFF", (unsigned)subValue);
    }
  }
private:
  uint32_t pgn_;
};

static inline bool hasSubscribers(uint32_t pgn) {
  auto it = gSubscribedByPGN.find(pgn);
  return (it != gSubscribedByPGN.end() && it->second);
}

static std::string makeCharUUID(uint32_t pgn) {
  char buf[64];
  snprintf(buf, sizeof(buf), "8f00%04X-1a12-4a86-9e26-9f1c9f0e1234", (unsigned)(pgn & 0xFFFF));
  return std::string(buf);
}

static NimBLECharacteristic* addNotifyChar(uint32_t pgn, const char* userDesc) {
  NimBLEUUID cuuid(makeCharUUID(pgn));
  auto* c = gSvc->createCharacteristic(cuuid, NIMBLE_PROPERTY::NOTIFY);
  if (c) {
    NimBLEDescriptor* ud = c->createDescriptor("2901", NIMBLE_PROPERTY::READ);
    if (ud) ud->setValue(std::string(userDesc));
    gSubscribedByPGN[pgn] = false;
    c->setCallbacks(new CharCB(pgn));
    Serial.printf("BLE characteristic added: PGN=%u UUID=%s DESC=%s\n",
                  (unsigned)pgn, makeCharUUID(pgn).c_str(), userDesc);
  } else {
    Serial.printf("ERROR: failed to create characteristic for PGN=%u\n", (unsigned)pgn);
  }
  return c;
}

static void validatePgnUuidUniquenessOrHalt() {
  std::map<uint16_t, uint32_t> seen;
  bool ok = true;
  for (const auto& kv : gCharByPGN) {
    const uint32_t pgn = kv.first;
    const uint16_t lo  = (uint16_t)(pgn & 0xFFFF);
    auto it = seen.find(lo);
    if (it != seen.end() && it->second != pgn) {
      ok = false;
      Serial.printf("ERROR: UUID collision: PGN %u and PGN %u share low16=0x%04X\n",
                    (unsigned)it->second, (unsigned)pgn, (unsigned)lo);
    } else { seen[lo] = pgn; }
  }
  if (!ok) {
    Serial.println("ERROR: PGN UUID collisions detected. Fix makeCharUUID() mapping before shipping.");
    while (1) { setExternalLed(true, false, false); delay(200); }
  }
}

static void setupBLE() {
  gBleName = std::string("MAPTATTOO N2K");
  NimBLEDevice::init(gBleName);
  NimBLEDevice::setDeviceName(gBleName);
  NimBLEDevice::setPower(ESP_PWR_LVL_P3);
  NimBLEDevice::setMTU(247);
  NimBLEDevice::setSecurityAuth(false, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  gSrv = NimBLEDevice::createServer();
  gSrv->setCallbacks(new SrvCB());

  NimBLEService* dis = gSrv->createService(NimBLEUUID((uint16_t)0x180A));
  if (dis) {
    auto* manu   = dis->createCharacteristic(NimBLEUUID((uint16_t)0x2A29), NIMBLE_PROPERTY::READ);
    auto* model  = dis->createCharacteristic(NimBLEUUID((uint16_t)0x2A24), NIMBLE_PROPERTY::READ);
    auto* serial = dis->createCharacteristic(NimBLEUUID((uint16_t)0x2A25), NIMBLE_PROPERTY::READ);
    auto* fw     = dis->createCharacteristic(NimBLEUUID((uint16_t)0x2A26), NIMBLE_PROPERTY::READ);
    if (manu)   manu->setValue("MAPTATTOO");
    if (model)  model->setValue("N2K Gateway Simulator (XIAO ESP32-C6)");
    if (serial) serial->setValue(chipIdString());
    if (fw)     fw->setValue(std::string(FW_VERSION));
    dis->start();
  }

  gSvc = gSrv->createService(NimBLEUUID(SVC_UUID));

  gCharByPGN[65281]  = addNotifyChar(65281,  "PGN 65281 RSSI Status");
  gCharByPGN[129025] = addNotifyChar(129025, "PGN 129025 Position");
  gCharByPGN[129026] = addNotifyChar(129026, "PGN 129026 COG/SOG");
  gCharByPGN[126992] = addNotifyChar(126992, "PGN 126992 System Time");
  gCharByPGN[129029] = addNotifyChar(129029, "PGN 129029 GNSS");
  gCharByPGN[128259] = addNotifyChar(128259, "PGN 128259 Water Speed");
  gCharByPGN[128267] = addNotifyChar(128267, "PGN 128267 Water Depth");
  gCharByPGN[127250] = addNotifyChar(127250, "PGN 127250 Vessel Heading");
  gCharByPGN[127257] = addNotifyChar(127257, "PGN 127257 Attitude");
  gCharByPGN[130306] = addNotifyChar(130306, "PGN 130306 Wind Data");
  gCharByPGN[130310] = addNotifyChar(130310, "PGN 130310 Env Params");
  gCharByPGN[130311] = addNotifyChar(130311, "PGN 130311 Env Params (old)");
  gCharByPGN[130312] = addNotifyChar(130312, "PGN 130312 Temperature");
  gCharByPGN[130314] = addNotifyChar(130314, "PGN 130314 Actual Pressure");
  gCharByPGN[130316] = addNotifyChar(130316, "PGN 130316 Temp Extended");
  gCharByPGN[127505] = addNotifyChar(127505, "PGN 127505 Fluid Level");
  gCharByPGN[127508] = addNotifyChar(127508, "PGN 127508 Battery Status");
  gCharByPGN[127506] = addNotifyChar(127506, "PGN 127506 DC Detailed Status");
  gCharByPGN[127488] = addNotifyChar(127488, "PGN 127488 Engine Rapid");
  gCharByPGN[127489] = addNotifyChar(127489, "PGN 127489 Engine Dynamic");
  gCharByPGN[127493] = addNotifyChar(127493, "PGN 127493 Transmission Dyn");
  gCharByPGN[127245] = addNotifyChar(127245, "PGN 127245 Rudder");
  gCharByPGN[129038] = addNotifyChar(129038, "PGN 129038 AIS Class A Position");
  gCharByPGN[129039] = addNotifyChar(129039, "PGN 129039 AIS Class B Position");
  gCharByPGN[129040] = addNotifyChar(129040, "PGN 129040 AIS Class B Extended Position");
  gCharByPGN[129041] = addNotifyChar(129041, "PGN 129041 AIS AtoN");
  gCharByPGN[129793] = addNotifyChar(129793, "PGN 129793 AIS UTC/Date Report");
  gCharByPGN[129794] = addNotifyChar(129794, "PGN 129794 AIS Class A Static/Voyage");
  gCharByPGN[129809] = addNotifyChar(129809, "PGN 129809 AIS Class B Static Part A");
  gCharByPGN[129810] = addNotifyChar(129810, "PGN 129810 AIS Class B Static Part B");

  validatePgnUuidUniquenessOrHalt();
  gSvc->start();

  gAdv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData advData;
  advData.setName(gBleName);
  advData.setFlags(0x06);
  advData.addServiceUUID(NimBLEUUID(SVC_UUID));
  gAdv->setAdvertisementData(advData);
  gAdv->setMinInterval(0x00A0);
  gAdv->setMaxInterval(0x0140);
  gAdv->start();
  Serial.printf("BLE up, advertising as %s.\n", gBleName.c_str());
}

static inline bool isAIS(uint32_t pgn) {
  switch (pgn) {
    case 129038: case 129039: case 129040: case 129041:
    case 129793: case 129794: case 129809: case 129810:
      return true;
    default: return false;
  }
}

static std::map<uint64_t, uint32_t> lastTxMs;

static inline uint64_t pgnSrcKey(uint32_t pgn, uint8_t src) {
  return ((uint64_t)pgn << 8) | (uint64_t)src;
}

static void notifyPGN(uint32_t pgn, uint8_t src, const uint8_t* payload, size_t len, bool is_ais) {
  gBleNotifyAttempts++;

  // Suppress notifications while a client is connecting / doing service discovery.
  if (isConnectPaused(millis())) { gBleNotifyThrottled++; return; }

  auto it = gCharByPGN.find(pgn);
  if (it == gCharByPGN.end() || it->second == nullptr) { gBleNotifyNoChar++; return; }
  if (!hasSubscribers(pgn)) { gBleNotifyNoSub++; return; }

  uint32_t now = millis();
  const uint32_t minIntervalMs = is_ais ? 100 : 1000;
  uint64_t k = pgnSrcKey(pgn, src);
  uint32_t &lt = lastTxMs[k];
  if (now - lt < minIntervalMs) { gBleNotifyThrottled++; return; }
  lt = now;

  uint8_t framed[224];
  if (len + 1 > sizeof(framed)) {
    Serial.printf("NOTIFY DROP PGN=%u SRC=%u len=%u reason=payload too large\n",
                  (unsigned)pgn, (unsigned)src, (unsigned)len);
    return;
  }
  framed[0] = src;
  memcpy(&framed[1], payload, len);

  if (LOG_NOTIFY_FLOW) {
    Serial.printf("NOTIFY SEND PGN=%u SRC=%u framedLen=%u data=",
                  (unsigned)pgn, (unsigned)src, (unsigned)(len + 1));
    hexDump(framed, len + 1);
    Serial.println();
  }

  it->second->setValue(framed, len + 1);
  bool ok = it->second->notify();
  if (ok) gBleNotifySent++;
  else {
    gBleNotifyFailed++;
    Serial.printf("BLE notify failed PGN=%u SRC=%u len=%u\n",
                  (unsigned)pgn, (unsigned)src, (unsigned)(len + 1));
  }
}

// -------- Simulated N2K payload builders --------

static std::vector<uint8_t> buildPGN129025_Position(double latDeg, double lonDeg) {
  std::vector<uint8_t> p;
  appendI32LE(p, (int32_t)llround(latDeg * 1e7));
  appendI32LE(p, (int32_t)llround(lonDeg * 1e7));
  return p;
}

static std::vector<uint8_t> buildPGN129026_CogSog(double cogDeg, double sogKt) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00);
  appendU8(p, 0xFC);
  appendU16LE(p, rad1e4(cogDeg));
  appendU16LE(p, speedCms(sogKt));
  appendU16LE(p, 0xFFFF);
  return p;
}

static std::vector<uint8_t> buildPGN127250_Heading(double headingDeg, double variationDeg) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00);
  appendU16LE(p, rad1e4(headingDeg));
  appendI16LE(p, signedRad1e4(0.0));
  appendI16LE(p, signedRad1e4(variationDeg));
  appendU8(p, 0xFC);
  return p;
}

static std::vector<uint8_t> buildPGN127257_Attitude(double yawDeg, double pitchDeg, double rollDeg) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00);
  appendI16LE(p, signedRad1e4(yawDeg));
  appendI16LE(p, signedRad1e4(pitchDeg));
  appendI16LE(p, signedRad1e4(rollDeg));
  appendU8(p, 0xFF);
  return p;
}

static std::vector<uint8_t> buildPGN128259_WaterSpeed(double waterSpeedKt) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00);
  appendU16LE(p, speedCms(waterSpeedKt));
  appendU16LE(p, 0xFFFF);
  appendU8(p, 0xFF); appendU8(p, 0xFF); appendU8(p, 0xFF);
  return p;
}

static std::vector<uint8_t> buildPGN128267_Depth(double depthM, double offsetM) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00);
  appendU32LE(p, (uint32_t)lround(depthM * 100.0));
  appendI16LE(p, (int16_t)lround(offsetM * 1000.0));
  appendU8(p, 0xFF);
  return p;
}

static std::vector<uint8_t> buildPGN130306_Wind(double windSpeedKt, double windAngleDeg) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00);
  appendU16LE(p, speedCms(windSpeedKt));
  appendU16LE(p, rad1e4(windAngleDeg));
  appendU8(p, 0xFD); appendU8(p, 0xFF); appendU8(p, 0xFF);
  return p;
}

static std::vector<uint8_t> buildPGN130310_Env(double waterTempC, double airTempC, double pressurePa) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00);
  appendU16LE(p, tempK_0p01C(waterTempC));
  appendU16LE(p, tempK_0p01C(airTempC));
  appendU16LE(p, (uint16_t)lround(pressurePa / 100.0));
  appendU8(p, 0xFF);
  return p;
}

static std::vector<uint8_t> buildPGN130311_EnvOld(double tempC, double humidityPct, double pressurePa) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00); appendU8(p, 0x00);
  appendU16LE(p, tempK_0p01C(tempC));
  appendU16LE(p, (uint16_t)lround(humidityPct * 250.0));
  appendU16LE(p, (uint16_t)lround(pressurePa / 100.0));
  return p;
}

static std::vector<uint8_t> buildPGN130312_Temperature(double tempC) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00); appendU8(p, 0x00); appendU8(p, 0x01);
  appendU16LE(p, tempK_0p01C(tempC));
  appendI16LE(p, 0x7FFF);
  appendU8(p, 0xFF);
  return p;
}

static std::vector<uint8_t> buildPGN130314_Pressure(double pressurePa) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00); appendU8(p, 0x00); appendU8(p, 0x00);
  appendU32LE(p, pressurePa_0p1(pressurePa));
  appendU8(p, 0xFF);
  return p;
}

static std::vector<uint8_t> buildPGN130316_TempExtended(double tempC) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00); appendU8(p, 0x00); appendU8(p, 0x01);
  appendI32LE(p, (int32_t)lround((tempC + 273.15) * 1000.0));
  appendU8(p, 0xFF);
  return p;
}

static std::vector<uint8_t> buildPGN127505_Fluid(double levelPct, double capacityL) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00);
  appendU16LE(p, (uint16_t)lround(levelPct * 250.0));
  appendU32LE(p, (uint32_t)lround(capacityL * 10.0));
  appendU8(p, 0xFF);
  return p;
}

static std::vector<uint8_t> buildPGN127508_Battery(double volts, double amps, double tempC) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00);
  appendU16LE(p, (uint16_t)lround(volts * 100.0));
  appendI16LE(p, (int16_t)lround(amps  * 10.0));
  appendU16LE(p, tempK_0p01C(tempC));
  appendU8(p, 0xFF);
  return p;
}

static std::vector<uint8_t> buildPGN127506_DcDetailed(double volts, double amps, double socPct) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00); appendU8(p, 0x00); appendU8(p, 0x01); appendU8(p, 0xFC);
  appendU16LE(p, (uint16_t)lround(socPct * 250.0));
  appendU16LE(p, (uint16_t)lround(95.0   * 250.0));
  appendU16LE(p, (uint16_t)lround(60.0   * 60.0));
  appendU16LE(p, 0);
  appendU16LE(p, (uint16_t)lround(volts * 100.0));
  appendI16LE(p, (int16_t)lround(amps * 10.0));
  return p;
}

static std::vector<uint8_t> buildPGN127488_EngineRapid(double rpm) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00);
  appendU16LE(p, (uint16_t)lround(rpm * 4.0));
  appendU16LE(p, (uint16_t)lround(25.0 * 100.0));
  appendI8(p, 0); appendU8(p, 0xFF); appendU8(p, 0xFF);
  return p;
}

static std::vector<uint8_t> buildPGN127489_EngineDynamic(double oilPa, double coolantC,
                                                           double altV, double fuelLph, double hours) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00);
  appendU16LE(p, (uint16_t)lround(oilPa / 100.0));
  appendU16LE(p, tempK_0p01C(coolantC));
  appendU16LE(p, tempK_0p01C(65.0));
  appendU16LE(p, (uint16_t)lround(altV * 100.0));
  appendI16LE(p, (int16_t)lround(6.5 * 10.0));
  appendI16LE(p, (int16_t)lround(fuelLph * 10.0));
  appendU32LE(p, (uint32_t)lround(hours * 3600.0));
  appendU16LE(p, 0x0000); appendU16LE(p, 0xFFFF);
  appendU8(p, 0x00); appendU8(p, 0x00);
  return p;
}

static std::vector<uint8_t> buildPGN127493_Transmission(double oilPa, double oilTempC) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00); appendU8(p, 0x00);
  appendU16LE(p, (uint16_t)lround(oilPa / 100.0));
  appendU16LE(p, tempK_0p01C(oilTempC));
  appendU8(p, 0x00); appendU8(p, 0x00);
  return p;
}

static std::vector<uint8_t> buildPGN127245_Rudder(double rudderDeg) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00); appendU8(p, 0xFF);
  appendI16LE(p, signedRad1e4(rudderDeg));
  appendI16LE(p, 0x7FFF);
  appendU8(p, 0xFF); appendU8(p, 0xFF);
  return p;
}

static std::vector<uint8_t> buildPGN126992_SystemTime(uint32_t nowMs) {
  const uint32_t simSeconds  = nowMs / 1000;
  const uint16_t daysSince70 = 20000 + (simSeconds / 86400);
  const uint32_t secondsToday = simSeconds % 86400;
  std::vector<uint8_t> p;
  appendU8(p, 0x00); appendU8(p, 0x00);
  appendU16LE(p, daysSince70);
  appendU32LE(p, secondsToday * 10000UL);
  return p;
}

static std::vector<uint8_t> buildPGN129029_GNSS(double latDeg, double lonDeg, double altM) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00);
  appendU16LE(p, 20000);
  appendU32LE(p, (millis() / 1000UL % 86400UL) * 10000UL);
  appendI64LE(p, (int64_t)llround(latDeg * 1e16));
  appendI64LE(p, (int64_t)llround(lonDeg * 1e16));
  appendI64LE(p, (int64_t)llround(altM   * 1e6));
  appendU8(p, 0x01); appendU8(p, 0x00); appendU8(p, 12);
  appendI16LE(p, (int16_t)lround(0.8 * 100.0));
  appendI16LE(p, (int16_t)lround(1.2 * 100.0));
  appendI32LE(p, 0); appendU8(p, 0);
  return p;
}

// -------- N2K bit-packing helper --------
// N2K uses LSB-first bit ordering within bytes (little-endian bit stream).
// Sets numBits of value starting at startBit in buf.
static void n2kSetBits(uint8_t* buf, int startBit, int numBits, uint64_t value) {
  for (int i = 0; i < numBits; ++i) {
    int pos = startBit + i;
    if (value & (1ULL << i))
      buf[pos >> 3] |=  (uint8_t)(1u << (pos & 7));
    else
      buf[pos >> 3] &= ~(uint8_t)(1u << (pos & 7));
  }
}

// -------- AIS payload builders --------

static void appendFixedText(std::vector<uint8_t>& p, const char* s, size_t width) {
  size_t n = strlen(s);
  for (size_t i = 0; i < width; ++i) p.push_back((uint8_t)((i < n) ? s[i] : ' '));
}

// PGN 129038 — AIS Class A Position Report (26 bytes, bit-packed per N2K standard)
// Field bit offsets from canboat pgns.xml:
//   0: MsgID(6)  6: Repeat(2)  8: MMSI(30)  38: NavStatus(4)
//  42: ROT(16)  58: SOG(16,0.01kts)  74: PosAcc(1)
//  75: LON(32,1e-7deg)  107: LAT(32,1e-7deg)
// 139: COG(16,1e-4rad)  155: HDG(16,1e-4rad)  171: TimeStamp(6)
static std::vector<uint8_t> buildAISClassADynamic(uint32_t mmsi, double latDeg, double lonDeg,
                                                   double cogDeg, double sogKt, double hdgDeg) {
  uint8_t buf[26] = {};
  n2kSetBits(buf,   0,  6, 1);                                              // Message ID = 1
  n2kSetBits(buf,   6,  2, 0);                                              // Repeat = 0
  n2kSetBits(buf,   8, 30, (uint64_t)mmsi);                                 // MMSI
  n2kSetBits(buf,  38,  4, 0);                                              // Nav Status = 0
  n2kSetBits(buf,  42, 16, (uint64_t)(uint16_t)0x8000);                    // ROT = N/A
  // SOG: 0.01 m/s units (cm/s) — the app decodes as m/s * 0.01, then converts to kts
  n2kSetBits(buf,  58, 16, (uint64_t)(uint16_t)std::min(65534L, lround(sogKt * KNOT_TO_MPS * 100.0)));
  n2kSetBits(buf,  74,  1, 0);                                              // Pos Accuracy = 0
  n2kSetBits(buf,  75, 32, (uint64_t)(uint32_t)(int32_t)llround(lonDeg * 1e7));
  n2kSetBits(buf, 107, 32, (uint64_t)(uint32_t)(int32_t)llround(latDeg * 1e7));
  n2kSetBits(buf, 139, 16, (uint64_t)(uint16_t)std::min(62831L, lround(degToRad(cogDeg) * 10000.0)));
  n2kSetBits(buf, 155, 16, (uint64_t)(uint16_t)std::min(62831L, lround(degToRad(hdgDeg) * 10000.0)));
  n2kSetBits(buf, 171,  6, 60);                                             // Time Stamp = N/A
  return std::vector<uint8_t>(buf, buf + 26);
}

// PGN 129039 — AIS Class B Position Report (26 bytes, bit-packed per N2K standard)
// Field bit offsets:
//   0: MsgID(6)  6: Repeat(2)  8: MMSI(30)  38: Reserved(8)
//  46: SOG(16,0.01kts)  62: PosAcc(1)
//  63: LON(32,1e-7deg)  95: LAT(32,1e-7deg)
// 127: COG(16,1e-4rad)  143: HDG(16,1e-4rad)  159: TimeStamp(6)
static std::vector<uint8_t> buildAISClassBDynamic(uint32_t mmsi, double latDeg, double lonDeg,
                                                   double cogDeg, double sogKt, double hdgDeg) {
  uint8_t buf[26] = {};
  n2kSetBits(buf,   0,  6, 18);                                             // Message ID = 18
  n2kSetBits(buf,   6,  2, 0);                                              // Repeat = 0
  n2kSetBits(buf,   8, 30, (uint64_t)mmsi);                                 // MMSI
  n2kSetBits(buf,  38,  8, 0);                                              // Reserved
  // SOG: 0.01 m/s units (cm/s)
  n2kSetBits(buf,  46, 16, (uint64_t)(uint16_t)std::min(65534L, lround(sogKt * KNOT_TO_MPS * 100.0)));
  n2kSetBits(buf,  62,  1, 0);                                              // Pos Accuracy = 0
  n2kSetBits(buf,  63, 32, (uint64_t)(uint32_t)(int32_t)llround(lonDeg * 1e7));
  n2kSetBits(buf,  95, 32, (uint64_t)(uint32_t)(int32_t)llround(latDeg * 1e7));
  n2kSetBits(buf, 127, 16, (uint64_t)(uint16_t)std::min(62831L, lround(degToRad(cogDeg) * 10000.0)));
  n2kSetBits(buf, 143, 16, (uint64_t)(uint16_t)std::min(62831L, lround(degToRad(hdgDeg) * 10000.0)));
  n2kSetBits(buf, 159,  6, 60);                                             // Time Stamp = N/A
  return std::vector<uint8_t>(buf, buf + 26);
}

static std::vector<uint8_t> buildAISClassBStaticA(uint32_t mmsi, const char* name) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00); appendU32LE(p, mmsi); appendFixedText(p, name, 20);
  return p;
}

static std::vector<uint8_t> buildAISClassBStaticB(uint32_t mmsi, uint8_t shipType) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00); appendU32LE(p, mmsi); appendU8(p, shipType);
  appendFixedText(p, "MAPTATTOO SIM", 20);
  return p;
}

static std::vector<uint8_t> buildAISClassAStaticVoyage(uint32_t mmsi, const char* name, uint8_t shipType) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00); appendU32LE(p, mmsi); appendU32LE(p, 9876543);
  appendFixedText(p, "WZZ9999", 7); appendFixedText(p, name, 20);
  appendU8(p, shipType); appendU16LE(p, 300); appendU16LE(p, 80);
  appendFixedText(p, "MONTEREY", 20);
  return p;
}

static std::vector<uint8_t> buildAtoN(uint32_t mmsi, double latDeg, double lonDeg) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00); appendU32LE(p, mmsi);
  appendI32LE(p, (int32_t)llround(latDeg * 1e7));
  appendI32LE(p, (int32_t)llround(lonDeg * 1e7));
  appendFixedText(p, "MONTEREY AtoN", 20); appendU8(p, 0x00);
  return p;
}

static std::vector<uint8_t> buildAISUtcDate(uint32_t mmsi) {
  std::vector<uint8_t> p;
  appendU8(p, 0x00); appendU32LE(p, mmsi);
  appendU16LE(p, 2026); appendU8(p, 5); appendU8(p, 15);
  appendU8(p, (uint8_t)((millis() / 3600000UL) % 24));
  appendU8(p, (uint8_t)((millis() / 60000UL)   % 60));
  appendU8(p, (uint8_t)((millis() / 1000UL)    % 60));
  appendU8(p, 0xFF);
  return p;
}

// -------- Own-vessel simulation --------

static void getOwnship(double tSec, double& lat, double& lon,
                       double& cogDeg, double& sogKt, double& headingDeg) {
  const double distM = OWNSHIP_SOG_KT * KNOT_TO_MPS * tSec;
  movePoint(OWNSHIP_START_LAT, OWNSHIP_START_LON, OWNSHIP_COG_DEG, distM, lat, lon);
  sogKt      = OWNSHIP_SOG_KT + oscillate(0.0, 0.15, 47.0, 0.0, tSec);
  cogDeg     = wrap360(OWNSHIP_COG_DEG + oscillate(0.0, 1.2, 70.0, 1.2, tSec));
  headingDeg = wrap360(cogDeg + oscillate(0.0, 2.0, 32.0, 0.5, tSec));
}

static void simulateOwnship1Hz(uint32_t nowMs) {
  static uint32_t lastMs = 0;
  if (nowMs - lastMs < 1000) return;
  lastMs = nowMs;

  const double tSec = (nowMs - gSimStartMs) / 1000.0;
  double lat, lon, cogDeg, sogKt, headingDeg;
  getOwnship(tSec, lat, lon, cogDeg, sogKt, headingDeg);

  const double waterSpeed  = sogKt + oscillate(0.0,  0.10,  19.0, 0.0, tSec);
  const double depthM      = oscillate( 52.0,  7.0,   95.0, 0.4, tSec);
  const double rollDeg     = oscillate(  0.0,  4.0,    7.5, 0.0, tSec);
  const double pitchDeg    = oscillate(  0.0,  1.0,   11.0, 0.8, tSec);
  const double windSpeedKt = oscillate( 12.0,  3.0,   80.0, 0.0, tSec);
  const double windAngleDeg = wrap360(45.0 + oscillate(0.0, 18.0, 65.0, 2.0, tSec));
  const double waterTempC  = oscillate( 13.5,  0.4,  180.0, 0.0, tSec);
  const double airTempC    = oscillate( 16.5,  1.5,  240.0, 1.1, tSec);
  const double pressurePa  = oscillate(101325.0, 180.0, 360.0, 0.2, tSec);
  const double humidityPct = oscillate( 72.0,  6.0,  210.0, 0.3, tSec);
  const double batteryV    = oscillate( 13.25, 0.08,  50.0, 0.7, tSec);
  const double batteryA    = oscillate( -2.5,  0.7,   33.0, 1.6, tSec);
  const double socPct      = oscillate( 86.0,  1.5,  600.0, 0.0, tSec);
  const double fuelPct     = oscillate( 68.0,  1.0,  400.0, 0.0, tSec);
  const double rpm         = oscillate(1850.0, 80.0,  40.0, 0.9, tSec);
  const double coolantC    = oscillate( 82.0,  2.0,   65.0, 1.4, tSec);
  const double rudderDeg   = oscillate(  0.0,  5.0,   24.0, 2.0, tSec);

  std::vector<uint8_t> p;
  p = buildPGN129025_Position(lat, lon);               notifyPGN(129025, SRC_OWNSHIP_NAV, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN129026_CogSog(cogDeg, sogKt);            notifyPGN(129026, SRC_OWNSHIP_NAV, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN126992_SystemTime(nowMs);                notifyPGN(126992, SRC_OWNSHIP_NAV, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN129029_GNSS(lat, lon, 2.0);              notifyPGN(129029, SRC_OWNSHIP_NAV, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN127250_Heading(headingDeg, 13.0);        notifyPGN(127250, SRC_OWNSHIP_NAV, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN127257_Attitude(headingDeg, pitchDeg, rollDeg); notifyPGN(127257, SRC_OWNSHIP_NAV, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN128259_WaterSpeed(waterSpeed);           notifyPGN(128259, SRC_OWNSHIP_NAV, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN128267_Depth(depthM, 0.0);               notifyPGN(128267, SRC_OWNSHIP_NAV, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN130306_Wind(windSpeedKt, windAngleDeg);  notifyPGN(130306, SRC_OWNSHIP_ENV, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN130310_Env(waterTempC, airTempC, pressurePa); notifyPGN(130310, SRC_OWNSHIP_ENV, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN130311_EnvOld(airTempC, humidityPct, pressurePa); notifyPGN(130311, SRC_OWNSHIP_ENV, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN130312_Temperature(waterTempC);          notifyPGN(130312, SRC_OWNSHIP_ENV, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN130314_Pressure(pressurePa);             notifyPGN(130314, SRC_OWNSHIP_ENV, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN130316_TempExtended(waterTempC);         notifyPGN(130316, SRC_OWNSHIP_ENV, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN127505_Fluid(fuelPct, 80.0);             notifyPGN(127505, SRC_OWNSHIP_ENV, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN127508_Battery(batteryV, batteryA, 28.0); notifyPGN(127508, SRC_OWNSHIP_ENV, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN127506_DcDetailed(batteryV, batteryA, socPct); notifyPGN(127506, SRC_OWNSHIP_ENV, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN127488_EngineRapid(rpm);                 notifyPGN(127488, SRC_OWNSHIP_ENGINE, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN127489_EngineDynamic(350000.0, coolantC, 14.1, 4.2, 123.4); notifyPGN(127489, SRC_OWNSHIP_ENGINE, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN127493_Transmission(220000.0, 58.0);     notifyPGN(127493, SRC_OWNSHIP_ENGINE, p.data(), p.size(), false); gSimFramesGenerated++;
  p = buildPGN127245_Rudder(rudderDeg);                notifyPGN(127245, SRC_OWNSHIP_NAV, p.data(), p.size(), false); gSimFramesGenerated++;

  gLastN2KRxMs = nowMs;
}

// -------- AIS target data --------

static SimAisTarget gAisTargets[15] = {
  {367100001, "MB FISHER 01",  36.7200, -121.9400,  7.5, 125.0, 125.0, 30, false},
  {367100002, "MB FISHER 02",  36.6900, -121.8800,  6.2, 285.0, 285.0, 30, false},
  {367100003, "CYPRESS POINT", 36.6300, -121.9650,  9.0,  35.0,  35.0, 70, true },
  {367100004, "OTTER BAY",     36.7500, -121.8250,  5.5, 210.0, 210.0, 37, false},
  {367100005, "MOSS LANDING",  36.8000, -121.7900, 11.0, 165.0, 165.0, 52, true },
  {367100006, "SEAL ROCK",     36.6650, -121.9200,  4.8, 310.0, 310.0, 36, false},
  {367100007, "SANTA CRUZ 7",  36.8450, -121.9100, 13.0, 145.0, 145.0, 60, true },
  {367100008, "PACIFIC STAR",  36.7350, -121.9900,  8.1,  95.0,  95.0, 70, true },
  {367100009, "KELP RIDER",    36.6150, -121.8450,  3.7,  20.0,  20.0, 36, false},
  {367100010, "LOVERS POINT",  36.6500, -121.9050,  6.8, 250.0, 250.0, 37, false},
  {367100011, "BAY PILOT",     36.7050, -121.8750, 12.5, 180.0, 180.0, 50, true },
  {367100012, "ALBACORE",      36.7750, -121.9400,  7.0, 330.0, 330.0, 30, false},
  {367100013, "RED BUOY",      36.6100, -121.8950,  0.0,   0.0,   0.0,  0, false},
  {367100014, "SURVEYOR",      36.7000, -121.9550,  4.2,  75.0,  75.0, 55, true },
  {367100015, "ANCHOR WATCH",  36.7300, -121.8600,  0.2, 270.0, 270.0, 37, false}
};

// -------- Per-target AIS schedule --------
// Last time each target's dynamic / static report was sent.
static uint32_t gAisLastDynMs[15]    = {};
static uint32_t gAisLastStaticMs[15] = {};

// Minimum gap between any two successive AIS BLE notifications.
// Prevents back-to-back notifications (two targets whose intervals expire on
// the same loop() tick) from triggering the receiver's duplicate filter.
// 25 ms >> 10 ms dedup window; well below the 2 s minimum real AIS interval.
static const uint32_t AIS_MIN_INTER_FRAME_MS = 25u;
static uint32_t       gLastAisNotifyMs       = 0;

// Realistic dynamic report interval per ITU-R M.1371 / IEC 62287.
// Class A: 2 s (>23 kt), 6 s (14-23 kt), 10 s (0-14 kt), 180 s (<2 kt)
// Class B: 15 s (>14 kt), 30 s (2-14 kt), 180 s (<2 kt)
// AtoN:    180 s (fixed aid)
static uint32_t aisDynIntervalMs(const SimAisTarget& a) {
  if (a.sogKt < 0.2) return 180000u;
  if (a.classA) {
    if (a.sogKt >= 23.0) return  2000u;
    if (a.sogKt >= 14.0) return  6000u;
    if (a.sogKt >=  2.0) return 10000u;
    return 180000u;
  } else {
    if (a.sogKt >= 14.0) return 15000u;
    if (a.sogKt >=  2.0) return 30000u;
    return 180000u;
  }
}

// Static/voyage data every 6 minutes for all vessel types.
static const uint32_t AIS_STATIC_INTERVAL_MS = 360000u;

// Spread initial transmissions so all 15 targets don't burst at once on first connect.
static void initAisSchedule() {
  for (uint8_t i = 0; i < 15; ++i) {
    const uint32_t dynInterval = aisDynIntervalMs(gAisTargets[i]);
    gAisLastDynMs[i]    = millis() - (uint32_t)((uint64_t)dynInterval          * i / 15);
    gAisLastStaticMs[i] = millis() - (uint32_t)((uint64_t)AIS_STATIC_INTERVAL_MS * i / 15);
  }
}

static void simulateAisRealistic(uint32_t nowMs) {
  const double tSec = (nowMs - gSimStartMs) / 1000.0;

  for (uint8_t i = 0; i < 15; ++i) {
    SimAisTarget& a = gAisTargets[i];

    // ---- AtoN (target 12): every 3 minutes ----
    if (i == 12) {
      if (nowMs - gAisLastDynMs[i] >= 180000u) {
        if (nowMs - gLastAisNotifyMs >= AIS_MIN_INTER_FRAME_MS) {
          gAisLastDynMs[i] = nowMs;
          gLastAisNotifyMs = nowMs;
          std::vector<uint8_t> p = buildAtoN(a.mmsi, a.lat0, a.lon0);
          notifyPGN(129041, SRC_AIS_RECEIVER, p.data(), p.size(), true);
          gSimFramesGenerated++;
          gLastN2KRxMs = nowMs;
        }
      }
      continue;
    }

    // ---- Dynamic position report ----
    const uint32_t dynInterval = aisDynIntervalMs(a);
    if (nowMs - gAisLastDynMs[i] >= dynInterval) {
      // Inter-frame pacing: don't send if another AIS notification went out
      // less than AIS_MIN_INTER_FRAME_MS ago. Leave gAisLastDynMs[i] unchanged
      // so this target is retried on the next loop() iteration.
      if (nowMs - gLastAisNotifyMs < AIS_MIN_INTER_FRAME_MS) {
        // skip this target this tick — will be picked up next loop
      } else {
        gAisLastDynMs[i] = nowMs;
        gLastAisNotifyMs = nowMs;

        double lat = a.lat0, lon = a.lon0;
        double cog = wrap360(a.cogDeg + oscillate(0.0, 3.0, 75.0 + i * 3.0, (double)i, tSec));
        double sog = a.sogKt + oscillate(0.0, 0.4, 50.0 + i * 2.0, i * 0.7, tSec);
        if (sog < 0.0) sog = 0.0;
        if (sog > 0.3) {
          const double distM = sog * KNOT_TO_MPS * tSec;
          movePoint(a.lat0, a.lon0, cog, distM, lat, lon);
        }
        const double hdg    = wrap360(cog + oscillate(0.0, 2.0, 21.0, (double)i, tSec));
        std::vector<uint8_t> p;
        if (a.classA) {
          p = buildAISClassADynamic(a.mmsi, lat, lon, cog, sog, hdg);
          notifyPGN(129038, SRC_AIS_RECEIVER, p.data(), p.size(), true);
        } else {
          p = buildAISClassBDynamic(a.mmsi, lat, lon, cog, sog, hdg);
          notifyPGN(129039, SRC_AIS_RECEIVER, p.data(), p.size(), true);
        }
        gSimFramesGenerated++;

        if (!a.classA && (i % 4 == 0)) {
          p = buildAISClassBDynamic(a.mmsi, lat, lon, cog, sog, cog);
          notifyPGN(129040, SRC_AIS_RECEIVER, p.data(), p.size(), true);
          gSimFramesGenerated++;
        }
        gLastN2KRxMs = nowMs;
      }
    }

    // ---- Static / voyage data (every 6 minutes) ----
    if (nowMs - gAisLastStaticMs[i] >= AIS_STATIC_INTERVAL_MS) {
      if (nowMs - gLastAisNotifyMs < AIS_MIN_INTER_FRAME_MS) {
        // skip this tick — will be picked up next loop
      } else {
        gAisLastStaticMs[i] = nowMs;
        gLastAisNotifyMs = nowMs;
        std::vector<uint8_t> p;
        if (a.classA) {
          p = buildAISClassAStaticVoyage(a.mmsi, a.name, a.shipType);
          notifyPGN(129794, SRC_AIS_RECEIVER, p.data(), p.size(), true);
          gSimFramesGenerated++;
        } else {
          p = buildAISClassBStaticA(a.mmsi, a.name);
          notifyPGN(129809, SRC_AIS_RECEIVER, p.data(), p.size(), true);
          gSimFramesGenerated++;
          p = buildAISClassBStaticB(a.mmsi, a.shipType);
          notifyPGN(129810, SRC_AIS_RECEIVER, p.data(), p.size(), true);
          gSimFramesGenerated++;
        }
        if (i % 5 == 0) {
          p = buildAISUtcDate(a.mmsi);
          notifyPGN(129793, SRC_AIS_RECEIVER, p.data(), p.size(), true);
          gSimFramesGenerated++;
        }
        gLastN2KRxMs = nowMs;
      }
    }
  }
}

// -------- RSSI pseudo-PGN --------

static void notifyRSSI_1Hz(uint32_t nowMs) {
  static uint32_t lastRSSINotify = 0;
  if (!gSrv || gSrv->getConnectedCount() == 0) return;
  if (isConnectPaused(nowMs)) return;
  if (nowMs - lastRSSINotify < 1000) return;
  lastRSSINotify = nowMs;
  int8_t rssi = getClientRSSI();
  if (rssi == -128) return;
  auto it = gCharByPGN.find(65281);
  if (it != gCharByPGN.end() && it->second && hasSubscribers(65281)) {
    uint8_t rssiData[2] = { SRC_GATEWAY, (uint8_t)(int8_t)rssi };
    it->second->setValue(rssiData, 2);
    bool ok = it->second->notify();
    if (ok) gBleNotifySent++;
    else { gBleNotifyFailed++; Serial.println("RSSI notify failed"); }
  }
}

// -------- Arduino entry points --------

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  ledcAttach((uint8_t)LED_EXT_RED,   LED_PWM_FREQ, LED_PWM_RES);
  ledcAttach((uint8_t)LED_EXT_GREEN, LED_PWM_FREQ, LED_PWM_RES);
  ledcAttach((uint8_t)LED_EXT_BLUE,  LED_PWM_FREQ, LED_PWM_RES);
  externalLedOff();
  setExternalLed(true, false, false);

  Serial.begin(115200);
  delay(150);
  Serial.println();
  Serial.println("BOOT - MAPTATTOO BLE Gateway Simulator");
  Serial.printf("FW_VERSION=%s\n", FW_VERSION);
  Serial.printf("LED_BRIGHTNESS=%.2f\n", LED_BRIGHTNESS);

  selectExternalAntenna();
  Serial.println("External antenna selected.");

  setupBLE();

  gSimStartMs   = millis();
  gLastN2KRxMs  = millis();
  initAisSchedule();

  Serial.println("Simulator running. No TWAI/CAN/N2K ingestion is used.");
  Serial.println("Ownship: ~5 NM north of Monterey Harbor, moving north at 5 kt.");
  Serial.println("AIS: 15 simulated Monterey Bay targets at realistic ITU-R M.1371 rates.");
}

void loop() {
  uint32_t now = millis();

  printConnectedRSSI_1Hz(now);
  printSummary(now);
  printHeap_60s(now);
  notifyRSSI_1Hz(now);

  // Only generate and push data when at least one client is connected.
  // No point building payloads or attempting notifies when nobody is listening.
  if (isBleConnected()) {
    simulateOwnship1Hz(now);
    simulateAisRealistic(now);
  }

  updateStatusLed(now);

  static uint32_t lastAdvChk = 0;
  if (now - lastAdvChk > 3000) {
    lastAdvChk = now;
    if (gSrv && gAdv && gSrv->getConnectedCount() == 0 && !gAdv->isAdvertising()) {
      Serial.println("ADV watchdog: restarting advertising");
      NimBLEDevice::startAdvertising();
    }
  }
}