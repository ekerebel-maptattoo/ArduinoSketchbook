

// MAPTATTOO — N2K → BLE (per-PGN characteristics)
// Option A: prepend N2K source address (SRC) to every BLE notification payload.
// IMPORTANT: throttle is per-(PGN,SRC) so multiple sensors on same PGN don't suppress each other.
// Board: Seeed XIAO ESP32-C6 (TWAI: TX=D6=GPIO16, RX=D7=GPIO17)
//
// Stability changes applied:
// - Reduced high-volume serial logging defaults
// - Added BLE notify failure counter
// - Added heap diagnostics every 60 seconds
// - Replaced dynamic BLE notification framing vector with fixed stack buffer
// - AIS is throttled to max 10 Hz per PGN/SRC instead of fully unthrottled
// - notify() return value is checked
// - Subscription state is cleared on disconnect
// - Buzzer support retained

#include <Arduino.h>
#include <driver/twai.h>
#include <NimBLEDevice.h>

#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <cstring>

#include <esp_system.h>
#include <esp_heap_caps.h>

extern "C" {
  #include "nimble/nimble/host/include/host/ble_gap.h"
}

// -------- Tuning / Logging --------
static const bool LOG_FP_FRAMES        = false; // deep fast-packet debug (very chatty)
static const bool LOG_CAN_RX           = false; // raw incoming CAN frames; keep false for long-running stability tests
static const bool LOG_PGN_DECODE       = false; // decoded PGN/src/dst; keep false for long-running stability tests
static const bool LOG_NOTIFY_FLOW      = false; // why notify does / does not happen; keep false for long-running stability tests
static const bool LOG_SUBSCRIPTION     = true;  // BLE subscribe state
static const bool LOG_TWAI_STATUS_1HZ  = true;  // TWAI status once per second
static const bool LOG_UNKNOWN_PGN      = false; // log unmapped PGNs in notify path
static const uint32_t LOG_SUMMARY_MS   = 5000;  // summary interval

// -------- Pins --------
#define CAN_TX GPIO_NUM_16
#define CAN_RX GPIO_NUM_17

static const uint8_t BUZZER_PIN = 20;

// External RGB LED (Dialight 5218559F = common-anode)
static const gpio_num_t LED_EXT_GREEN = GPIO_NUM_0;
static const gpio_num_t LED_EXT_BLUE  = GPIO_NUM_1;
static const gpio_num_t LED_EXT_RED   = GPIO_NUM_2;

// Consider N2K "alive" if a packet was received within this timeout
static const uint32_t N2K_ACTIVE_TIMEOUT_MS = 15000;

// -------- LED brightness / PWM --------
// 1.00 = full brightness
// 0.25 = quarter brightness
static const float LED_BRIGHTNESS = 0.1f;

// ESP32 core on your setup uses pin-based LEDC API: ledcAttach(pin, freq, res), ledcWrite(pin, duty)
static const uint32_t LED_PWM_FREQ = 5000;
static const uint8_t  LED_PWM_RES  = 8;   // 8-bit => 0..255

// -------- Firmware identity --------
static const char* FW_VERSION = "1.0.2";

// -------- BLE --------
static std::string gBleName;  // built at runtime
static const std::string SVC_UUID = "8f000000-1a12-4a86-9e26-9f1c9f0e1234";

static NimBLEServer*       gSrv = nullptr;
static NimBLEService*      gSvc = nullptr;
static NimBLEAdvertising*  gAdv = nullptr;
static std::map<uint32_t, NimBLECharacteristic*> gCharByPGN;

// Track whether ANY client subscribed to a given PGN characteristic.
// Note: with multiple clients this is simplified state, but good enough for debug.
static std::map<uint32_t, bool> gSubscribedByPGN;

// ---- Multi-client policy ----
static const uint8_t MAX_CLIENTS = 3;

// For pseudo-PGNs that originate from the gateway (not from N2K bus)
static const uint8_t SRC_GATEWAY = 0xFF;

// -------- Debug counters --------
static uint32_t gCanFramesRx        = 0;
static uint32_t gCanExtendedRx      = 0;
static uint32_t gBleNotifyAttempts  = 0;
static uint32_t gBleNotifySent      = 0;
static uint32_t gBleNotifyFailed    = 0;
static uint32_t gBleNotifyNoChar    = 0;
static uint32_t gBleNotifyNoSub     = 0;
static uint32_t gBleNotifyThrottled = 0;
static uint32_t gFastPacketComplete = 0;
static uint32_t gFastPacketDropped  = 0;

// -------- LED / state tracking --------
static uint32_t gLastN2KRxMs = 0;   // last time any N2K extended packet was received

// Disconnect reasons
static const char* discReasonStr(int r) {
  switch (r) {
    case 0x08: return "Connection Timeout (often supervision timeout / RF loss)";
    case 0x13: return "Remote User Terminated (peer explicitly disconnected)";
    case 0x16: return "Local Host Terminated (gateway side explicitly disconnected)";
    case 0x3E: return "Connection Failed to be Established";
    default:   return "Other/Unknown";
  }
}

// --- Hex dump helper ---
static void hexDump(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    Serial.printf("%02X", data[i]);
    if (i + 1 < len) Serial.print(" ");
  }
}

// --- PWM brightness helpers ---
static float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

// --- External RGB LED helpers ---
// Dialight 5218559F is COMMON-ANODE:
// common pin -> 3.3V
// pin LOW    -> color ON
// pin HIGH   -> color OFF
//
// With PWM on a common-anode LED:
//  - OFF = 100% duty HIGH
//  - ON  = partially low, depending on brightness setting
static void setExternalLed(bool redOn, bool greenOn, bool blueOn) {
  const float b = clamp01(LED_BRIGHTNESS);
  const uint32_t maxDuty = (1u << LED_PWM_RES) - 1u;   // 255 for 8-bit
  const uint32_t dutyOff = maxDuty;                    // common-anode OFF = always HIGH
  const uint32_t dutyOn  = (uint32_t)lroundf((1.0f - b) * (float)maxDuty);

  ledcWrite((uint8_t)LED_EXT_RED,   redOn   ? dutyOn : dutyOff);
  ledcWrite((uint8_t)LED_EXT_GREEN, greenOn ? dutyOn : dutyOff);
  ledcWrite((uint8_t)LED_EXT_BLUE,  blueOn  ? dutyOn : dutyOff);
}

static void externalLedOff() {
  setExternalLed(false, false, false);
}

static bool isBleConnected() {
  return (gSrv && gSrv->getConnectedCount() > 0);
}

static bool isN2KActive(uint32_t nowMs) {
  return (gLastN2KRxMs != 0) && ((nowMs - gLastN2KRxMs) <= N2K_ACTIVE_TIMEOUT_MS);
}

static void updateStatusLed(uint32_t nowMs) {
  static bool blinkPhase = false;
  static uint32_t lastBlinkToggleMs = 0;

  const bool bleConnected = isBleConnected();
  const bool n2kActive    = isN2KActive(nowMs);

  // BLE connected, but no recent N2K -> blink blue 0.5s on / 0.5s off
  if (bleConnected && !n2kActive) {
    if (nowMs - lastBlinkToggleMs >= 500) {
      lastBlinkToggleMs = nowMs;
      blinkPhase = !blinkPhase;
    }
    setExternalLed(false, false, blinkPhase);
    return;
  }

  // BLE connected and N2K active -> solid blue
  if (bleConnected && n2kActive) {
    setExternalLed(false, false, true);
    return;
  }

  // No BLE, but N2K active -> solid green
  if (!bleConnected && n2kActive) {
    setExternalLed(false, true, false);
    return;
  }

  // Default powered state -> solid red
  setExternalLed(true, false, false);
}

// --- XIAO ESP32-C6 RF switch: select external antenna ---
// Per Seeed: GPIO3 LOW enables RF switch control; GPIO14 HIGH selects external antenna.
static void selectExternalAntenna() {
  pinMode(GPIO_NUM_3, OUTPUT);
  digitalWrite(GPIO_NUM_3, LOW);     // enable RF switch control

  pinMode(GPIO_NUM_14, OUTPUT);
  digitalWrite(GPIO_NUM_14, HIGH);   // HIGH = external antenna (LOW = onboard ceramic)

  delay(10); // small settle time
}

// --- RSSI measurement function ---
// Returns min RSSI across connected peers; -128 when none/unknown.
static int8_t getClientRSSI() {
  if (!gSrv) return -128;

  const std::vector<uint16_t> peers = gSrv->getPeerDevices();
  if (peers.empty()) return -128;

  bool have = false;
  int8_t rssi_min = 127;

  for (uint16_t h : peers) {
    int8_t rssi = 0;
    const int rc = ble_gap_conn_rssi(h, &rssi);
    if (rc == 0 && rssi != 127) {
      if (!have || rssi < rssi_min) rssi_min = rssi;
      have = true;
    }
  }

  if (!have) return -128;
  return rssi_min;
}

// Print RSSI once per second if connected
static void printConnectedRSSI_1Hz(uint32_t nowMs) {
  static uint32_t lastPrintMs = 0;
  if (nowMs - lastPrintMs < 1000) return;
  lastPrintMs = nowMs;

  if (!gSrv || gSrv->getConnectedCount() == 0) return;

  const int8_t rssi = getClientRSSI();
  if (rssi == -128) return;

  Serial.printf("RSSI (connected) = %d dBm\n", (int)rssi);
}

// --- TWAI status logging ---
static void printTwaiStatus_1Hz(uint32_t nowMs) {
  if (!LOG_TWAI_STATUS_1HZ) return;

  static uint32_t lastTwaiStatusMs = 0;
  if (nowMs - lastTwaiStatusMs < 1000) return;
  lastTwaiStatusMs = nowMs;

  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK) {
    Serial.printf(
      "TWAI status: state=%d rxq=%lu txq=%lu rx_missed=%lu rx_overrun=%lu bus_err=%lu arb_lost=%lu\n",
      (int)st.state,
      (unsigned long)st.msgs_to_rx,
      (unsigned long)st.msgs_to_tx,
      (unsigned long)st.rx_missed_count,
      (unsigned long)st.rx_overrun_count,
      (unsigned long)st.bus_error_count,
      (unsigned long)st.arb_lost_count
    );
  } else {
    Serial.println("TWAI status read failed");
  }
}

// --- Summary logging ---
static void printSummary(uint32_t nowMs) {
  static uint32_t lastSummaryMs = 0;
  if (nowMs - lastSummaryMs < LOG_SUMMARY_MS) return;
  lastSummaryMs = nowMs;

  Serial.printf(
    "SUMMARY canRx=%lu extRx=%lu notifyAttempts=%lu sent=%lu failed=%lu noChar=%lu noSub=%lu throttled=%lu fpComplete=%lu fpDropped=%lu connected=%u adv=%u n2kActive=%u\n",
    (unsigned long)gCanFramesRx,
    (unsigned long)gCanExtendedRx,
    (unsigned long)gBleNotifyAttempts,
    (unsigned long)gBleNotifySent,
    (unsigned long)gBleNotifyFailed,
    (unsigned long)gBleNotifyNoChar,
    (unsigned long)gBleNotifyNoSub,
    (unsigned long)gBleNotifyThrottled,
    (unsigned long)gFastPacketComplete,
    (unsigned long)gFastPacketDropped,
    gSrv ? (unsigned)gSrv->getConnectedCount() : 0,
    gAdv ? (unsigned)gAdv->isAdvertising() : 0,
    (unsigned)isN2KActive(nowMs)
  );
}

// --- Heap diagnostics ---
static void printHeap_60s(uint32_t nowMs) {
  static uint32_t lastHeapPrintMs = 0;
  if (nowMs - lastHeapPrintMs < 60000) return;
  lastHeapPrintMs = nowMs;

  Serial.printf("HEAP free=%u minFree=%u largest=%u\n",
                (unsigned)ESP.getFreeHeap(),
                (unsigned)ESP.getMinFreeHeap(),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}

static void beepOnBleConnect() {
  tone(BUZZER_PIN, 800, 250);   // pin, frequency Hz, duration ms
  delay(250);
  noTone(BUZZER_PIN);
}

// --- Server callbacks: keep adv on connect; optionally stop at MAX_CLIENTS; restart on disconnect ---
class SrvCB : public NimBLEServerCallbacks {
public:
  void onConnect(NimBLEServer* s, NimBLEConnInfo& ci) override {
    Serial.printf("BLE CONNECT from %s handle=%u (count=%u)\n",
                  ci.getAddress().toString().c_str(),
                  (unsigned)ci.getConnHandle(),
                  (unsigned)s->getConnectedCount());

    beepOnBleConnect();

    // Connection params: min/max interval are in 1.25ms units, timeout in 10ms units.
    s->updateConnParams(ci.getConnHandle(),
                        /*minInterval*/40,   /*=50ms*/
                        /*maxInterval*/120,  /*=150ms*/
                        /*latency*/0,
                        /*timeout*/1200);    /*=12s*/

    // Allow multiple clients: keep advertising.
    if (s->getConnectedCount() >= MAX_CLIENTS) {
      Serial.printf("Reached MAX_CLIENTS=%u -> stop advertising\n", (unsigned)MAX_CLIENTS);
      NimBLEDevice::stopAdvertising();
    } else {
      NimBLEDevice::startAdvertising();
    }
  }

  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& ci, int reason) override {
    Serial.printf("BLE DISCONNECT from %s handle=%u reason=0x%02X (%s) (count=%u)\n",
                  ci.getAddress().toString().c_str(),
                  (unsigned)ci.getConnHandle(),
                  reason,
                  discReasonStr(reason),
                  (unsigned)s->getConnectedCount());

    // Clear simplified global subscription state so stale subscriptions do not survive disconnects.
    // This is intentionally conservative for single-client testing and reconnect reliability.
    for (auto &kv : gSubscribedByPGN) {
      kv.second = false;
    }

    if (s->getConnectedCount() < MAX_CLIENTS) {
      delay(80);
      NimBLEDevice::startAdvertising();
    }
  }
};

// ---------- Device identity helpers ----------
static std::string chipIdString() {
  uint64_t mac = ESP.getEfuseMac();
  char buf[32];
  snprintf(buf, sizeof(buf), "%02X%02X%02X%02X%02X%02X",
           (uint8_t)(mac >> 40), (uint8_t)(mac >> 32),
           (uint8_t)(mac >> 24), (uint8_t)(mac >> 16),
           (uint8_t)(mac >>  8), (uint8_t)(mac >>  0));
  return std::string(buf);
}

// ---------- Subscription tracking callbacks ----------
class CharCB : public NimBLECharacteristicCallbacks {
public:
  explicit CharCB(uint32_t pgn) : pgn_(pgn) {}

  void onSubscribe(NimBLECharacteristic* /*c*/, NimBLEConnInfo& ci, uint16_t subValue) override {
    const bool subscribed = (subValue != 0);
    gSubscribedByPGN[pgn_] = subscribed;

    if (LOG_SUBSCRIPTION) {
      Serial.printf("SUBSCRIBE PGN %u from %s handle=%u -> %s (subValue=0x%04X)\n",
                    (unsigned)pgn_,
                    ci.getAddress().toString().c_str(),
                    (unsigned)ci.getConnHandle(),
                    subscribed ? "ON" : "OFF",
                    (unsigned)subValue);
    }
  }

private:
  uint32_t pgn_;
};

static inline bool hasSubscribers(uint32_t pgn) {
  auto it = gSubscribedByPGN.find(pgn);
  return (it != gSubscribedByPGN.end() && it->second);
}

// ---------- UUID helpers ----------
static std::string makeCharUUID(uint32_t pgn) {
  // NOTE: uses low 16 bits of PGN. We validate uniqueness for the configured PGNs at runtime.
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
    const uint16_t lo = (uint16_t)(pgn & 0xFFFF);

    auto it = seen.find(lo);
    if (it != seen.end() && it->second != pgn) {
      ok = false;
      Serial.printf("ERROR: UUID collision: PGN %u and PGN %u share low16=0x%04X\n",
                    (unsigned)it->second, (unsigned)pgn, (unsigned)lo);
    } else {
      seen[lo] = pgn;
    }
  }

  if (!ok) {
    Serial.println("ERROR: PGN UUID collisions detected. Fix makeCharUUID() mapping before shipping.");
    while (1) {
      setExternalLed(true, false, false);
      delay(200);
    }
  }
}

// -------- Setup BLE --------
static void setupBLE() {
  gBleName = std::string("MAPTATTOO N2K");

  NimBLEDevice::init(gBleName);
  NimBLEDevice::setDeviceName(gBleName);
  NimBLEDevice::setPower(ESP_PWR_LVL_P3);   // ~+3 dBm
  NimBLEDevice::setMTU(247);
  NimBLEDevice::setSecurityAuth(false, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  gSrv = NimBLEDevice::createServer();
  gSrv->setCallbacks(new SrvCB());

  // Standard Device Information Service
  NimBLEService* dis = gSrv->createService(NimBLEUUID((uint16_t)0x180A));
  if (dis) {
    auto* manu   = dis->createCharacteristic(NimBLEUUID((uint16_t)0x2A29), NIMBLE_PROPERTY::READ);
    auto* model  = dis->createCharacteristic(NimBLEUUID((uint16_t)0x2A24), NIMBLE_PROPERTY::READ);
    auto* serial = dis->createCharacteristic(NimBLEUUID((uint16_t)0x2A25), NIMBLE_PROPERTY::READ);
    auto* fw     = dis->createCharacteristic(NimBLEUUID((uint16_t)0x2A26), NIMBLE_PROPERTY::READ);

    if (manu)   manu->setValue("MAPTATTOO");
    if (model)  model->setValue("N2K Gateway (XIAO ESP32-C6)");
    if (serial) serial->setValue(chipIdString());
    if (fw)     fw->setValue(std::string(FW_VERSION));

    dis->start();
  }

  gSvc = gSrv->createService(NimBLEUUID(SVC_UUID));

  // RSSI Status pseudo-PGN
  gCharByPGN[65281] = addNotifyChar(65281, "PGN 65281 RSSI Status");

  // Core nav/time
  gCharByPGN[129025] = addNotifyChar(129025, "PGN 129025 Position");
  gCharByPGN[129026] = addNotifyChar(129026, "PGN 129026 COG/SOG");
  gCharByPGN[126992] = addNotifyChar(126992, "PGN 126992 System Time");
  gCharByPGN[129029] = addNotifyChar(129029, "PGN 129029 GNSS");

  // ENV / speed / depth / heading / attitude / wind / fluids / battery
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

  // Engine / transmission
  gCharByPGN[127488] = addNotifyChar(127488, "PGN 127488 Engine Rapid");
  gCharByPGN[127489] = addNotifyChar(127489, "PGN 127489 Engine Dynamic");
  gCharByPGN[127493] = addNotifyChar(127493, "PGN 127493 Transmission Dyn");
  gCharByPGN[127245] = addNotifyChar(127245, "PGN 127245 Rudder");

  // AIS (dyn + static)
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
  advData.setFlags(0x06); // LE General Discoverable + BR/EDR not supported
  advData.addServiceUUID(NimBLEUUID(SVC_UUID));

  gAdv->setAdvertisementData(advData);
  gAdv->setMinInterval(0x00A0);
  gAdv->setMaxInterval(0x0140);
  gAdv->start();

  Serial.printf("BLE up, advertising as %s.\n", gBleName.c_str());
}

// -------- Helpers --------
static inline void decodeId(uint32_t id, uint8_t& prio, uint32_t& pgn, uint8_t& src, int16_t& dst) {
  prio = (id >> 26) & 0x7;
  uint8_t dp = (id >> 24) & 0x1;
  uint8_t pf = (id >> 16) & 0xFF;
  uint8_t ps = (id >> 8)  & 0xFF;
  src = id & 0xFF;
  if (pf < 240) { pgn = ((uint32_t)dp << 16) | ((uint32_t)pf << 8); dst = ps; }
  else          { pgn = ((uint32_t)dp << 16) | ((uint32_t)pf << 8) | ps; dst = -1; }
}

static inline bool isAIS(uint32_t pgn){
  switch (pgn){
    case 129038: case 129039: case 129040: case 129041:
    case 129793: case 129794: case 129809: case 129810:
      return true;
    default: return false;
  }
}

static inline bool isLikelyFast(uint32_t pgn){
  switch (pgn){
    case 129029: case 127489: case 127493:
    case 129038: case 129039: case 129040: case 129041:
    case 129793: case 129794: case 129809: case 129810:
      return true;
    default: return false;
  }
}

// -------- Fast-packet reassembly --------
struct FPBuf {
  bool started = false;
  uint32_t t0 = 0;
  uint8_t seq = 0;
  uint8_t nextIdx = 0;
  uint16_t total = 0;
  std::vector<uint8_t> data;
};

static std::map<uint64_t, FPBuf> fpmap;

static inline bool fpIsStale(uint32_t t0){ return (millis() - t0) > 500; }

static inline uint64_t fpKey(uint32_t pgn, uint8_t src, uint8_t seq){
  return ( (uint64_t)pgn << 16 ) | ((uint64_t)src << 8) | (uint64_t)(seq & 0x1F);
}

static bool fp_push(uint32_t pgn, uint8_t src, const uint8_t* d, uint8_t dlc, std::vector<uint8_t>& out){
  if (dlc < 2) return false;

  uint8_t hdr = d[0];
  uint8_t seq = (hdr >> 5) & 0x07;
  uint8_t idx = hdr & 0x1F;

  uint64_t key = fpKey(pgn, src, seq);
  FPBuf &b = fpmap[key];

  if (LOG_FP_FRAMES) {
    Serial.printf("FP RX PGN=%u SRC=%u seq=%u idx=%u dlc=%u data=",
                  (unsigned)pgn, (unsigned)src, (unsigned)seq, (unsigned)idx, (unsigned)dlc);
    hexDump(d, dlc);
    Serial.println();
  }

  if (idx == 0) { // START
    b.started = true;
    b.t0 = millis();
    b.seq = seq;
    b.nextIdx = 1;
    b.data.clear();
    b.total = d[1];

    for (int i = 2; i < dlc && (int)b.data.size() < b.total; i++) {
      b.data.push_back(d[i]);
    }

    if (LOG_FP_FRAMES) {
      Serial.printf("FP START PGN=%u SRC=%u seq=%u total=%u have=%u\n",
                    (unsigned)pgn, (unsigned)src, (unsigned)seq,
                    (unsigned)b.total, (unsigned)b.data.size());
    }

    if ((int)b.data.size() >= b.total) {
      out = b.data;
      fpmap.erase(key);
      gFastPacketComplete++;
      if (LOG_FP_FRAMES) {
        Serial.printf("FP COMPLETE (single start frame) PGN=%u SRC=%u len=%u\n",
                      (unsigned)pgn, (unsigned)src, (unsigned)out.size());
      }
      return true;
    }
    return false;
  } else { // CONT
    if (!b.started) {
      gFastPacketDropped++;
      if (LOG_FP_FRAMES) {
        Serial.printf("FP DROP no-start PGN=%u SRC=%u seq=%u idx=%u\n",
                      (unsigned)pgn, (unsigned)src, (unsigned)seq, (unsigned)idx);
      }
      return false;
    }

    if (b.seq != seq) {
      gFastPacketDropped++;
      if (LOG_FP_FRAMES) {
        Serial.printf("FP DROP seq mismatch PGN=%u SRC=%u expected=%u got=%u\n",
                      (unsigned)pgn, (unsigned)src, (unsigned)b.seq, (unsigned)seq);
      }
      fpmap.erase(key);
      return false;
    }

    if (fpIsStale(b.t0)) {
      gFastPacketDropped++;
      if (LOG_FP_FRAMES) {
        Serial.printf("FP DROP stale PGN=%u SRC=%u seq=%u\n",
                      (unsigned)pgn, (unsigned)src, (unsigned)seq);
      }
      fpmap.erase(key);
      return false;
    }

    if (b.nextIdx != idx) {
      gFastPacketDropped++;
      if (LOG_FP_FRAMES) {
        Serial.printf("FP DROP idx mismatch PGN=%u SRC=%u expected=%u got=%u\n",
                      (unsigned)pgn, (unsigned)src, (unsigned)b.nextIdx, (unsigned)idx);
      }
      fpmap.erase(key);
      return false;
    }

    for (int i = 1; i < dlc && (int)b.data.size() < b.total; i++) {
      b.data.push_back(d[i]);
    }
    b.nextIdx++;

    if (LOG_FP_FRAMES) {
      Serial.printf("FP CONT PGN=%u SRC=%u seq=%u have=%u/%u nextIdx=%u\n",
                    (unsigned)pgn, (unsigned)src, (unsigned)seq,
                    (unsigned)b.data.size(), (unsigned)b.total, (unsigned)b.nextIdx);
    }

    if ((int)b.data.size() >= b.total) {
      out = b.data;
      fpmap.erase(key);
      gFastPacketComplete++;
      if (LOG_FP_FRAMES) {
        Serial.printf("FP COMPLETE PGN=%u SRC=%u len=%u payload=",
                      (unsigned)pgn, (unsigned)src, (unsigned)out.size());
        hexDump(out.data(), out.size());
        Serial.println();
      }
      return true;
    }
    return false;
  }
}

// -------- Throttle, per (PGN,SRC) --------
static std::map<uint64_t, uint32_t> lastTxMs;

static inline uint64_t pgnSrcKey(uint32_t pgn, uint8_t src) {
  return ((uint64_t)pgn << 8) | (uint64_t)src;
}

// Option A framing: first byte is SRC, followed by original PGN payload bytes
static void notifyPGN(uint32_t pgn, uint8_t src, const uint8_t* payload, size_t len, bool is_ais) {
  gBleNotifyAttempts++;

  auto it = gCharByPGN.find(pgn);
  if (it == gCharByPGN.end() || it->second == nullptr) {
    gBleNotifyNoChar++;
    if (LOG_NOTIFY_FLOW && LOG_UNKNOWN_PGN) {
      Serial.printf("NOTIFY SKIP PGN=%u SRC=%u len=%u reason=no characteristic\n",
                    (unsigned)pgn, (unsigned)src, (unsigned)len);
    }
    return;
  }

  if (!hasSubscribers(pgn)) {
    gBleNotifyNoSub++;
    if (LOG_NOTIFY_FLOW) {
      Serial.printf("NOTIFY SKIP PGN=%u SRC=%u len=%u reason=no subscribers\n",
                    (unsigned)pgn, (unsigned)src, (unsigned)len);
    }
    return;
  }

  uint32_t now = millis();

  // Non-AIS: 1 Hz per PGN/SRC.
  // AIS: max 10 Hz per PGN/SRC to avoid BLE notify queue pressure during AIS-heavy traffic.
  const uint32_t minIntervalMs = is_ais ? 100 : 1000;

  uint64_t k = pgnSrcKey(pgn, src);
  uint32_t &lt = lastTxMs[k];
  if (now - lt < minIntervalMs) {
    gBleNotifyThrottled++;
    if (LOG_NOTIFY_FLOW) {
      Serial.printf("NOTIFY SKIP PGN=%u SRC=%u len=%u reason=throttled dt=%lu ms\n",
                    (unsigned)pgn, (unsigned)src, (unsigned)len,
                    (unsigned long)(now - lt));
    }
    return;
  }
  lt = now;

  // Fixed stack buffer avoids repeated heap allocations in the BLE hot path.
  // N2K fast-packet payloads are normally <= 223 bytes; +1 for SRC.
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
  if (ok) {
    gBleNotifySent++;
  } else {
    gBleNotifyFailed++;
    Serial.printf("BLE notify failed PGN=%u SRC=%u len=%u\n",
                  (unsigned)pgn, (unsigned)src, (unsigned)(len + 1));
  }
}

// -------- Setup / Loop --------
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  // PWM setup for external common-anode RGB LED
  ledcAttach((uint8_t)LED_EXT_RED,   LED_PWM_FREQ, LED_PWM_RES);
  ledcAttach((uint8_t)LED_EXT_GREEN, LED_PWM_FREQ, LED_PWM_RES);
  ledcAttach((uint8_t)LED_EXT_BLUE,  LED_PWM_FREQ, LED_PWM_RES);

  // Start with all colors off, then show RED immediately at power-up
  externalLedOff();
  setExternalLed(true, false, false);

  Serial.begin(115200);
  delay(150);
  Serial.println();
  Serial.println("BOOT");
  Serial.printf("FW_VERSION=%s\n", FW_VERSION);
  Serial.printf("LED_BRIGHTNESS=%.2f\n", LED_BRIGHTNESS);

  selectExternalAntenna();
  Serial.println("External antenna selected.");

  setupBLE();

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  g.tx_queue_len = 0;
  g.rx_queue_len = 256;

  twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) {
    Serial.println("TWAI install failed");
    while (1) {
      setExternalLed(true, false, false);
      delay(200);
    }
  }

  if (twai_start() != ESP_OK) {
    Serial.println("TWAI start failed");
    while (1) {
      setExternalLed(true, false, false);
      delay(200);
    }
  }

  Serial.println("TWAI up @ 250k. N2K→BLE forwarding running. RSSI printed 1 Hz when connected.");
}

void loop() {
  uint32_t now = millis();

  // Periodic health logs
  printConnectedRSSI_1Hz(now);
  printTwaiStatus_1Hz(now);
  printSummary(now);
  printHeap_60s(now);

  // Notify RSSI characteristic every second, silently
  static uint32_t lastRSSINotify = 0;
  if (gSrv && gSrv->getConnectedCount() > 0 && now - lastRSSINotify >= 1000) {
    lastRSSINotify = now;
    int8_t rssi = getClientRSSI();
    if (rssi != -128) {
      auto it = gCharByPGN.find(65281);
      if (it != gCharByPGN.end() && it->second && hasSubscribers(65281)) {
        uint8_t rssiData[2] = { SRC_GATEWAY, (uint8_t)(int8_t)rssi };
        it->second->setValue(rssiData, 2);

        bool ok = it->second->notify();
        if (ok) {
          gBleNotifySent++;
        } else {
          gBleNotifyFailed++;
          Serial.println("RSSI notify failed");
        }

        if (LOG_NOTIFY_FLOW) {
          Serial.printf("RSSI NOTIFY data=");
          hexDump(rssiData, sizeof(rssiData));
          Serial.println();
        }
      }
    }
  }

  // Drain CAN
  for (int i = 0; i < 96; ++i) {
    twai_message_t m;
    if (twai_receive(&m, 0) != ESP_OK) break;

    gCanFramesRx++;

    if (!m.extd) {
      if (LOG_CAN_RX) {
        Serial.printf("CAN RX STD id=0x%03lX dlc=%u data=",
                      (unsigned long)m.identifier,
                      (unsigned)m.data_length_code);
        hexDump(m.data, m.data_length_code);
        Serial.println();
      }
      continue;
    }

    gCanExtendedRx++;
    gLastN2KRxMs = now;

    if (LOG_CAN_RX) {
      Serial.printf("CAN RX EXT id=0x%08lX dlc=%u data=",
                    (unsigned long)m.identifier,
                    (unsigned)m.data_length_code);
      hexDump(m.data, m.data_length_code);
      Serial.println();
    }

    uint8_t prio, src;
    int16_t dst;
    uint32_t pgn;
    decodeId(m.identifier, prio, pgn, src, dst);

    if (LOG_PGN_DECODE) {
      Serial.printf("N2K DECODE prio=%u PGN=%lu SRC=%u DST=%d %s\n",
                    (unsigned)prio,
                    (unsigned long)pgn,
                    (unsigned)src,
                    (int)dst,
                    isLikelyFast(pgn) ? "[FAST]" : "[SINGLE]");
    }

    // Fast-packet PGNs
    if (isLikelyFast(pgn)) {
      std::vector<uint8_t> payload;
      if (fp_push(pgn, src, m.data, m.data_length_code, payload)) {
        notifyPGN(pgn, src, payload.data(), payload.size(), isAIS(pgn));
      }
      continue;
    }

    // Single-frame PGNs
    notifyPGN(pgn, src, (const uint8_t*)m.data, m.data_length_code, false);
  }

  // Update external RGB status LED
  updateStatusLed(now);

  // Re-advertise watchdog
  static uint32_t lastAdvChk = 0;
  if (now - lastAdvChk > 3000) {
    lastAdvChk = now;
    if (gSrv && gAdv) {
      if (gSrv->getConnectedCount() == 0 && !gAdv->isAdvertising()) {
        Serial.println("ADV watchdog: restarting advertising");
        NimBLEDevice::startAdvertising();
      }
    }
  }
}
