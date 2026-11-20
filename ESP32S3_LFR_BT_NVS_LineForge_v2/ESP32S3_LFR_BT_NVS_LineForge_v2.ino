/*
  ESP32-S3 LFR: QTR-8A + TB6612FNG + built-in BLE (Nordic UART Service)
  Arduino-ESP32 core 3.x | Board: ESP32S3 Dev Module | USB CDC On Boot: Enabled
  No HC-05/HC-06 needed. BLE name: "ESP32-S3 Line Follower".
    Service 6E400001-..., RX 6E400002-... (phone writes), TX 6E400003-... (notify)

  - Power-on does NOT calibrate: it reuses the calibration saved in NVS.
    Calibrate only from the app (RECALIBRATE SENSORS): 2 s stationary on
    WHITE, then 5 s sweeping all 8 sensors over the BLACK line.
    LED: GREEN = calibrated, ORANGE = no calibration yet (press RECALIBRATE),
    WHITE = calibrating, RED = calibration failed.
    (Set CALIBRATE_AT_BOOT=true for the old calibrate-at-power-on behaviour.)
  - Valid calibration is saved in NVS.
  - PID tuning is saved in NVS 2 s after the last changed parameter;
    STOP also saves pending PID changes immediately.
  - Press BOOT (GPIO0) to stop immediately; hold for 2 s AFTER boot,
    or send [8, 1], to recalibrate. Put the array on WHITE first.
  - Recalibration stops the motors and requires a fresh START command.
  - START/STOP emit checksummed status replies so the
    unchanged phone app shows the actual ESP32 run state, not just a request.
  - START includes an adjustable 180 ms motor breakaway pulse. Test wheels up.
  - BLE: incoming commands are ALWAYS 2 binary bytes.
      [1,value] Kp numerator    [2,value] Kp decimal divider (0..4)
      [3,value] Ki numerator    [4,value] Ki decimal divider (0..4)
      [5,value] Kd numerator    [6,value] Kd decimal divider (0..4)
      [7,0] STOP               [7,1] START
      [8,1] Recalibrate manually: 2 s still on white + 5 s hand sweep
      [8,2] Auto calibrate: spin in place on the line
      [8,3] Auto calibrate: wiggle left/right over the line
            (BOOT button held 2 s = manual calibration)
    [9,0] Read back PID and status: reply = 9 bytes:
      [0xD0, kpVal, kpDiv, kiVal, kiDiv, kdVal, kdDiv, flags, checksum]
      flags bit 0=calibration OK, bit 1=running, bit 2=calibrating
      checksum = XOR of 7 bytes after 0xD0 (values + flags).
    [10,1] Heartbeat while running, every 600 ms; timeout 2500 ms.
    [11,1] Enable framed telemetry for 4 s (refresh periodically)
    [11,0] Disable telemetry; [12,0] Read white/black calibration arrays.
    [13,value] Base speed % (0..100)   [14,value] Max speed % (0..100)
    [15,value] Auto-calibration time, seconds (1..10)
    [16,value] Auto-calibration turn speed % (10..100)
    STOP [7,0] during calibration cancels it (previous calibration kept).
    [17,flags] Option bits: bit 0 = sharp-turn recovery,
               bit 1 = keep the saved calibration if a new one fails,
               bit 2 = competition route mode
    [19,0] Use the calibration saved in NVS (reply 0xC2, or NAK if none)
    Competition route (option bit 2 = route mode):
    [20,n] Start uploading a route of n steps (0..32)
    [21,step] One route step: high nibble = action, low nibble = seconds
              1 left, 2 right, 3 straight, 4 U-turn, 5 wait + blue LED,
              6 wait + green LED, 7 finish (stop)
    [22,value] Turn delay: drive forward value x 10 ms before turning
    Replies: 0xE1 route finished, 0xE0 route stopped (turn not completed).
    [18,value] Sharp-turn spin speed % (10..100)
    Frame: A5 5A type length seq payload CRC8(poly=07 over type..payload).
    Type 51: status (14 payload bytes: six PID fields, flags, reserved,
             base speed %, max speed %, rule flags, spin speed %,
             route length, turn delay / 10 ms).
    Type 52: eight QTR intensities + position (u16) + flags + signed PWM L/R.
    Type 53: stored white and black ADC raw readings (32 bytes).
    Replies: 0x06 ACK, 0x15 NAK, 0xC1 calibration success,
             0xC0 calibration failed, 0xC2 stored calibration loaded,
             0xD1 motor safety timeout.
    The 2-byte command format is unchanged for commands 1..7.
    All replies are binary (never text) so the app can verify them.

  IMPORTANT ELECTRICAL REQUIREMENTS:
    - QTR-8A analog outputs MUST stay within the ESP32-S3 input range.
      Power the QTR at 3.3 V (check sensitivity/LED resistor configuration).
    - TB6612FNG STBY must be held HIGH; common ground is essential.
    - Separate motor power and USB/battery connection safely in hardware.
    - GPIO48 is RGB LED on some boards; DevKitC-1 v1.1 uses GPIO38.
*/

#include <Arduino.h>
#include <Preferences.h>
#include <stddef.h>   // offsetof
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Arduino auto-generates function prototypes near the top of the sketch;
// declare the record types early so those prototypes compile.
struct CalibrationRecord;
struct PIDRecord;
struct SpeedRecord;
struct RuleRecord;
struct RouteRecord;

// ---------------- User options ----------------
const bool WAIT_FOR_APP = true;
// false = never calibrate at power-on; calibrate only from the app's
//         RECALIBRATE SENSORS button (saved calibration is reused at boot).
// true  = calibrate (2 s white + 5 s sweep) every time the robot is switched on.
const bool CALIBRATE_AT_BOOT = false;
const int RGB_LED_PIN = 48;              // use 38 for official DevKitC-1 v1.1
// Default speeds (percent of full PWM). The app can change them (commands
// 13/14); the app's values are saved in NVS and used after the next boot.
const int DEFAULT_BASE_SPEED = 25;      // forward speed in PID mode
const int DEFAULT_MAX_SPEED  = 30;      // maximum magnitude of either motor
// START gives a short straight pulse at MAX speed to overcome static friction.
const uint32_t START_PULSE_MS = 180;    // disable by setting to 0
// Automatic calibration defaults (the app sends its own values first).
const int DEFAULT_AUTO_CAL_SECONDS = 3;
const int DEFAULT_AUTO_CAL_SPEED = 30;  // percent
const uint32_t WIGGLE_HALF_MS = 400;   // one left or right swing
const uint32_t PID_SAVE_DELAY_MS = 2000;
const uint32_t CONTROL_PERIOD_US = 5000;
const uint32_t BOOT_HOLD_MS = 2000;
const uint32_t TELEMETRY_PERIOD_MS = 125; // 8Hz, 19-byte packet, ~152B/s
const uint32_t TELEMETRY_LEASE_MS = 4000; // off after app stops refreshing
// When WAIT_FOR_APP=false, the robot may run without a phone.
// Otherwise a lost Bluetooth connection stops motors within 2.5 s.
const bool REQUIRE_APP_HEARTBEAT = WAIT_FOR_APP;
const uint32_t HEARTBEAT_TIMEOUT_MS = 2500;
const int MIN_CAL_RANGE = 300;          // raw 12-bit ADC counts
// Sharp-turn recovery rule (switched on/off from the app's Rules tab).
const int EDGE_THRESHOLD = 500;         // calibrated 0..1000: outer sensor sees line
const float CENTERED_ERROR = 1.5f;      // line this close to center clears the edge memory
const uint32_t PIVOT_TIMEOUT_MS = 1500; // give up spinning if the line is not found
const int DEFAULT_PIVOT_SPEED = 35;     // percent

// ---------------- Pin connections ----------------
const int AIN1 = 42, AIN2 = 41, PWMA = 21; // left motor
const int BIN1 = 40, BIN2 = 39, PWMB = 47; // right motor
const int QTR[8] = {7, 15, 16, 17, 18, 8, 9, 10};
const int USER_BOOT_GPIO  = 0;                // hold after startup to recalibrate

// ---------------- BLE Nordic UART transport ----------------
// Replaces the HC-05/HC-06 UART. It offers the same available/peek/read/write
// calls the protocol code below uses, so the app protocol is unchanged.
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone -> ESP32 (write)
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // ESP32 -> phone (notify)
const char *BLE_DEVICE_NAME = "ESP32-S3 Line Follower";

portMUX_TYPE bleRxMux = portMUX_INITIALIZER_UNLOCKED;

class BleUart {
 public:
  void begin(const char *name);

  int available() {
    portENTER_CRITICAL(&bleRxMux);
    int n = (int)((head + RX_SIZE - tail) % RX_SIZE);
    portEXIT_CRITICAL(&bleRxMux);
    return n;
  }
  int peek() {
    int b = -1;
    portENTER_CRITICAL(&bleRxMux);
    if (head != tail) b = rx[tail];
    portEXIT_CRITICAL(&bleRxMux);
    return b;
  }
  int read() {
    int b = -1;
    portENTER_CRITICAL(&bleRxMux);
    if (head != tail) { b = rx[tail]; tail = (tail + 1) % RX_SIZE; }
    portEXIT_CRITICAL(&bleRxMux);
    return b;
  }

  size_t write(uint8_t b) { return write(&b, 1); }
  // Sends binary bytes as TX notifications. 20-byte chunks fit even the
  // smallest BLE MTU; the app reassembles frames split across notifications.
  size_t write(const uint8_t *data, size_t len) {
    if (!connected || tx == nullptr) return 0;
    for (size_t at = 0; at < len; at += 20) {
      size_t n = (len - at < 20) ? (len - at) : 20;
      tx->setValue((uint8_t *)(data + at), n);
      tx->notify();
    }
    return len;
  }

  // Called from the BLE task when the phone writes to RX.
  void received(const uint8_t *data, size_t len) {
    portENTER_CRITICAL(&bleRxMux);
    for (size_t i = 0; i < len; ++i) {
      size_t next = (head + 1) % RX_SIZE;
      if (next == tail) break;  // buffer full: drop the rest
      rx[head] = data[i];
      head = next;
    }
    portEXIT_CRITICAL(&bleRxMux);
  }
  void linkUp() { connected = true; }
  void linkDown() {
    connected = false;
    portENTER_CRITICAL(&bleRxMux);
    head = 0;                   // discard any half-received command
    tail = 0;
    portEXIT_CRITICAL(&bleRxMux);
    dropped = true;
  }
  bool takeLinkDropped() { bool d = dropped; dropped = false; return d; }
  bool isConnected() const { return connected; }

  BLECharacteristic *tx = nullptr;

 private:
  static const size_t RX_SIZE = 512;
  uint8_t rx[RX_SIZE];
  volatile size_t head = 0, tail = 0;
  volatile bool connected = false, dropped = false;
};

BleUart BT;

class NusRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    // getData/getLength keep 0x00 bytes (e.g. STOP = 07 00) intact.
    BT.received(c->getData(), c->getLength());
  }
};

class NusServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override { BT.linkUp(); }
  void onDisconnect(BLEServer *s) override {
    BT.linkDown();
    BLEDevice::startAdvertising();  // otherwise the phone cannot reconnect
  }
};

void BleUart::begin(const char *name) {
  BLEDevice::init(name);
  BLEDevice::setMTU(247);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new NusServerCallbacks());
  BLEService *service = server->createService(NUS_SERVICE_UUID);
  tx = service->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  tx->addDescriptor(new BLE2902());
  BLECharacteristic *rxChar = service->createCharacteristic(
      NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxChar->setCallbacks(new NusRxCallbacks());
  service->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setScanResponse(true);   // device name goes in the scan response
  BLEDevice::startAdvertising();
}

Preferences prefs;
bool prefsReady = false;

// ---------------- Bluetooth one-byte replies ----------------
const uint8_t ACK = 0x06;
const uint8_t NAK = 0x15;
const uint8_t CAL_SUCCESS = 0xC1;
const uint8_t CAL_FAILURE = 0xC0;
const uint8_t CAL_LOADED = 0xC2;
const uint8_t STATUS_REPLY = 0xD0;
const uint8_t PKT_STATUS = 0x51;
const uint8_t PKT_SENSOR = 0x52;
const uint8_t PKT_CAL = 0x53;
const uint8_t HEARTBEAT_TIMEOUT_REPLY = 0xD1;

// ---------------- Working state ----------------
int kpVal = 40, kpDiv = 0;
int kiVal = 0,  kiDiv = 0;
int kdVal = 1,  kdDiv = 0;
float Kp = 40.0f, Ki = 0.0f, Kd = 1.0f;
int white[8] = {0};
int black[8] = {0};
bool calOk = false;
bool calibrating = false;
bool calibrationPending = false;
const uint8_t CAL_MANUAL = 1, CAL_SPIN = 2, CAL_WIGGLE = 3;
uint8_t pendingCalMode = CAL_MANUAL;
volatile bool calibrationAbort = false;  // set by STOP during calibration
int autoCalSeconds = DEFAULT_AUTO_CAL_SECONDS;
int autoCalSpeed = DEFAULT_AUTO_CAL_SPEED;
const uint8_t RULE_SHARP_TURN = 0x01;
const uint8_t RULE_KEEP_OLD_CAL = 0x02;  // failed calibration falls back to NVS
uint8_t ruleFlags = RULE_KEEP_OLD_CAL;  // cmd 17, saved in NVS (keep-old-cal on by default)
int pivotSpeed = DEFAULT_PIVOT_SPEED;   // cmd 18, saved in NVS
int8_t lastEdge = 0;                    // -1 = S1 (left) saw line last, +1 = S8 (right)
int8_t pivotDir = 0;                    // 0 = not pivoting, -1 left, +1 right
uint32_t pivotStartMs = 0;

// ---- Competition route (option bit 2) ----
const uint8_t RULE_ROUTE = 0x04;
const int MAX_ROUTE_STEPS = 32;
const uint8_t ACT_LEFT = 1, ACT_RIGHT = 2, ACT_STRAIGHT = 3, ACT_UTURN = 4,
              ACT_WAIT_BLUE = 5, ACT_WAIT_GREEN = 6, ACT_FINISH = 7;
// Optional external indicator LEDs (-1 = only use the on-board RGB LED).
const int BLUE_LED_PIN = -1;            // water filled
const int GREEN_LED_PIN = -1;           // people rescued
const uint32_t ARM_MS = 150;            // centered line needed before the next junction
const uint32_t BOX_DETECT_MS = 250;     // whole array dark this long = inside a box
const uint32_t JUNCTION_TIMEOUT_MS = 1500;
const uint32_t TURN_TIMEOUT_MS = 3000;  // a turn that never finds the line stops the run
const uint32_t REVERSE_MS = 250;        // back out of a box before a U-turn
const uint32_t TURN_LEAVE_MIN_MS = 80;
const uint32_t UTURN_LEAVE_MIN_MS = 300;
const uint8_t ROUTE_FINISHED = 0xE1, ROUTE_FAILED = 0xE0;
const uint8_t RS_FOLLOW = 0, RS_JUNCTION = 1, RS_TURN_FWD = 2, RS_TURN_LEAVE = 3,
              RS_TURN_FIND = 4, RS_REVERSE = 5, RS_WAIT = 6;
uint8_t route[MAX_ROUTE_STEPS];
int routeLen = 0;
int routeIdx = 0;
int routeUploadLeft = 0;
int turnDelayMs = 100;                  // cmd 22, saved in NVS
uint8_t routeState = RS_FOLLOW;
uint32_t routeStateMs = 0, armedSinceMs = 0, fullDarkSinceMs = 0, waitMs = 0;
bool routeArmed = false, jSawLeft = false, jSawRight = false, turnIsUTurn = false;
int8_t turnDir = 0;
bool robotOn = false;
bool pidLast = false;
bool startPulseActive = false;
uint32_t startPulseAtMs = 0;
bool pidDirty = false;
float lastError = 0.0f;
float lastSeenSide = 1.0f;
float iTerm = 0.0f;
uint32_t lastUs = 0;
uint32_t lastPIDChangeMs = 0;
uint32_t lastSaveAttemptMs = 0;
uint32_t lastHeartbeatMs = 0;
bool telemetryEnabled = false;
uint32_t lastTelemetryLeaseMs = 0;
uint32_t lastTelemetryMs = 0;
uint8_t frameSequence = 0;
int lastMotorLeft = 0, lastMotorRight = 0;
int baseSpeed = DEFAULT_BASE_SPEED;     // percent, set from app (cmd 13)
int maxSpeed = DEFAULT_MAX_SPEED;       // percent, set from app (cmd 14)

int maxPwm() { return maxSpeed * 255 / 100; }
// Base can never exceed max, otherwise PID could not steer.
int basePwm() { return min(baseSpeed, maxSpeed) * 255 / 100; }

// ---------------- NVS blob definitions ----------------
const uint32_t CAL_MAGIC = 0x4C465243;  // 'LFRC'
const uint32_t PID_MAGIC = 0x4C465250;  // 'LFRP'
const uint32_t SPD_MAGIC = 0x4C465253;  // 'LFRS'
const uint32_t RUL_MAGIC = 0x4C465252;  // 'LFRR'
const uint32_t RTE_MAGIC = 0x4C465254;  // 'LFRT'
const uint16_t STORAGE_VERSION = 1;

struct CalibrationRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  uint16_t white[8];
  uint16_t black[8];
  uint32_t checksum;
};

struct PIDRecord {
  uint32_t magic;
  uint16_t version;
  uint8_t kpVal, kpDiv;
  uint8_t kiVal, kiDiv;
  uint8_t kdVal, kdDiv;
  uint32_t checksum;
};

struct RouteRecord {
  uint32_t magic;
  uint16_t version;
  uint8_t len, turnDelay10;
  uint8_t steps[32];
  uint32_t checksum;
};

struct RuleRecord {
  uint32_t magic;
  uint16_t version;
  uint8_t flags, pivotSpeed;
  uint32_t checksum;
};

struct SpeedRecord {
  uint32_t magic;
  uint16_t version;
  uint8_t baseSpeed, maxSpeed;
  uint32_t checksum;
};

// FNV-1a detects mismatched/corrupted saved records; not a security MAC.
uint32_t checksum32(const void *data, size_t len) {
  const uint8_t *p = (const uint8_t *)data;
  uint32_t h = 2166136261UL;
  for (size_t i = 0; i < len; ++i) {
    h ^= p[i];
    h *= 16777619UL;
  }
  return h;
}

void updatePID() {
  const float tenPow[] = {1.0f, 10.0f, 100.0f, 1000.0f, 10000.0f};
  Kp = kpVal / tenPow[kpDiv];
  Ki = kiVal / tenPow[kiDiv];
  Kd = kdVal / tenPow[kdDiv];
  if (Ki == 0.0f) iTerm = 0.0f;
}

bool loadPID() {
  if (!prefsReady || prefs.getBytesLength("pid") != sizeof(PIDRecord))
    return false;

  PIDRecord r = {};
  if (prefs.getBytes("pid", &r, sizeof(r)) != sizeof(r)) return false;
  if (r.magic != PID_MAGIC || r.version != STORAGE_VERSION) return false;
  if (r.checksum != checksum32(&r, offsetof(PIDRecord, checksum)))
    return false;
  if (r.kpDiv > 4 || r.kiDiv > 4 || r.kdDiv > 4) return false;

  kpVal = r.kpVal; kpDiv = r.kpDiv;
  kiVal = r.kiVal; kiDiv = r.kiDiv;
  kdVal = r.kdVal; kdDiv = r.kdDiv;
  updatePID();
  return true;
}

bool loadSpeed() {
  if (!prefsReady || prefs.getBytesLength("spd") != sizeof(SpeedRecord))
    return false;
  SpeedRecord r = {};
  if (prefs.getBytes("spd", &r, sizeof(r)) != sizeof(r)) return false;
  if (r.magic != SPD_MAGIC || r.version != STORAGE_VERSION) return false;
  if (r.checksum != checksum32(&r, offsetof(SpeedRecord, checksum)))
    return false;
  if (r.baseSpeed > 100 || r.maxSpeed > 100) return false;
  baseSpeed = r.baseSpeed;
  maxSpeed = r.maxSpeed;
  return true;
}

bool saveSpeed() {
  if (!prefsReady) return false;
  SpeedRecord r = {};
  r.magic = SPD_MAGIC;
  r.version = STORAGE_VERSION;
  r.baseSpeed = (uint8_t)baseSpeed;
  r.maxSpeed = (uint8_t)maxSpeed;
  r.checksum = checksum32(&r, offsetof(SpeedRecord, checksum));
  return prefs.putBytes("spd", &r, sizeof(r)) == sizeof(r);
}

bool loadRules() {
  if (!prefsReady || prefs.getBytesLength("rul") != sizeof(RuleRecord)) return false;
  RuleRecord r = {};
  if (prefs.getBytes("rul", &r, sizeof(r)) != sizeof(r)) return false;
  if (r.magic != RUL_MAGIC || r.version != STORAGE_VERSION) return false;
  if (r.checksum != checksum32(&r, offsetof(RuleRecord, checksum))) return false;
  if (r.pivotSpeed < 10 || r.pivotSpeed > 100) return false;
  ruleFlags = r.flags;
  pivotSpeed = r.pivotSpeed;
  return true;
}

bool saveRules() {
  if (!prefsReady) return false;
  RuleRecord r = {};
  r.magic = RUL_MAGIC;
  r.version = STORAGE_VERSION;
  r.flags = ruleFlags;
  r.pivotSpeed = (uint8_t)pivotSpeed;
  r.checksum = checksum32(&r, offsetof(RuleRecord, checksum));
  return prefs.putBytes("rul", &r, sizeof(r)) == sizeof(r);
}

bool loadRoute() {
  if (!prefsReady || prefs.getBytesLength("rte") != sizeof(RouteRecord)) return false;
  RouteRecord r = {};
  if (prefs.getBytes("rte", &r, sizeof(r)) != sizeof(r)) return false;
  if (r.magic != RTE_MAGIC || r.version != STORAGE_VERSION) return false;
  if (r.checksum != checksum32(&r, offsetof(RouteRecord, checksum))) return false;
  if (r.len > MAX_ROUTE_STEPS || r.turnDelay10 > 50) return false;
  routeLen = r.len;
  for (int i = 0; i < routeLen; ++i) route[i] = r.steps[i];
  turnDelayMs = r.turnDelay10 * 10;
  return true;
}

bool saveRoute() {
  if (!prefsReady) return false;
  RouteRecord r = {};
  r.magic = RTE_MAGIC;
  r.version = STORAGE_VERSION;
  r.len = (uint8_t)routeLen;
  r.turnDelay10 = (uint8_t)(turnDelayMs / 10);
  for (int i = 0; i < routeLen; ++i) r.steps[i] = route[i];
  r.checksum = checksum32(&r, offsetof(RouteRecord, checksum));
  return prefs.putBytes("rte", &r, sizeof(r)) == sizeof(r);
}

// Saves PID, speed and rule settings (all change through the same "dirty" flag).
bool savePID() {
  if (!prefsReady) return false;
  saveSpeed();
  saveRules();
  saveRoute();

  PIDRecord r = {};
  r.magic = PID_MAGIC;
  r.version = STORAGE_VERSION;
  r.kpVal = (uint8_t)kpVal; r.kpDiv = (uint8_t)kpDiv;
  r.kiVal = (uint8_t)kiVal; r.kiDiv = (uint8_t)kiDiv;
  r.kdVal = (uint8_t)kdVal; r.kdDiv = (uint8_t)kdDiv;
  r.checksum = checksum32(&r, offsetof(PIDRecord, checksum));

  lastSaveAttemptMs = millis();
  if (prefs.putBytes("pid", &r, sizeof(r)) == sizeof(r)) {
    pidDirty = false;
    Serial.println("PID saved to NVS");
    return true;
  }
  Serial.println("ERROR: PID NVS save failed");
  return false;
}

bool calibrationDataValid(const CalibrationRecord &r) {
  if (r.magic != CAL_MAGIC || r.version != STORAGE_VERSION) return false;
  if (r.checksum != checksum32(&r, offsetof(CalibrationRecord, checksum)))
    return false;
  for (int s = 0; s < 8; ++s) {
    if (r.white[s] > 4095 || r.black[s] > 4095) return false;
    if ((int)r.black[s] - (int)r.white[s] < MIN_CAL_RANGE) return false;
  }
  return true;
}

bool loadCalibration() {
  if (!prefsReady ||
      prefs.getBytesLength("cal") != sizeof(CalibrationRecord))
    return false;

  CalibrationRecord r = {};
  if (prefs.getBytes("cal", &r, sizeof(r)) != sizeof(r)) return false;
  if (!calibrationDataValid(r)) return false;

  for (int s = 0; s < 8; ++s) {
    white[s] = r.white[s];
    black[s] = r.black[s];
  }
  return true;
}

bool saveCalibration() {
  if (!prefsReady) return false;

  CalibrationRecord r = {};
  r.magic = CAL_MAGIC;
  r.version = STORAGE_VERSION;
  for (int s = 0; s < 8; ++s) {
    r.white[s] = (uint16_t)white[s];
    r.black[s] = (uint16_t)black[s];
  }
  r.checksum = checksum32(&r, offsetof(CalibrationRecord, checksum));
  if (prefs.putBytes("cal", &r, sizeof(r)) == sizeof(r)) {
    Serial.println("Calibration saved to NVS");
    return true;
  }
  Serial.println("ERROR: Calibration NVS save failed");
  return false;
}

// ---------------- Hardware helpers ----------------
void showLED(uint8_t r, uint8_t g, uint8_t b) {
  rgbLedWrite(RGB_LED_PIN, r, g, b);
}

int readRaw(int pin) {
  analogRead(pin);  // discard the first read after channel selection
  uint32_t sum = 0;
  for (int k = 0; k < 4; ++k) sum += analogRead(pin);
  return (int)(sum / 4);
}

void driveMotor(int in1, int in2, int pwmPin, int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(in1, (speed >= 0) ? HIGH : LOW);
  digitalWrite(in2, (speed >= 0) ? LOW : HIGH);
  ledcWrite(pwmPin, abs(speed));
}

void setMotors(int left, int right) {
  lastMotorLeft = left; lastMotorRight = right;
  driveMotor(AIN1, AIN2, PWMA, left);
  driveMotor(BIN1, BIN2, PWMB, right);
}

void stopRobot() {
  startPulseActive = false;
  pivotDir = 0;
  lastEdge = 0;
  routeState = RS_FOLLOW;
  robotOn = false;
  pidLast = false;
  iTerm = 0.0f;
  setMotors(0, 0);
}

// CRC8 (poly x^8 + x^2 + x + 1), initial value 0, no final XOR.
uint8_t crc8Update(uint8_t crc, uint8_t b) {
  crc ^= b;
  for (uint8_t n = 0; n < 8; n++)
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  return crc;
}

void sendFramed(uint8_t type, const uint8_t *payload, uint8_t len) {
  if (len > 48) return;
  uint8_t frame[54];
  frame[0] = 0xA5; frame[1] = 0x5A;
  frame[2] = type; frame[3] = len; frame[4] = frameSequence++;
  uint8_t crc = 0;
  for (int i = 2; i < 5; i++) crc = crc8Update(crc, frame[i]);
  for (int i = 0; i < len; i++) {
    frame[5 + i] = payload[i];
    crc = crc8Update(crc, payload[i]);
  }
  frame[5 + len] = crc;
  BT.write(frame, 6 + len);
}

void sendCalibrationFrame() {
  uint8_t p[32];
  for (int i = 0; i < 8; i++) {
    p[4*i+0] = (uint8_t)(white[i] & 0xff);
    p[4*i+1] = (uint8_t)(white[i] >> 8);
    p[4*i+2] = (uint8_t)(black[i] & 0xff);
    p[4*i+3] = (uint8_t)(black[i] >> 8);
  }
  sendFramed(PKT_CAL, p, sizeof(p));
}

// Eight calibrated intensities (0 white to 255 black) plus line position.
// During calibration send raw 12-bit ADC scaled down to 8 bits instead.
void sendSensorFrameIfDue() {
  if (!telemetryEnabled) return;
  const uint32_t now = millis();
  if ((uint32_t)(now - lastTelemetryLeaseMs) > TELEMETRY_LEASE_MS) {
    telemetryEnabled = false;
    return;
  }
  if ((uint32_t)(now - lastTelemetryMs) < TELEMETRY_PERIOD_MS) return;
  lastTelemetryMs = now;
  uint8_t p[16] = {};
  long weighted = 0, total = 0;
  for (int s = 0; s < 8; s++) {
    int raw = readRaw(QTR[s]);
    int signal = 0;
    if (calOk && black[s] - white[s] >= MIN_CAL_RANGE)
      signal = constrain((int32_t)(raw - white[s]) * 1000 /
                (black[s] - white[s]), (int32_t)0, (int32_t)1000);
    else
      signal = raw * 1000L / 4095L;
    p[s] = (uint8_t)((signal * 255L + 500) / 1000);
    if (calOk && signal > 250) {
      weighted += (long)(signal - 250) * s;
      total += (long)(signal - 250);
    }
  }
  uint16_t pos = total > 0 ? (uint16_t)(weighted * 1000L / total) : 0xffff;
  p[8] = (uint8_t)(pos & 255); p[9] = (uint8_t)(pos >> 8);
  p[10] = ((total > 0) ? 1 : 0) | (calOk ? 2 : 0) |
          (robotOn ? 4 : 0) | (calibrating ? 8 : 0) | (pivotDir != 0 ? 16 : 0);
  p[11] = (uint8_t)(int8_t)constrain(lastMotorLeft * 100 / 255, -100, 100);
  p[12] = (uint8_t)(int8_t)constrain(lastMotorRight * 100 / 255, -100, 100);
  p[13] = (uint8_t)routeIdx;   // route progress (steps done)
  p[14] = (uint8_t)routeLen;
  p[15] = routeState;
  sendFramed(PKT_SENSOR, p, sizeof(p));
}

// 9-byte legacy status message or CRC-framed status (when telemetry enabled).
// 9-byte status message. This also lets the app read NVS-restored PID settings.
void sendStatusFrame() {
  uint8_t frame[9];
  frame[0] = STATUS_REPLY;
  frame[1] = (uint8_t)kpVal;
  frame[2] = (uint8_t)kpDiv;
  frame[3] = (uint8_t)kiVal;
  frame[4] = (uint8_t)kiDiv;
  frame[5] = (uint8_t)kdVal;
  frame[6] = (uint8_t)kdDiv;
  frame[7] = (calOk ? 0x01 : 0) | (robotOn ? 0x02 : 0) |
             (calibrating ? 0x04 : 0);
  uint8_t crc = 0;
  for (int i = 1; i <= 7; i++) crc ^= frame[i];
  frame[8] = crc;
  if (telemetryEnabled) {
    uint8_t p[14];
    for (int i = 0; i < 7; ++i) p[i] = frame[i+1];
    p[7] = 0; // reserved for future protocol versions
    p[8] = (uint8_t)baseSpeed;
    p[9] = (uint8_t)maxSpeed;
    p[10] = ruleFlags;
    p[11] = (uint8_t)pivotSpeed;
    p[12] = (uint8_t)routeLen;
    p[13] = (uint8_t)(turnDelayMs / 10);
    sendFramed(PKT_STATUS, p, sizeof(p));
  } else BT.write(frame, sizeof(frame));
}

// ---------------- Bluetooth command reader ----------------
// Called frequently, including inside the blocking calibration routine.
void readBluetooth() {
  while (BT.available() >= 2) {
    int cmd = BT.peek();
    if (cmd < 1 || cmd > 22) {
      BT.read();  // skip invalid leading byte
      continue;
    }
    BT.read();
    int val = BT.read();

    if (((cmd == 2 || cmd == 4 || cmd == 6) && val > 4) ||
        (cmd == 7 && val > 1) ||
        (cmd == 8 && (val < 1 || val > 3)) ||
        (cmd == 9 && val != 0) ||
        (cmd == 10 && val != 1) ||
        (cmd == 11 && val > 1) ||
        (cmd == 12 && val != 0) ||
        ((cmd == 13 || cmd == 14) && val > 100) ||
        (cmd == 15 && (val < 1 || val > 10)) ||
        (cmd == 16 && (val < 10 || val > 100)) ||
        (cmd == 18 && (val < 10 || val > 100)) ||
        (cmd == 19 && val != 0) ||
        (cmd == 20 && val > MAX_ROUTE_STEPS) ||
        (cmd == 21 && ((val >> 4) < ACT_LEFT || (val >> 4) > ACT_FINISH)) ||
        (cmd == 22 && val > 50)) {
      BT.write(NAK);
      continue;
    }

    if (cmd >= 20 && cmd <= 22) {
      if (robotOn) { BT.write(NAK); continue; }  // never change the route mid-run
      if (cmd == 20) {
        routeLen = 0;
        routeUploadLeft = val;
      } else if (cmd == 21) {
        if (routeUploadLeft <= 0 || routeLen >= MAX_ROUTE_STEPS) { BT.write(NAK); continue; }
        route[routeLen++] = (uint8_t)val;
        routeUploadLeft--;
      } else {
        turnDelayMs = val * 10;
      }
      if (cmd == 22 || routeUploadLeft == 0) {
        pidDirty = true;               // saved to NVS together with PID
        lastPIDChangeMs = millis();
        if (cmd != 22) Serial.printf("Route received: %d steps\n", routeLen);
      }
      BT.write(ACK);
      continue;
    }

    if (cmd == 19) {
      // Go back to the calibration stored in NVS (e.g. a new one failed).
      if (calibrating || calibrationPending || robotOn) {
        BT.write(NAK);
      } else if (loadCalibration()) {
        calOk = true;
        showLED(0, 40, 0);
        Serial.println("Saved calibration restored from NVS");
        BT.write(CAL_LOADED);
      } else {
        Serial.println("No valid saved calibration in NVS");
        BT.write(NAK);
      }
      if (telemetryEnabled) { sendCalibrationFrame(); sendStatusFrame(); }
      continue;
    }

    if (cmd == 17 || cmd == 18) {
      if (cmd == 17) {
        ruleFlags = (uint8_t)val;
        if (!(ruleFlags & RULE_SHARP_TURN)) { pivotDir = 0; lastEdge = 0; }
      } else {
        pivotSpeed = val;
      }
      pidDirty = true;                 // saved to NVS together with PID
      lastPIDChangeMs = millis();
      BT.write(ACK);
      Serial.printf("Rules: sharp-turn recovery %s, spin %d%%\n",
                    (ruleFlags & RULE_SHARP_TURN) ? "ON" : "OFF", pivotSpeed);
      continue;
    }

    if (cmd == 15 || cmd == 16) {
      if (cmd == 15) autoCalSeconds = val;
      else autoCalSpeed = val;
      BT.write(ACK);
      continue;
    }

    if (cmd == 13 || cmd == 14) {
      if (cmd == 13) baseSpeed = val;
      else maxSpeed = val;
      pidDirty = true;                 // saved to NVS together with PID
      lastPIDChangeMs = millis();
      BT.write(ACK);
      Serial.printf("Speed: base %d%%  max %d%%\n", baseSpeed, maxSpeed);
      continue;
    }

    if (cmd == 11) {
      telemetryEnabled = (val == 1);
      lastTelemetryLeaseMs = millis();
      if (val == 1) lastTelemetryMs = 0;
      BT.write(ACK);
      continue;
    }
    if (cmd == 12) {
      if (telemetryEnabled) sendCalibrationFrame();
      else BT.write(NAK);
      continue;
    }
    if (cmd == 9) {
      sendStatusFrame();  // 9 bytes, no additional ACK
      continue;
    }
    if (cmd == 10) {
      lastHeartbeatMs = millis();
      // Heartbeats are deliberately silent in telemetry mode.
      if (!telemetryEnabled) BT.write(ACK);
      continue;
    }
    if (cmd == 8) {
      if (calibrating || calibrationPending) {
        BT.write(NAK);
      } else {
        pendingCalMode = (uint8_t)val;
        calibrationPending = true;
        stopRobot();
        BT.write(ACK);
      }
      continue;
    }

    if (cmd == 7) {
      if (val == 0) {
        if (calibrating) calibrationAbort = true;  // cancel calibration
        stopRobot();
        BT.write(ACK);
        sendStatusFrame(); // App already parses this: confirms motors DISARMED.
        Serial.println("BT STOP accepted; motor PWM = 0");
        if (pidDirty) savePID();
      } else if (calOk && !calibrating && !calibrationPending) {
        robotOn = true;
        pidLast = false;
        iTerm = 0.0f;
        lastHeartbeatMs = millis();
        lastUs = micros();
        startPulseAtMs = millis();
        startPulseActive = START_PULSE_MS > 0;
        if (ruleFlags & RULE_ROUTE) routeReset();

        // Drive both motors immediately, before the first QTR PID sample.
        // Gives low-speed motors an initial kick against static friction.
        if (startPulseActive) {
          setMotors(maxPwm(), maxPwm());
        }
        BT.write(ACK);
        sendStatusFrame(); // App already parses this: confirms START accepted.
        Serial.printf("BT START accepted: startup PWM=%d/%d, PID base=%d max=%d\n",
                      maxPwm(), 255, basePwm(), maxPwm());
      } else {
        BT.write(NAK);
        sendStatusFrame(); // Revert app's optimistic RUNNING UI to actual state.
        Serial.printf("BT START rejected: calOk=%d calibrating=%d pending=%d\n",
                      calOk, calibrating, calibrationPending);
      }
      continue;
    }

    bool changed = false;
    switch (cmd) {
      case 1: changed = (kpVal != val); kpVal = val; break;
      case 2: changed = (kpDiv != val); kpDiv = val; break;
      case 3: changed = (kiVal != val); kiVal = val; break;
      case 4: changed = (kiDiv != val); kiDiv = val; break;
      case 5: changed = (kdVal != val); kdVal = val; break;
      case 6: changed = (kdDiv != val); kdDiv = val; break;
    }
    if (changed) {
      updatePID();
      pidDirty = true;
      lastPIDChangeMs = millis();
    }
    BT.write(ACK);
  }
}

// ---------------- Calibration routine ----------------
// Modes (app command [8, mode]):
//   CAL_MANUAL (1): 2 s still on WHITE, then 5 s hand sweep over the line.
//   CAL_SPIN   (2): robot ON the line spins in place for autoCalSeconds.
//   CAL_WIGGLE (3): robot centered ON the line rocks left/right for autoCalSeconds.
// Automatic modes record the lowest (white) and highest (black) reading of
// every sensor while moving. Returns true only when all eight sensors have a
// usable white-to-black range. A failed or cancelled calibration never
// overwrites the previous valid one; STOP or a lost BLE link cancels it.
bool performCalibration(uint8_t mode) {
  const bool automatic = (mode == CAL_SPIN || mode == CAL_WIGGLE);
  const bool hadCalibration = calOk;
  calibrating = true;
  calibrationPending = false;
  calibrationAbort = false;
  calOk = false;
  stopRobot();
  showLED(40, 40, 40);  // WHITE: calibrating

  int newWhite[8];
  int newBlack[8] = {0};
  bool aborted = false;

  if (!automatic) {
    uint32_t sum[8] = {0};
    uint32_t samples = 0;
    uint32_t t0 = millis();
    Serial.println("Calibration (manual): keep all sensors on WHITE for 2 s");
    while ((uint32_t)(millis() - t0) < 2000U && !calibrationAbort) {
      for (int s = 0; s < 8; ++s) sum[s] += (uint32_t)readRaw(QTR[s]);
      ++samples;
      readBluetooth();
      sendSensorFrameIfDue();
      yield();
    }
    for (int s = 0; s < 8; ++s)
      newWhite[s] = samples ? (int)(sum[s] / samples) : 4095;

    Serial.println("Calibration (manual): sweep all sensors over BLACK for 5 s");
    t0 = millis();
    while ((uint32_t)(millis() - t0) < 5000U && !calibrationAbort) {
      for (int s = 0; s < 8; ++s) {
        int raw = readRaw(QTR[s]);
        if (raw > newBlack[s]) newBlack[s] = raw;
      }
      readBluetooth();
      sendSensorFrameIfDue();
      yield();
    }
    aborted = calibrationAbort;
  } else {
    for (int s = 0; s < 8; ++s) newWhite[s] = 4095;
    const int pwm = autoCalSpeed * 255 / 100;
    uint32_t durationMs = (uint32_t)autoCalSeconds * 1000U;
    if (mode == CAL_WIGGLE) {
      // Whole left-right cycles only, so the robot ends back where it started.
      uint32_t cycles = (durationMs + WIGGLE_HALF_MS) / (2U * WIGGLE_HALF_MS);
      if (cycles < 1) cycles = 1;
      durationMs = cycles * 2U * WIGGLE_HALF_MS;
    }
    Serial.printf("Calibration (%s): %lu ms at %d%%\n",
                  mode == CAL_SPIN ? "auto spin" : "auto wiggle",
                  (unsigned long)durationMs, autoCalSpeed);
    uint32_t t0 = millis();
    while (true) {
      uint32_t t = millis() - t0;
      if (t >= durationMs) break;
      if (calibrationAbort || !BT.isConnected()) { aborted = true; break; }
      int dir = 1;  // +1 = turn right (clockwise), -1 = turn left
      if (mode == CAL_WIGGLE) {
        // right for half a swing, then alternate full swings left/right
        uint32_t phase = (t + WIGGLE_HALF_MS / 2U) / WIGGLE_HALF_MS;
        dir = (phase % 2U == 0U) ? 1 : -1;
      }
      setMotors(dir * pwm, -dir * pwm);
      for (int s = 0; s < 8; ++s) {
        int raw = readRaw(QTR[s]);
        if (raw < newWhite[s]) newWhite[s] = raw;
        if (raw > newBlack[s]) newBlack[s] = raw;
      }
      readBluetooth();
      sendSensorFrameIfDue();
      yield();
    }
    setMotors(0, 0);
  }

  if (aborted) {
    stopRobot();
    calOk = hadCalibration;  // white[]/black[] were never touched
    calibrating = false;
    showLED(hadCalibration ? 0 : 40, hadCalibration ? 40 : 15, 0);
    Serial.println("Calibration cancelled; previous calibration kept");
    BT.write(hadCalibration ? CAL_LOADED : CAL_FAILURE);
    if (telemetryEnabled) sendStatusFrame();
    lastUs = micros();
    return false;
  }

  bool good = true;
  for (int s = 0; s < 8; ++s) {
    Serial.printf("D%d white %4d black %4d range %4d\n", s + 1,
                  newWhite[s], newBlack[s], newBlack[s] - newWhite[s]);
    if (newBlack[s] - newWhite[s] < MIN_CAL_RANGE) good = false;
  }

  if (good) {
    for (int s = 0; s < 8; ++s) {
      white[s] = newWhite[s];
      black[s] = newBlack[s];
    }
    calOk = true;
    saveCalibration();  // valid in RAM even if NVS write fails
    showLED(0, 40, 0);  // GREEN: calibration OK
    Serial.println("Calibration OK");
    BT.write(CAL_SUCCESS);
    if (telemetryEnabled) { sendCalibrationFrame(); sendStatusFrame(); }
  } else {
    calOk = false;
    showLED(40, 0, 0);  // RED: calibration failed
    Serial.println("Calibration FAILED");
    BT.write(CAL_FAILURE);
    if ((ruleFlags & RULE_KEEP_OLD_CAL) && loadCalibration()) {
      calOk = true;     // keep driving on the calibration saved in NVS
      showLED(0, 40, 0);
      Serial.println("Using the previous calibration saved in NVS");
      BT.write(CAL_LOADED);
      if (telemetryEnabled) sendCalibrationFrame();
    } else {
      Serial.println("Motors disabled until calibrated");
    }
    if (telemetryEnabled) sendStatusFrame();
  }
  calibrating = false;
  lastUs = micros();
  return good;
}

// Hold BOOT after startup to force recalibration, no app update required.
// Do not hold BOOT through reset/power-up; that enters flash download mode.
void checkBootButton() {
  static bool previouslyDown = false;
  static bool triggered = false;
  static uint32_t pressedAt = 0;

  bool down = (digitalRead(USER_BOOT_GPIO) == LOW);
  if (down && !previouslyDown) {
    pressedAt = millis();
    triggered = false;
    stopRobot();  // immediate stop, even for a short press
  }
  if (down && !triggered && !calibrating &&
      (uint32_t)(millis() - pressedAt) >= BOOT_HOLD_MS) {
    triggered = true;
    pendingCalMode = CAL_MANUAL;
    calibrationPending = true;
    stopRobot();
  }
  if (!down) triggered = false;
  previouslyDown = down;
}

void setup() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(USER_BOOT_GPIO, INPUT_PULLUP);
  if (BLUE_LED_PIN >= 0) { pinMode(BLUE_LED_PIN, OUTPUT); digitalWrite(BLUE_LED_PIN, LOW); }
  if (GREEN_LED_PIN >= 0) { pinMode(GREEN_LED_PIN, OUTPUT); digitalWrite(GREEN_LED_PIN, LOW); }

  // Start outputs at 0 duty; STBY on TB6612 must be held HIGH in hardware.
  bool leftPWMok = ledcAttach(PWMA, 20000, 8);
  bool rightPWMok = ledcAttach(PWMB, 20000, 8);
  setMotors(0, 0);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Serial.begin(115200);
  BT.begin(BLE_DEVICE_NAME);
  Serial.printf("BLE advertising as \"%s\" (Nordic UART Service)\n", BLE_DEVICE_NAME);

  prefsReady = prefs.begin("lfr", false);
  if (!prefsReady) Serial.println("WARNING: NVS unavailable; settings won't persist");
  if (!leftPWMok || !rightPWMok) {
    Serial.println("FATAL: PWM setup failed");
    showLED(40, 0, 0);
    while (true) { setMotors(0, 0); delay(100); }
  }

  if (loadPID()) {
    Serial.println("Stored PID loaded from NVS");
  } else {
    updatePID();
    Serial.println("Using default PID");
    savePID();
  }
  Serial.printf("Kp %.4f Ki %.4f Kd %.4f\n", Kp, Ki, Kd);
  if (loadSpeed()) Serial.println("Stored speed loaded from NVS");
  if (loadRules()) Serial.println("Stored rules loaded from NVS");
  if (loadRoute()) Serial.printf("Stored route loaded: %d steps\n", routeLen);
  Serial.printf("Rules: sharp-turn recovery %s, spin %d%%\n",
                (ruleFlags & RULE_SHARP_TURN) ? "ON" : "OFF", pivotSpeed);
  Serial.printf("Speed: base %d%%  max %d%%\n", baseSpeed, maxSpeed);

  if (CALIBRATE_AT_BOOT) {
    Serial.println("Calibrating now: 2 s on WHITE, then 5 s sweep over the line");
    performCalibration(CAL_MANUAL);
  } else if (loadCalibration()) {
    calOk = true;
    showLED(0, 40, 0);   // GREEN: saved calibration ready
    Serial.println("Stored calibration loaded; no calibration at power-on");
    BT.write(CAL_LOADED);
  } else {
    calOk = false;       // motors stay off until calibrated from the app
    showLED(40, 15, 0);  // ORANGE: waiting for RECALIBRATE SENSORS
    Serial.println("No saved calibration: press RECALIBRATE SENSORS in the app");
  }

  // Only an initial startup is allowed to auto-start when configured.
  // Runtime recalibration always stops until START is explicitly requested.
  if (calOk && !WAIT_FOR_APP) {
    robotOn = true;
    lastHeartbeatMs = millis();
  }
  lastUs = micros();
}

// ---------------- Competition route engine ----------------
// The robot follows the line with PID. When a side of the array (S1+S2 or
// S7+S8) turns dark it drives straight through the junction, noting which
// sides had a branch, then decides:
//   - only one side and nothing ahead = plain 90-degree corner: turn there
//     automatically (no route step used);
//   - otherwise (branch with a straight path, T, cross, or a box border)
//     = run the next route step.
// Waits stop the motors and light the blue (water) or green (rescue) LED;
// a U-turn right after a wait backs out of the box first.

void waterLed(bool on) {
  if (BLUE_LED_PIN >= 0) digitalWrite(BLUE_LED_PIN, on ? HIGH : LOW);
}

void rescueLed(bool on) {
  if (GREEN_LED_PIN >= 0) digitalWrite(GREEN_LED_PIN, on ? HIGH : LOW);
}

void routeReset() {
  routeIdx = 0;
  routeState = RS_FOLLOW;
  routeArmed = false;
  armedSinceMs = 0;
  waterLed(false);
  rescueLed(false);
  showLED(0, 0, 0);
  Serial.printf("Route mode: %d steps\n", routeLen);
}

void enterRouteState(uint8_t state) {
  routeState = state;
  routeStateMs = millis();
}

void routeBackToFollow() {
  enterRouteState(RS_FOLLOW);
  routeArmed = false;
  armedSinceMs = 0;
  pidLast = false;     // no derivative kick when PID restarts
  iTerm = 0.0f;
}

void routeStartTurn(int8_t dir, bool uturn) {
  turnDir = dir;
  turnIsUTurn = uturn;
  enterRouteState(uturn ? RS_REVERSE : RS_TURN_FWD);
}

void routeFinish() {
  Serial.println("Route finished");
  stopRobot();
  BT.write(ROUTE_FINISHED);
  sendStatusFrame();
}

void routeFail(const char *why) {
  Serial.printf("Route stopped: %s\n", why);
  stopRobot();
  BT.write(ROUTE_FAILED);
  sendStatusFrame();
}

// Runs the next programmed step at a junction or box.
void routeNextStep(bool straightAhead) {
  if (routeIdx >= routeLen) {
    if (straightAhead) routeBackToFollow();
    else routeFinish();      // no steps left and nowhere to go
    return;
  }
  const uint8_t step = route[routeIdx++];
  const uint8_t act = step >> 4;
  const uint8_t secs = step & 0x0F;
  Serial.printf("Route step %d/%d: action %d (%d s)\n", routeIdx, routeLen, act, secs);
  switch (act) {
    case ACT_LEFT:     routeStartTurn(-1, false); break;
    case ACT_RIGHT:    routeStartTurn(1, false); break;
    case ACT_STRAIGHT: routeBackToFollow(); break;
    case ACT_UTURN:    routeStartTurn(1, true); break;
    case ACT_WAIT_BLUE:
      showLED(0, 0, 60);
      waterLed(true);
      waitMs = secs * 1000UL;
      setMotors(0, 0);
      enterRouteState(RS_WAIT);
      break;
    case ACT_WAIT_GREEN:
      showLED(0, 60, 0);
      rescueLed(true);
      waitMs = secs * 1000UL;
      setMotors(0, 0);
      enterRouteState(RS_WAIT);
      break;
    default:           routeFinish(); break;
  }
}

// Returns true when the route engine drives the motors this cycle (PID skipped).
bool routeStep(const int *cal, int32_t total) {
  bool d[8];
  int darkCount = 0;
  for (int s = 0; s < 8; ++s) {
    d[s] = cal[s] > EDGE_THRESHOLD;
    if (d[s]) darkCount++;
  }
  const bool center = d[3] || d[4];
  const bool leftSide = d[0] && d[1];
  const bool rightSide = d[6] && d[7];
  const uint32_t now = millis();
  const uint32_t inState = now - routeStateMs;
  const int fwd = basePwm();
  const int spin = pivotSpeed * 255 / 100;
  (void)total;

  switch (routeState) {
    case RS_FOLLOW:
      if (!routeArmed) {
        // Need a normal, centered line first (leaving the start box or a turn).
        if (center && !leftSide && !rightSide) {
          if (armedSinceMs == 0) armedSinceMs = now;
          if (now - armedSinceMs >= ARM_MS) routeArmed = true;
        } else {
          armedSinceMs = 0;
        }
        return false;
      }
      if (leftSide || rightSide) {
        jSawLeft = leftSide;
        jSawRight = rightSide;
        fullDarkSinceMs = 0;
        enterRouteState(RS_JUNCTION);
        setMotors(fwd, fwd);
        return true;
      }
      return false;

    case RS_JUNCTION: {
      jSawLeft = jSawLeft || leftSide;
      jSawRight = jSawRight || rightSide;
      if (darkCount >= 7) { if (fullDarkSinceMs == 0) fullDarkSinceMs = now; }
      else fullDarkSinceMs = 0;
      const bool inBox = fullDarkSinceMs != 0 && now - fullDarkSinceMs >= BOX_DETECT_MS;
      if (!inBox && (leftSide || rightSide) && inState < JUNCTION_TIMEOUT_MS) {
        setMotors(fwd, fwd);   // keep rolling straight through the junction
        return true;
      }
      const bool straightAhead = center && !inBox;
      if (!inBox && !straightAhead && jSawLeft != jSawRight) {
        routeStartTurn(jSawLeft ? -1 : 1, false);  // plain corner: only one way
        return true;
      }
      routeNextStep(straightAhead);
      return routeState != RS_FOLLOW || !robotOn;
    }

    case RS_REVERSE:
      if (inState < REVERSE_MS) { setMotors(-fwd, -fwd); return true; }
      enterRouteState(RS_TURN_LEAVE);
      return true;

    case RS_TURN_FWD:
      if (inState < (uint32_t)turnDelayMs) { setMotors(fwd, fwd); return true; }
      enterRouteState(RS_TURN_LEAVE);
      return true;

    case RS_TURN_LEAVE: {
      setMotors(turnDir < 0 ? -spin : spin, turnDir < 0 ? spin : -spin);
      const uint32_t minLeave = turnIsUTurn ? UTURN_LEAVE_MIN_MS : TURN_LEAVE_MIN_MS;
      if (!center && inState >= minLeave) enterRouteState(RS_TURN_FIND);
      else if (inState > TURN_TIMEOUT_MS) routeFail("could not leave the line");
      return true;
    }

    case RS_TURN_FIND:
      if (center) {
        lastSeenSide = (float)turnDir;
        routeBackToFollow();
        return false;
      }
      setMotors(turnDir < 0 ? -spin : spin, turnDir < 0 ? spin : -spin);
      if (inState > TURN_TIMEOUT_MS) routeFail("line not found after turn");
      return true;

    case RS_WAIT:
      setMotors(0, 0);
      if (inState < waitMs) return true;
      // A U-turn programmed right after a wait runs immediately (leave the box).
      if (routeIdx < routeLen && (route[routeIdx] >> 4) == ACT_UTURN) {
        routeIdx++;
        Serial.printf("Route step %d/%d: U-turn\n", routeIdx, routeLen);
        routeStartTurn(1, true);
      } else {
        routeBackToFollow();
      }
      return true;
  }
  return false;
}

// ---------------- Rule: sharp-turn recovery ----------------
// If the leftmost (S1) or rightmost (S8) sensor saw the line and then the
// whole array loses it (typical 90-degree turn taken too fast), spin in place
// toward that side until that same outer sensor finds the line again:
//   S1 side: right motor forward, left motor backward (turn left)
//   S8 side: left motor forward, right motor backward (turn right)
// Then PID resumes. Returns true while pivoting (PID skipped this cycle).
bool sharpTurnRule(const int *cal, int32_t weighted, int32_t total) {
  if (!(ruleFlags & RULE_SHARP_TURN)) return false;
  const bool leftOuter = cal[0] > EDGE_THRESHOLD;
  const bool rightOuter = cal[7] > EDGE_THRESHOLD;

  if (pivotDir != 0) {
    const bool found = (pivotDir < 0) ? leftOuter : rightOuter;
    const bool timedOut = (uint32_t)(millis() - pivotStartMs) > PIVOT_TIMEOUT_MS;
    if (found || timedOut) {
      Serial.println(found ? "Sharp turn: line found, PID resumes"
                           : "Sharp turn: timeout, PID search resumes");
      pivotDir = 0;
      lastEdge = 0;
      pidLast = false;   // no derivative kick when PID restarts
      iTerm = 0.0f;
      return false;
    }
    const int pwm = pivotSpeed * 255 / 100;
    setMotors(pivotDir < 0 ? -pwm : pwm, pivotDir < 0 ? pwm : -pwm);
    return true;
  }

  // Remember which outer sensor saw the line last (ignore crossings where both do).
  if (leftOuter && !rightOuter) lastEdge = -1;
  else if (rightOuter && !leftOuter) lastEdge = 1;
  else if (total > 0 && !leftOuter && !rightOuter &&
           fabsf((float)weighted / (float)total - 3.5f) < CENTERED_ERROR)
    lastEdge = 0;      // back on the line normally: forget the edge

  if (total == 0 && lastEdge != 0) {
    pivotDir = lastEdge;
    pivotStartMs = millis();
    Serial.printf("Sharp turn: line lost after %s sensor, pivoting %s\n",
                  pivotDir < 0 ? "S1" : "S8", pivotDir < 0 ? "left" : "right");
    const int pwm = pivotSpeed * 255 / 100;
    setMotors(pivotDir < 0 ? -pwm : pwm, pivotDir < 0 ? pwm : -pwm);
    return true;
  }
  return false;
}

void loop() {
  if (BT.takeLinkDropped()) {
    telemetryEnabled = false;
    if (REQUIRE_APP_HEARTBEAT && robotOn) {
      stopRobot();
      Serial.println("Safety stop: BLE connection lost");
    } else {
      Serial.println("BLE connection lost; advertising again");
    }
  }
  readBluetooth();
  checkBootButton();
  sendSensorFrameIfDue();

  if (REQUIRE_APP_HEARTBEAT && robotOn &&
      (uint32_t)(millis() - lastHeartbeatMs) > HEARTBEAT_TIMEOUT_MS) {
    stopRobot();
    BT.write(HEARTBEAT_TIMEOUT_REPLY);
    sendStatusFrame();
    Serial.println("Safety stop: no app heartbeat for 2.5 seconds");
  }

  if (calibrationPending) {
    performCalibration(pendingCalMode);
    // Deliberately remain stopped, even when WAIT_FOR_APP is false.
    stopRobot();
    return;
  }

  // Save only after the user stops changing PID values, to limit flash wear.
  // NVS write may briefly interrupt the control loop.
  if (!robotOn && pidDirty &&
      (uint32_t)(millis() - lastPIDChangeMs) >= PID_SAVE_DELAY_MS &&
      (lastSaveAttemptMs == 0 ||
       (uint32_t)(millis() - lastSaveAttemptMs) >= 5000U)) {
    savePID();
  }

  if (!calOk || !robotOn) return; // stopRobot() already set zero duty

  // Nonblocking startup pulse: still checks Bluetooth, BOOT and heartbeat.
  // No delay() here; STOP always acts immediately.
  if (startPulseActive) {
    if ((uint32_t)(millis() - startPulseAtMs) < START_PULSE_MS) return;
    startPulseActive = false;
    pidLast = false;
    lastUs = micros();
    Serial.println("Startup pulse complete; PID mode entered");
    return;
  }

  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastUs) < CONTROL_PERIOD_US) return;
  float dt = (uint32_t)(nowUs - lastUs) * 1.0e-6f;
  lastUs = nowUs;

  int calibrated[8];
  int32_t weighted = 0, total = 0;
  for (int s = 0; s < 8; ++s) {
    int raw = readRaw(QTR[s]);
    int32_t span = black[s] - white[s]; // >= MIN_CAL_RANGE if calOk
    calibrated[s] = constrain((int32_t)(raw - white[s]) * 1000 / span,
                              (int32_t)0, (int32_t)1000);
    if (calibrated[s] > 250) {
      weighted += (int32_t)(calibrated[s] - 250) * s;
      total += calibrated[s] - 250;
    }
  }

  if (ruleFlags & RULE_ROUTE) {
    if (routeStep(calibrated, total)) return;   // route engine drives this cycle
    if (!robotOn) return;                       // route finished or stopped
  } else if (sharpTurnRule(calibrated, weighted, total)) {
    return;                                     // pivoting this cycle
  }

  float error;
  if (total > 0) {
    error = (float)weighted / (float)total - 3.5f;
    if (error > 0.25f) lastSeenSide = 1.0f;
    else if (error < -0.25f) lastSeenSide = -1.0f;
  } else {
    error = 3.5f * lastSeenSide;  // search toward last known line side
    iTerm = 0.0f;               // prevent windup while line is lost
  }

  if (!pidLast) lastError = error;  // no derivative spike on START
  if (Ki == 0.0f) iTerm = 0.0f;
  else if (total > 0) iTerm = constrain(iTerm + Ki * error * dt, -40.0f, 40.0f);

  float correction = Kp * error + iTerm + Kd * (error - lastError) / dt;
  lastError = error;
  pidLast = true;

  const int MAX_PWM = maxPwm();    // speeds can change live from the app
  const int BASE_PWM = basePwm();
  correction = constrain(correction, (float)(-2 * MAX_PWM),
                         (float)(2 * MAX_PWM));
  int left = BASE_PWM + (int)correction;
  int right = BASE_PWM - (int)correction;
  if (left > MAX_PWM) { right -= left - MAX_PWM; left = MAX_PWM; }
  if (right > MAX_PWM) { left -= right - MAX_PWM; right = MAX_PWM; }
  left = constrain(left, -MAX_PWM, MAX_PWM);
  right = constrain(right, -MAX_PWM, MAX_PWM);
  setMotors(left, right);

  // USB-only debugging, never transmitted over Bluetooth.
  static uint32_t lastPrintMs = 0;
  if (Serial && (uint32_t)(millis() - lastPrintMs) >= 100U) {
    lastPrintMs = millis();
    for (int s = 0; s < 8; ++s) Serial.printf("%5d", calibrated[s]);
    Serial.printf(" | err %+.2f | L %+4d R %+4d | Kp %.3f Ki %.3f Kd %.3f | dt %.1fms\n",
                  error, left, right, Kp, Ki, Kd, dt * 1000.0f);
  }
}
