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
               bit 2 = competition route mode, bit 3 = IR turn rule
    [19,0] Use the calibration saved in NVS (reply 0xC2, or NAK if none)
    Competition route (option bit 2 = route mode):
    [20,n] Start uploading a route of n steps (0..32)
    [21,step] One route step: high nibble = action, low nibble = seconds
              1 left, 2 right, 3 straight, 4 U-turn, 5 wait + blue LED,
              6 wait + green LED, 7 finish (stop)
    [22,value] Turn delay: drive forward value x 10 ms before turning
    Replies: 0xE1 route finished, 0xE0 route stopped (turn not completed).
    Settings (every rule's thresholds/timeouts, saved in NVS):
    [23,id] select setting id (see PARAM table)   [24,value] set it
    [26,0] send all settings (frame type 54)      [27,0] reset counters
    [28,s] wheel-balance test: drive straight at base speed for s seconds (1..10), no PID
    Type 54: settings, one byte per setting id 1..PARAM_COUNT.
    Type 55: counters: IR-L count u16, IR-R count u16, junctions, left turns,
             right turns, gaps, flags (bit0 IR-L sees black, bit1 IR-R,
             bit2 line inverted), inversions.
    [18,value] Sharp-turn spin speed % (10..100)
    Frame: A5 5A type length seq payload CRC8(poly=07 over type..payload).
    Type 51: status (14 payload bytes: six PID fields, flags, reset reason
             (esp_reset_reason at last boot, 9 = brownout),
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
struct ParamRecord;

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
// ---- Settings table: every rule's settings, changeable from the app ----
// Stored value is one byte; the comment gives its unit. Saved in NVS.
const uint8_t P_LINE_THRESHOLD = 1;     // x10 = calibrated level (0..1000) that counts as black
const uint8_t P_SHARP_TIMEOUT = 2;      // x100 ms: sharp turn gives up
const uint8_t P_SHARP_END = 3;          // (unused, replaced by P_SHARP_STOP)
const uint8_t P_IR_TRIGGER = 4;         // 0 = turn when line is lost, 1 = turn immediately
const uint8_t P_IR_SPEED = 5;           // % spin speed for the IR turn
const uint8_t P_IR_TIMEOUT = 6;         // (unused: the IR turn has no time settings)
const uint8_t P_IR_END = 7;             // (unused, replaced by P_IR_END_SENSOR)
const uint8_t P_IR_DEBOUNCE = 8;        // (unused: every IR detection is recorded)
const uint8_t P_INVERT_MODE = 9;        // 0 = normal line, 1 = auto detect, 2 = always inverted
const uint8_t P_INVERT_DARK = 10;       // sensors that must be dark to switch (auto)
const uint8_t P_INVERT_CONFIRM = 11;    // x10 ms the pattern must last before switching
const uint8_t P_GAP_MIN = 12;           // (unused: any lost-then-found line is a gap)
const uint8_t P_BOX_DETECT = 13;        // x10 ms whole array dark = inside a box (route)
const uint8_t P_ARM = 14;               // x10 ms centered line before the next junction (route)
const uint8_t P_TURN_TIMEOUT = 15;      // x100 ms route turn gives up
const uint8_t P_JUNCTION_GAP = 16;      // (unused: every junction is counted)
const uint8_t P_EDGE_ZONE = 17;         // x100: line position <= this (or >= 7000 - this) = at the edge
const uint8_t P_SHARP_STOP = 18;        // x100: sharp turn stops when the line is within +/- this of 3500
const uint8_t P_IR_END_SENSOR = 19;     // IR turn stops (PID starts) at: 0 = same IR detects again, 1 = either IR
const uint8_t P_LEFT_TRIM = 20;         // % of commanded power for the LEFT motor (balance the wheels)
const uint8_t P_RIGHT_TRIM = 21;        // % of commanded power for the RIGHT motor
const uint8_t PARAM_COUNT = 21;
struct ParamDef { uint8_t minV, maxV, def; };
const ParamDef PARAM_DEFS[PARAM_COUNT + 1] = {
  {0, 0, 0},       // 0 unused
  {10, 100, 50},   // 1 line threshold (500)
  {1, 50, 15},     // 2 sharp timeout (1.5 s)
  {0, 1, 0},       // 3 sharp end: outer sensor
  {0, 1, 0},       // 4 IR trigger: when line lost
  {10, 100, 35},   // 5 IR spin speed 35 %
  {1, 50, 15},     // 6 IR timeout (1.5 s)
  {0, 1, 1},       // 7 IR end: center sensors
  {0, 255, 0},     // 8 IR repeat filter: 0 = record every detection
  {0, 2, 0},       // 9 invert mode: normal
  {5, 8, 6},       // 10 invert: 6 dark sensors
  {2, 50, 10},     // 11 invert confirm 100 ms
  {1, 100, 5},     // 12 gap minimum 50 ms
  {5, 100, 25},    // 13 box detect 250 ms
  {5, 100, 15},    // 14 arm 150 ms
  {5, 100, 30},    // 15 route turn timeout 3 s
  {5, 100, 30},    // 16 (unused)
  {1, 30, 5},      // 17 edge zone: position <= 500 / >= 6500
  {0, 35, 10},     // 18 sharp stop: within +/-1000 of center
  {0, 1, 0},       // 19 IR stops at: same IR detects again
  {50, 150, 100},  // 20 left motor trim 100 %
  {50, 150, 100},  // 21 right motor trim 100 %
};
uint8_t params[PARAM_COUNT + 1];
uint8_t selectedParam = 0;
#define EDGE_THRESHOLD ((int)params[P_LINE_THRESHOLD] * 10)
#define PIVOT_TIMEOUT_MS ((uint32_t)params[P_SHARP_TIMEOUT] * 100U)
#define ARM_MS ((uint32_t)params[P_ARM] * 10U)
#define BOX_DETECT_MS ((uint32_t)params[P_BOX_DETECT] * 10U)
#define TURN_TIMEOUT_MS ((uint32_t)params[P_TURN_TIMEOUT] * 100U)
const float CENTERED_ERROR = 1.5f;      // line this close to center clears the edge memory
const int DEFAULT_PIVOT_SPEED = 35;     // percent

// ---------------- Pin connections ----------------
const int AIN1 = 42, AIN2 = 41, PWMA = 21; // left motor
const int BIN1 = 40, BIN2 = 39, PWMB = 47; // right motor
const int QTR[8] = {7, 15, 16, 17, 18, 8, 9, 10};
const int IR_LEFT = 14, IR_RIGHT = 4;   // side IR sensors (IR-L, IR-R), digital modules
const int IR_BLACK = HIGH;              // module output level when it sees the black line
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
uint8_t bootResetReason = 0;   // why the ESP32 last restarted (reported to the app)

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
const uint8_t PKT_PARAMS = 0x54;
const uint8_t PKT_COUNTERS = 0x55;
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
const uint8_t RULE_IR_TURN = 0x08;
const uint8_t SRC_SHARP = 0, SRC_IR = 1;
uint8_t pivotSource = SRC_SHARP;         // which rule started the current spin
uint32_t straightTestEndMs = 0;         // wheel-balance test running until this time (0 = off)
bool pivotLeftLine = false;             // sharp turn: the line has left the array during the spin
const uint32_t IR_TURN_FAILSAFE_MS = 5000;  // safety only: never reached in a normal turn

// ---- Side IR sensors ----
bool irLeftActive = false, irRightActive = false;   // debounced: sees black
bool irLeftRose = false, irRightRose = false;       // new detection, for rules
// Every detection is caught by a hardware interrupt, so even a very short
// pulse is recorded while the main loop is busy; updateIR() handles them.
volatile uint16_t irLeftPending = 0, irRightPending = 0;
portMUX_TYPE irMux = portMUX_INITIALIZER_UNLOCKED;
void irLeftIsr();    // declared here: Arduino does not auto-declare IRAM_ATTR functions
void irRightIsr();
int8_t irEdge = 0;                     // -1 = IR-L saw the line last, +1 = IR-R

// ---- Inverted line (white line on black) ----
bool lineInverted = false;
uint32_t invertPatternSinceMs = 0;

// ---- Counters and event flags (for current and future conditions) ----
const uint8_t EV_GAP = 1, EV_JUNCTION = 2, EV_LEFT_TURN = 3, EV_RIGHT_TURN = 4,
              EV_IR_LEFT = 5, EV_IR_RIGHT = 6, EV_INVERTED_ON = 7, EV_INVERTED_OFF = 8;
struct RunCounters {
  uint16_t irLeft, irRight;
  uint8_t junctions, leftTurns, rightTurns, gaps, inversions;
};
RunCounters counters = {};
// Flags: set when the event happens, cleared on START / reset.
bool flagGap = false, flagJunction = false, flagLeftTurn = false, flagRightTurn = false;
bool flagIrLeft = false, flagIrRight = false;
bool lineWasLost = false;
bool junctionActive = false;

// ---- Competition route (option bit 2) ----
const uint8_t RULE_ROUTE = 0x04;
const int MAX_ROUTE_STEPS = 32;
const uint8_t ACT_LEFT = 1, ACT_RIGHT = 2, ACT_STRAIGHT = 3, ACT_UTURN = 4,
              ACT_WAIT_BLUE = 5, ACT_WAIT_GREEN = 6, ACT_FINISH = 7;
// Optional external indicator LEDs (-1 = only use the on-board RGB LED).
const int BLUE_LED_PIN = -1;            // water filled
const int GREEN_LED_PIN = -1;           // people rescued
const uint32_t JUNCTION_TIMEOUT_MS = 1500;
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
const uint32_t PRM_MAGIC = 0x4C465251;  // 'LFRQ'
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

struct ParamRecord {
  uint32_t magic;
  uint16_t version;
  uint8_t count, reserved;
  uint8_t values[32];
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

void defaultParams() {
  for (int i = 0; i <= PARAM_COUNT; ++i) params[i] = PARAM_DEFS[i].def;
}

bool loadParams() {
  defaultParams();
  if (!prefsReady || prefs.getBytesLength("prm") != sizeof(ParamRecord)) return false;
  ParamRecord r = {};
  if (prefs.getBytes("prm", &r, sizeof(r)) != sizeof(r)) return false;
  if (r.magic != PRM_MAGIC || r.version != STORAGE_VERSION) return false;
  if (r.checksum != checksum32(&r, offsetof(ParamRecord, checksum))) return false;
  // Settings added in a newer firmware keep their defaults.
  for (int i = 1; i <= PARAM_COUNT && i <= r.count; ++i) {
    uint8_t v = r.values[i];
    if (v >= PARAM_DEFS[i].minV && v <= PARAM_DEFS[i].maxV) params[i] = v;
  }
  return true;
}

bool saveParams() {
  if (!prefsReady) return false;
  ParamRecord r = {};
  r.magic = PRM_MAGIC;
  r.version = STORAGE_VERSION;
  r.count = PARAM_COUNT;
  for (int i = 0; i <= PARAM_COUNT; ++i) r.values[i] = params[i];
  r.checksum = checksum32(&r, offsetof(ParamRecord, checksum));
  return prefs.putBytes("prm", &r, sizeof(r)) == sizeof(r);
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
  saveParams();

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
  // Wheel balance: scale each motor so both wheels really turn at the same speed.
  left = constrain(left * (int)params[P_LEFT_TRIM] / 100, -255, 255);
  right = constrain(right * (int)params[P_RIGHT_TRIM] / 100, -255, 255);
  lastMotorLeft = left; lastMotorRight = right;
  driveMotor(AIN1, AIN2, PWMA, left);
  driveMotor(BIN1, BIN2, PWMB, right);
}

void stopRobot() {
  startPulseActive = false;
  straightTestEndMs = 0;
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
    if (lineInverted && calOk) signal = 1000 - signal;  // show the line, not the floor
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
  sendCountersFrame();
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
    p[7] = bootResetReason;  // lets the app warn about brownout restarts
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
    if (cmd < 1 || cmd > 28) {
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
        (cmd == 22 && val > 50) ||
        (cmd == 23 && (val < 1 || val > PARAM_COUNT)) ||
        (cmd == 25) ||
        (cmd == 26 && val != 0) ||
        (cmd == 27 && val != 0) ||
        (cmd == 28 && (val < 1 || val > 10))) {
      BT.write(NAK);
      continue;
    }

    if (cmd == 23) { selectedParam = (uint8_t)val; BT.write(ACK); continue; }
    if (cmd == 24) {
      if (selectedParam < 1 || selectedParam > PARAM_COUNT ||
          val < PARAM_DEFS[selectedParam].minV || val > PARAM_DEFS[selectedParam].maxV) {
        BT.write(NAK);
        continue;
      }
      params[selectedParam] = (uint8_t)val;
      if (selectedParam == P_INVERT_MODE) {
        lineInverted = (val == 2);
        invertPatternSinceMs = 0;
      }
      pidDirty = true;                 // saved to NVS together with PID
      lastPIDChangeMs = millis();
      BT.write(ACK);
      continue;
    }
    if (cmd == 26) { sendParamsFrame(); continue; }
    if (cmd == 28) {
      if (robotOn || calibrating || calibrationPending) { BT.write(NAK); continue; }
      straightTestEndMs = millis() + (uint32_t)val * 1000U;
      if (straightTestEndMs == 0) straightTestEndMs = 1;
      Serial.printf("Wheel test: straight at base speed for %d s (trim L %d%% R %d%%)\n",
                    val, params[P_LEFT_TRIM], params[P_RIGHT_TRIM]);
      BT.write(ACK);
      continue;
    }
    if (cmd == 27) { resetCounters(); BT.write(ACK); sendCountersFrame(); continue; }

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
        if (pidDirty) savePID();   // keep new settings even if motors cause a reset
        robotOn = true;
        pidLast = false;
        iTerm = 0.0f;
        lastHeartbeatMs = millis();
        lastUs = micros();
        startPulseAtMs = millis();
        startPulseActive = START_PULSE_MS > 0;
        if (ruleFlags & RULE_ROUTE) routeReset();
        resetCounters();
        lineInverted = params[P_INVERT_MODE] == 2;

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
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  defaultParams();
  attachInterrupt(digitalPinToInterrupt(IR_LEFT), irLeftIsr, IR_BLACK == HIGH ? RISING : FALLING);
  attachInterrupt(digitalPinToInterrupt(IR_RIGHT), irRightIsr, IR_BLACK == HIGH ? RISING : FALLING);
  if (BLUE_LED_PIN >= 0) { pinMode(BLUE_LED_PIN, OUTPUT); digitalWrite(BLUE_LED_PIN, LOW); }
  if (GREEN_LED_PIN >= 0) { pinMode(GREEN_LED_PIN, OUTPUT); digitalWrite(GREEN_LED_PIN, LOW); }

  // Start outputs at 0 duty; STBY on TB6612 must be held HIGH in hardware.
  bool leftPWMok = ledcAttach(PWMA, 20000, 8);
  bool rightPWMok = ledcAttach(PWMB, 20000, 8);
  setMotors(0, 0);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Serial.begin(115200);
  bootResetReason = (uint8_t)esp_reset_reason();
  Serial.printf("Reset reason: %d%s\n", bootResetReason,
                bootResetReason == ESP_RST_BROWNOUT
                    ? "  (BROWNOUT: battery voltage dropped, often when motors start)" : "");
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
  if (loadParams()) Serial.println("Stored rule settings loaded from NVS");
  lineInverted = params[P_INVERT_MODE] == 2;
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
  if (!uturn) robotEvent(dir < 0 ? EV_LEFT_TURN : EV_RIGHT_TURN);
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

// ---------------- Events, counters and flags ----------------
// Every detected event goes through robotEvent(): it updates the counters
// (sent to the app 8x per second) and sets a flag. Add future conditions at
// the bottom of robotEvent(), e.g. "turn at the 3rd IR-L detection".
void robotEvent(uint8_t ev) {
  switch (ev) {
    case EV_GAP:          if (counters.gaps < 255) counters.gaps++; flagGap = true; break;
    case EV_JUNCTION:     if (counters.junctions < 255) counters.junctions++; flagJunction = true; break;
    case EV_LEFT_TURN:    if (counters.leftTurns < 255) counters.leftTurns++; flagLeftTurn = true; break;
    case EV_RIGHT_TURN:   if (counters.rightTurns < 255) counters.rightTurns++; flagRightTurn = true; break;
    case EV_IR_LEFT:      if (counters.irLeft < 65535) counters.irLeft++; flagIrLeft = true; break;
    case EV_IR_RIGHT:     if (counters.irRight < 65535) counters.irRight++; flagIrRight = true; break;
    case EV_INVERTED_ON:
    case EV_INVERTED_OFF: if (counters.inversions < 255) counters.inversions++; break;
  }
  // ---- Future conditions go here, for example: ----
  // if (ev == EV_GAP && counters.gaps == 2) { /* do something at the 2nd gap */ }
  // if (ev == EV_IR_LEFT && counters.irLeft == 3) { /* 3rd left IR detection */ }
}

void resetCounters() {
  counters = RunCounters();
  flagGap = flagJunction = flagLeftTurn = flagRightTurn = false;
  flagIrLeft = flagIrRight = false;
  lineWasLost = false;
  junctionActive = false;
  irEdge = 0;
}

void sendCountersFrame() {
  uint8_t p[10];
  p[0] = (uint8_t)(counters.irLeft & 255);  p[1] = (uint8_t)(counters.irLeft >> 8);
  p[2] = (uint8_t)(counters.irRight & 255); p[3] = (uint8_t)(counters.irRight >> 8);
  p[4] = counters.junctions;
  p[5] = counters.leftTurns;
  p[6] = counters.rightTurns;
  p[7] = counters.gaps;
  p[8] = (irLeftActive ? 1 : 0) | (irRightActive ? 2 : 0) | (lineInverted ? 4 : 0);
  p[9] = counters.inversions;
  sendFramed(PKT_COUNTERS, p, sizeof(p));
}

void sendParamsFrame() {
  sendFramed(PKT_PARAMS, &params[1], PARAM_COUNT);
}

// IR interrupt handlers: run the instant a sensor starts seeing black.
// No time filter: every detection is recorded, however short.
void IRAM_ATTR irLeftIsr() {
  portENTER_CRITICAL_ISR(&irMux);
  if (irLeftPending < 65535) irLeftPending = irLeftPending + 1;
  portEXIT_CRITICAL_ISR(&irMux);
}

void IRAM_ATTR irRightIsr() {
  portENTER_CRITICAL_ISR(&irMux);
  if (irRightPending < 65535) irRightPending = irRightPending + 1;
  portEXIT_CRITICAL_ISR(&irMux);
}

// Counts every detection recorded by the interrupts (none are lost, however
// short) and refreshes the live "sees black" state.
void updateIR() {
  irLeftActive = digitalRead(IR_LEFT) == IR_BLACK;
  irRightActive = digitalRead(IR_RIGHT) == IR_BLACK;
  portENTER_CRITICAL(&irMux);
  uint16_t l = irLeftPending, r = irRightPending;
  irLeftPending = 0;
  irRightPending = 0;
  portEXIT_CRITICAL(&irMux);
  while (l > 0) { irLeftRose = true; robotEvent(EV_IR_LEFT); l--; }
  while (r > 0) { irRightRose = true; robotEvent(EV_IR_RIGHT); r--; }
}

// Inverted line: in "auto" mode, when most sensors are dark but at least one
// sees light (a white line on a black floor) for the confirm time, flip the
// readings so PID keeps following the line. The same test on the flipped
// readings switches back. All sensors dark (junction / box) never flips.
void updateInversion(const int *cal) {
  const uint8_t mode = params[P_INVERT_MODE];
  if (mode != 1) {
    bool want = (mode == 2);
    if (want != lineInverted) {
      lineInverted = want;
      robotEvent(want ? EV_INVERTED_ON : EV_INVERTED_OFF);
    }
    return;
  }
  int dark = 0, light = 0;
  for (int s = 0; s < 8; ++s) {
    if (cal[s] > EDGE_THRESHOLD) dark++;
    else light++;
  }
  const bool pattern = dark >= params[P_INVERT_DARK] && light >= 1;
  const uint32_t now = millis();
  if (!pattern) { invertPatternSinceMs = 0; return; }
  if (invertPatternSinceMs == 0) invertPatternSinceMs = now;
  if (now - invertPatternSinceMs >= (uint32_t)params[P_INVERT_CONFIRM] * 10U) {
    lineInverted = !lineInverted;
    invertPatternSinceMs = 0;
    Serial.println(lineInverted ? "Inverted line detected: following a white line"
                                : "Normal line detected: following a black line");
    robotEvent(lineInverted ? EV_INVERTED_ON : EV_INVERTED_OFF);
  }
}

// Counts junctions (both sides dark) and gaps (line lost, then found again
// without a recovery turn). Purely value based, no time windows, so nothing
// is missed at high speed.
void updateLineEvents(const int *cal, int32_t total) {
  const int thr = EDGE_THRESHOLD;
  int dark = 0;
  for (int s = 0; s < 8; ++s) if (cal[s] > thr) dark++;
  const bool bothSides = cal[0] > thr && cal[1] > thr && cal[6] > thr && cal[7] > thr;
  const bool junction = bothSides || dark >= 7;
  if (junction && !junctionActive) robotEvent(EV_JUNCTION);
  junctionActive = junction;

  if (total == 0) {
    lineWasLost = true;
  } else {
    if (lineWasLost && pivotDir == 0 && routeState == RS_FOLLOW) robotEvent(EV_GAP);
    lineWasLost = false;
  }
}

void startPivot(int8_t dir, uint8_t source) {
  pivotDir = dir;
  pivotSource = source;
  pivotStartMs = millis();
  robotEvent(dir < 0 ? EV_LEFT_TURN : EV_RIGHT_TURN);
  Serial.printf("%s: pivoting %s\n", source == SRC_IR ? "IR turn" : "Sharp turn",
                dir < 0 ? "left" : "right");
}

// ---------------- Rules: sharp-turn recovery + IR turn ----------------
// QTR line position: 0 = line under S1 (far left), 3500 = center,
// 7000 = line under S8 (far right).
//
// Sharp turn (QTR): if the line was in the left edge zone (position <= edge
// zone) and then the whole array loses it, spin left in place (right motor
// forward, left motor backward); right edge mirrored. The spin stops when
// the line is back within +/- the stop window of the center (spinning left
// the line enters at S1 and moves right, so stop at position >= 3500 - window).
//
// IR turn: IR-R detection -> turn right with both wheels (left forward,
// right backward); IR-L mirrored. It starts when the line is lost or
// immediately (setting) and ends the moment the IR detects the line again
// (same IR, or either IR - setting); then PID continues at once.
// Returns true while turning (PID skipped this cycle).
bool edgeTurnRules(const int *cal, int32_t weighted, int32_t total) {
  (void)cal;
  const bool sharpOn = (ruleFlags & RULE_SHARP_TURN) != 0;
  const bool irOn = (ruleFlags & RULE_IR_TURN) != 0;
  const bool leftRose = irLeftRose, rightRose = irRightRose;
  irLeftRose = irRightRose = false;
  if (!sharpOn && !irOn) { pivotDir = 0; return false; }

  const bool seen = total > 0;
  const int32_t pos = seen ? weighted * 1000 / total : -1;   // 0..7000
  const int32_t zone = (int32_t)params[P_EDGE_ZONE] * 100;

  if (pivotDir != 0) {
    bool found;
    bool timedOut;
    if (pivotSource == SRC_IR) {
      found = params[P_IR_END_SENSOR] == 1 ? (leftRose || rightRose)
                                           : (pivotDir < 0 ? leftRose : rightRose);
      timedOut = (uint32_t)(millis() - pivotStartMs) > IR_TURN_FAILSAFE_MS;
    } else {
      if (!seen) pivotLeftLine = true;
      const int32_t window = (int32_t)params[P_SHARP_STOP] * 100;
      found = pivotLeftLine && seen &&
              (pivotDir < 0 ? pos >= 3500 - window : pos <= 3500 + window);
      timedOut = (uint32_t)(millis() - pivotStartMs) > PIVOT_TIMEOUT_MS;
    }
    if (found || timedOut) {
      if (found) Serial.printf("Turn finished (line position %ld), PID resumes\n", (long)pos);
      else Serial.println("Turn safety limit reached, PID resumes");
      pivotDir = 0;
      lastEdge = 0;
      irEdge = 0;
      pidLast = false;   // no derivative kick when PID restarts
      iTerm = 0.0f;
      return false;
    }
    const int speedPct = pivotSource == SRC_IR ? params[P_IR_SPEED] : pivotSpeed;
    const int pwm = speedPct * 255 / 100;
    setMotors(pivotDir < 0 ? -pwm : pwm, pivotDir < 0 ? pwm : -pwm);
    return true;
  }

  const bool leftEdge = seen && pos <= zone;
  const bool rightEdge = seen && pos >= 7000 - zone;
  const bool centered = seen && abs((int)(pos - 3500)) < (int)(CENTERED_ERROR * 1000.0f);

  // Sharp turn memory: on which edge the line was last seen.
  if (sharpOn) {
    if (leftEdge) lastEdge = -1;
    else if (rightEdge) lastEdge = 1;
    else if (centered) lastEdge = 0;
  }
  // IR memory: which side IR detected last (any detection, however short).
  if (irOn) {
    if (leftRose && !rightRose) irEdge = -1;
    else if (rightRose && !leftRose) irEdge = 1;
    else if (irLeftActive && !irRightActive) irEdge = -1;
    else if (irRightActive && !irLeftActive) irEdge = 1;
    else if (centered && !irLeftActive && !irRightActive) irEdge = 0;
  }

  if (irOn && params[P_IR_TRIGGER] == 1 && (leftRose != rightRose)) {
    startPivot(leftRose ? -1 : 1, SRC_IR);       // turn immediately on detection
  } else if (!seen && sharpOn && lastEdge != 0) {
    startPivot(lastEdge, SRC_SHARP);
  } else if (!seen && irOn && irEdge != 0) {
    startPivot(irEdge, SRC_IR);
  } else {
    return false;
  }
  pivotLeftLine = !seen;
  const int speedPct = pivotSource == SRC_IR ? params[P_IR_SPEED] : pivotSpeed;
  const int pwm = speedPct * 255 / 100;
  setMotors(pivotDir < 0 ? -pwm : pwm, pivotDir < 0 ? pwm : -pwm);
  return true;
}

void loop() {
  updateIR();
  if (BT.takeLinkDropped()) {
    telemetryEnabled = false;
    if (straightTestEndMs != 0) { straightTestEndMs = 0; setMotors(0, 0); }
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

  if (straightTestEndMs != 0) {
    if ((int32_t)(millis() - straightTestEndMs) >= 0) {
      straightTestEndMs = 0;
      setMotors(0, 0);
      Serial.println("Wheel test finished");
    } else {
      setMotors(basePwm(), basePwm());   // no PID: shows any drift
    }
    return;
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
    if (lineInverted) calibrated[s] = 1000 - calibrated[s];  // white line on black
    if (calibrated[s] > 250) {
      weighted += (int32_t)(calibrated[s] - 250) * s;
      total += calibrated[s] - 250;
    }
  }

  updateInversion(calibrated);
  updateLineEvents(calibrated, total);

  if (ruleFlags & RULE_ROUTE) {
    if (routeStep(calibrated, total)) return;   // route engine drives this cycle
    if (!robotOn) return;                       // route finished or stopped
  } else if (edgeTurnRules(calibrated, weighted, total)) {
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
