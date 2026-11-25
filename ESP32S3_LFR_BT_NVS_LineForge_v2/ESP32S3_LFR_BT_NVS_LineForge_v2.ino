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
    Type 55: counters (18 bytes; 14..17 = T, left T, right T, cross): IR-L count u16, IR-R count u16, junctions, left turns,
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
struct ParamRecordOld;
struct IrFilter;

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
const uint8_t P_IR_DEBOUNCE = 8;        // (unused, replaced by P_IR_STABLE)
const uint8_t P_INVERT_MODE = 9;        // 0 = normal line, 1 = auto detect, 2 = always inverted
const uint8_t P_INVERT_DARK = 10;       // sensors that must be dark to switch (auto)
const uint8_t P_INVERT_CONFIRM = 11;    // x10 ms the pattern must last before switching
const uint8_t P_GAP_MIN = 12;           // (unused: any lost-then-found line is a gap)
const uint8_t P_BOX_DETECT = 13;        // x10 ms whole array dark = inside a box (route)
const uint8_t P_ARM = 14;               // x10 ms centered line before the next junction (route)
const uint8_t P_TURN_TIMEOUT = 15;      // x100 ms route turn gives up
const uint8_t P_JUNCTION_CONFIRM = 16;  // readings in a row a junction shape must last (and be gone)
const uint8_t P_EDGE_ZONE = 17;         // x100: line position <= this (or >= 7000 - this) = at the edge
const uint8_t P_SHARP_STOP = 18;        // x100: sharp turn stops when the line is within +/- this of 3500
const uint8_t P_IR_END_SENSOR = 19;     // IR turn stops (PID starts) at: 0 = same IR detects again, 1 = either IR
const uint8_t P_LEFT_TRIM = 20;         // % of commanded power for the LEFT motor (balance the wheels)
const uint8_t P_RIGHT_TRIM = 21;        // % of commanded power for the RIGHT motor
const uint8_t P_SENSOR_FLIP = 22;       // 1 = QTR array mounted mirrored (S1 on the robot's right)
const uint8_t P_MOTOR_SWAP = 23;        // 1 = left/right motor driver channels swapped
const uint8_t P_JUNCTION_IR = 24;       // (unused: junctions use the QTR only)
const uint8_t P_LINE_WIDTH = 25;        // (unused)
const uint8_t P_IR_STABLE = 26;         // ms an IR must stay black (and white) to count: kills flicker
const uint8_t P_LOOKAHEAD = 27;         // x10 ms: drive on after a side path before looking ahead
const uint8_t P_JUNCTION_PREFER = 28;   // (unused)
const uint8_t P_JUNCTION_USE_IR = 29;   // 1 = side IR must also see a side path (when there is a choice)
const uint8_t P_SENSOR_OFF = 30;        // bit s = QTR sensor S(s+1) switched off (broken), S1 = robot's left
const uint8_t P_SHARP_USE_IR = 31;      // 1 = side IR detections also tell the sharp turn which side the line went
const uint8_t P_DEAD_END_ACTION = 32;   // line ends: 0 U-turn, 1 stop, 2 next route step, 3 off
const uint8_t P_DEAD_END_CONFIRM = 33;  // x10 ms the line must stay gone (drives straight meanwhile)
const uint8_t P_IR_MEMORY = 34;         // x10 ms: forget an IR side line after the line went on straight this long
const uint8_t P_RELATIVE = 35;          // 1 = relative black: stretch each reading lightest..darkest
const uint8_t P_REL_CONTRAST = 36;      // x10: minimum darkest-lightest difference to stretch
const uint8_t P_SMOOTH = 37;            // 0 off, 1 light, 2 medium, 3 strong
const uint8_t PARAM_COUNT = 37;
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
  {1, 10, 2},      // 16 junction confirm: 2 readings
  {1, 30, 5},      // 17 edge zone: position <= 500 / >= 6500
  {0, 35, 10},     // 18 sharp stop: within +/-1000 of center
  {0, 1, 0},       // 19 IR stops at: same IR detects again
  {50, 150, 100},  // 20 left motor trim 100 %
  {50, 150, 100},  // 21 right motor trim 100 %
  {0, 1, 1},       // 22 sensor order reversed (this robot: yes)
  {0, 1, 1},       // 23 motor channels swapped (this robot: yes)
  {0, 1, 0},       // 24 IR confirms branches: off
  {1, 5, 3},       // 25 normal line width: 3 sensors
  {0, 50, 4},      // 26 IR stable time 4 ms
  {0, 30, 3},      // 27 look-ahead 30 ms
  {0, 3, 0},       // 28 route OFF choice: straight, else left
  {0, 1, 1},       // 29 IR confirms side paths: on
  {0, 255, 0},     // 30 sensors switched off: none
  {0, 1, 1},       // 31 sharp turn uses IR: on
  {0, 3, 0},       // 32 line ends: U-turn
  {1, 50, 5},      // 33 line gone for 50 ms
  {5, 100, 20},    // 34 IR memory 200 ms
  {0, 1, 1},       // 35 relative black: on
  {5, 80, 40},     // 36 minimum contrast 40 %
  {0, 3, 1},       // 37 smoothing: light
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
// Flicker filter: a detection counts only after the module output stayed
// black for P_IR_STABLE ms; it must stay white as long before the next one.
struct IrFilter { bool state; bool raw; uint32_t sinceUs; };
IrFilter irFiltL = {false, false, 0}, irFiltR = {false, false, 0};
uint32_t irLeftRiseMs = 0, irRightRiseMs = 0;     // when each IR last counted a detection
int8_t irEdge = 0;                     // -1 = IR-L saw the line last, +1 = IR-R
int8_t sharpIrEdge = 0;                // sharp turn + IR option: side where an IR saw a line
uint32_t sharpIrWhiteMs = 0;           // when both IRs went white again (0 = not yet)

// ---- Inverted line (white line on black) ----
bool lineInverted = false;
uint32_t invertPatternSinceMs = 0;

// ---- Counters and event flags (for current and future conditions) ----
const uint8_t EV_GAP = 1, EV_JUNCTION = 2, EV_LEFT_TURN = 3, EV_RIGHT_TURN = 4,
              EV_IR_LEFT = 5, EV_IR_RIGHT = 6, EV_INVERTED_ON = 7, EV_INVERTED_OFF = 8;
struct RunCounters {
  uint16_t irLeft, irRight;
  uint8_t junctions, leftTurns, rightTurns, gaps, inversions;
  uint8_t tJunctions, leftTs, rightTs, crosses;   // per junction type
};
RunCounters counters = {};
// Flags: set when the event happens, cleared on START / reset.
bool flagGap = false, flagJunction = false, flagLeftTurn = false, flagRightTurn = false;
bool flagIrLeft = false, flagIrRight = false;
bool lineWasLost = false;

// ---- Junction detection (shape based, see updateLineEvents) ----
const uint8_t JT_NONE = 0, JT_LEFT_CORNER = 1, JT_RIGHT_CORNER = 2, JT_LEFT_BRANCH = 3,
              JT_RIGHT_BRANCH = 4, JT_T = 5, JT_CROSS = 6, JT_BOX = 7;
bool jActive = false;              // crossing a junction / corner shape right now
bool jLeft = false, jRight = false, jBoxReported = false;
bool jLooking = false, jIrL = false, jIrR = false;   // look-ahead phase, IR saw a side path
uint32_t jLookStartMs = 0;
const uint32_t JUNCTION_IR_EARLY_MS = 150;  // IR may see the side path just before the QTR
uint8_t jWideCount = 0, jClearCount = 0;
uint32_t jStartMs = 0, jAllDarkSinceMs = 0;
uint8_t jDoneType = JT_NONE;       // set for ONE control cycle when a shape is classified
bool jDoneStraight = false;        // line continues straight ahead after it
uint8_t lastJunctionType = JT_NONE, junctionSeq = 0;   // reported to the app

// ---- Competition route (option bit 2) ----
const uint8_t RULE_ROUTE = 0x04;
const int MAX_ROUTE_STEPS = 32;
const uint8_t ACT_LEFT = 1, ACT_RIGHT = 2, ACT_STRAIGHT = 3, ACT_UTURN = 4,
              ACT_WAIT_BLUE = 5, ACT_WAIT_GREEN = 6, ACT_FINISH = 7;
// Optional external indicator LEDs (-1 = only use the on-board RGB LED).
const int BLUE_LED_PIN = -1;            // water filled
const int GREEN_LED_PIN = -1;           // people rescued
const uint32_t JUNCTION_TIMEOUT_MS = 1500;   // safety: a junction shape never lasts longer
const uint32_t REVERSE_MS = 250;        // back out of a box before a U-turn
const uint32_t TURN_LEAVE_MIN_MS = 80;
const uint32_t UTURN_LEAVE_MIN_MS = 300;
const uint8_t ROUTE_FINISHED = 0xE1, ROUTE_FAILED = 0xE0, LINE_ENDED_STOP = 0xE2;
// Dead end: the line disappears while it was near the center.
bool deadEndArmed = false, deadEndChecking = false;
uint32_t deadEndArmMs = 0, deadEndLostMs = 0;
int32_t lastLinePos = 3500;
const uint8_t RS_FOLLOW = 0, RS_JUNCTION = 1, RS_TURN_FWD = 2, RS_TURN_LEAVE = 3,
              RS_TURN_FIND = 4, RS_REVERSE = 5, RS_WAIT = 6;
uint8_t route[MAX_ROUTE_STEPS];
int routeLen = 0;
int routeIdx = 0;
int routeUploadLeft = 0;
int turnDelayMs = 100;                  // cmd 22, saved in NVS
uint8_t routeState = RS_FOLLOW;
uint32_t routeStateMs = 0, armedSinceMs = 0, waitMs = 0;
bool routeArmed = false, turnIsUTurn = false;
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

// Settings record. Old firmware saved 32 values; it is still read once
// (key "prm"), new saves use 64 values under key "prm64".
const int PARAM_SLOTS = 64;
struct ParamRecord {
  uint32_t magic;
  uint16_t version;
  uint8_t count, reserved;
  uint8_t values[PARAM_SLOTS];
  uint32_t checksum;
};
struct ParamRecordOld {
  uint32_t magic;
  uint16_t version;
  uint8_t count, reserved;
  uint8_t values[32];
  uint32_t checksum;
};
static_assert(PARAM_COUNT < PARAM_SLOTS, "too many settings for the NVS record");

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

// Copies stored values; settings added in a newer firmware keep their defaults.
void applyStoredParams(const uint8_t *values, int count, int slots) {
  for (int i = 1; i <= PARAM_COUNT && i <= count && i < slots; ++i) {
    uint8_t v = values[i];
    if (v >= PARAM_DEFS[i].minV && v <= PARAM_DEFS[i].maxV) params[i] = v;
  }
}

bool loadParams() {
  defaultParams();
  if (!prefsReady) return false;
  if (prefs.getBytesLength("prm64") == sizeof(ParamRecord)) {
    ParamRecord r = {};
    if (prefs.getBytes("prm64", &r, sizeof(r)) == sizeof(r) && r.magic == PRM_MAGIC &&
        r.version == STORAGE_VERSION &&
        r.checksum == checksum32(&r, offsetof(ParamRecord, checksum))) {
      applyStoredParams(r.values, r.count, PARAM_SLOTS);
      return true;
    }
  }
  if (prefs.getBytesLength("prm") == sizeof(ParamRecordOld)) {   // saved by older firmware
    ParamRecordOld r = {};
    if (prefs.getBytes("prm", &r, sizeof(r)) == sizeof(r) && r.magic == PRM_MAGIC &&
        r.version == STORAGE_VERSION &&
        r.checksum == checksum32(&r, offsetof(ParamRecordOld, checksum))) {
      applyStoredParams(r.values, r.count, 32);
      return true;
    }
  }
  return false;
}

bool saveParams() {
  if (!prefsReady) return false;
  ParamRecord r = {};
  r.magic = PRM_MAGIC;
  r.version = STORAGE_VERSION;
  r.count = PARAM_COUNT;
  for (int i = 0; i <= PARAM_COUNT; ++i) r.values[i] = params[i];
  r.checksum = checksum32(&r, offsetof(ParamRecord, checksum));
  return prefs.putBytes("prm64", &r, sizeof(r)) == sizeof(r);
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
    if (channelOn(s) && (int)r.black[s] - (int)r.white[s] < MIN_CAL_RANGE) return false;
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

// Logical sensor 0 = the robot's physical LEFT side. When the array is
// mounted mirrored, logical sensor s is read from QTR channel 7 - s.
// Calibration (white/black) stays stored per physical channel.
int qtrChannel(int s) { return params[P_SENSOR_FLIP] ? 7 - s : s; }

// Sensors switched off in the app (broken) always read white and are
// ignored by position, PID, junctions, inversion and calibration checks.
bool sensorOn(int s) { return (params[P_SENSOR_OFF] & (1 << s)) == 0; }
bool channelOn(int ch) { return sensorOn(qtrChannel(ch)); }   // qtrChannel is its own inverse
// First working sensor starting at 'from', moving by 'step' (+1 right, -1 left).
int workingSensor(int from, int step) {
  for (int s = from; s >= 0 && s < 8; s += step) if (sensorOn(s)) return s;
  return constrain(from, 0, 7);
}
int workingCount() {
  int n = 0;
  for (int s = 0; s < 8; ++s) if (sensorOn(s)) n++;
  return n;
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
  if (params[P_MOTOR_SWAP]) {   // driver channel A is wired to the RIGHT wheel
    driveMotor(AIN1, AIN2, PWMA, right);
    driveMotor(BIN1, BIN2, PWMB, left);
  } else {
    driveMotor(AIN1, AIN2, PWMA, left);
    driveMotor(BIN1, BIN2, PWMB, right);
  }
}

void stopRobot() {
  startPulseActive = false;
  deadEndArmed = false;
  deadEndChecking = false;
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
    const int ch = qtrChannel(i);   // left-to-right as the robot sees it
    p[4*i+0] = (uint8_t)(white[ch] & 0xff);
    p[4*i+1] = (uint8_t)(white[ch] >> 8);
    p[4*i+2] = (uint8_t)(black[ch] & 0xff);
    p[4*i+3] = (uint8_t)(black[ch] >> 8);
  }
  sendFramed(PKT_CAL, p, sizeof(p));
}

// ---------------- Uneven floor: smoothing + relative black ----------------
// When the floor is not flat the QTR moves up and down: lifted, the black
// line reads only grey; lowered, white reads darker. Two fixes:
//  - smoothing: each sensor is averaged with its previous readings, so one
//    bad reading does not jerk the robot (light = 1/2, medium = 1/3, strong = 1/4);
//  - relative black: every reading is stretched so the lightest working
//    sensor becomes 0 and the darkest 1000. The line then stays "black" even
//    when the whole array reads lighter. Only done when the difference is at
//    least P_REL_CONTRAST, so plain white floor (only noise) stays white and
//    an all-dark junction / box stays dark.
int smoothCal[8] = {0};
bool smoothReady = false;            // cleared at START
int lastProcessed[8] = {0};          // what the control loop used (shown in the app)
int32_t lastProcessedPos = -1;

void applySmoothing(int *v) {
  const int k = params[P_SMOOTH];
  if (k == 0 || !smoothReady) {
    for (int s = 0; s < 8; ++s) smoothCal[s] = v[s];
    smoothReady = true;
    return;
  }
  for (int s = 0; s < 8; ++s) {
    smoothCal[s] += (v[s] - smoothCal[s]) / (k + 1);
    v[s] = smoothCal[s];
  }
}

void applyRelative(int *v) {
  if (!params[P_RELATIVE]) return;
  int lo = 1000, hi = 0;
  for (int s = 0; s < 8; ++s) {
    if (!sensorOn(s)) continue;
    lo = min(lo, v[s]);
    hi = max(hi, v[s]);
  }
  if (hi - lo < (int)params[P_REL_CONTRAST] * 10) return;
  for (int s = 0; s < 8; ++s)
    if (sensorOn(s)) v[s] = (int32_t)(v[s] - lo) * 1000 / (hi - lo);
}

// Line position from processed values: 0 (S1) .. 7000 (S8).
void linePosition(const int *v, int32_t &weighted, int32_t &total) {
  weighted = 0;
  total = 0;
  for (int s = 0; s < 8; ++s) {
    if (v[s] > 250) {
      weighted += (int32_t)(v[s] - 250) * s;
      total += v[s] - 250;
    }
  }
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
  int sig[8] = {0};
  for (int s = 0; s < 8; s++) {
    if (!sensorOn(s)) continue;        // switched off: shown as 0
    const int ch = qtrChannel(s);
    int raw = readRaw(QTR[ch]);
    int signal = 0;
    if (calOk && black[ch] - white[ch] >= MIN_CAL_RANGE)
      signal = constrain((int32_t)(raw - white[ch]) * 1000 /
                (black[ch] - white[ch]), (int32_t)0, (int32_t)1000);
    else
      signal = raw * 1000L / 4095L;
    if (lineInverted && calOk) signal = 1000 - signal;  // show the line, not the floor
    sig[s] = signal;
  }
  int32_t weighted = 0, total = 0;
  if (calOk && !calibrating) {
    if (robotOn) {
      for (int s = 0; s < 8; ++s) sig[s] = lastProcessed[s];   // what PID really used
    } else {
      applyRelative(sig);
    }
    linePosition(sig, weighted, total);
  }
  for (int s = 0; s < 8; s++) p[s] = (uint8_t)((sig[s] * 255L + 500) / 1000);
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
        smoothReady = false;   // smoothing starts fresh
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
    Serial.printf("D%d white %4d black %4d range %4d%s\n", s + 1,
                  newWhite[s], newBlack[s], newBlack[s] - newWhite[s],
                  channelOn(s) ? "" : "  (switched off)");
    if (channelOn(s) && newBlack[s] - newWhite[s] < MIN_CAL_RANGE) good = false;
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
// The robot follows the line with PID. When the junction detector (see
// updateJunction) sees a junction shape it drives straight across it,
// then decides:
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
  deadEndArmed = false;
  deadEndChecking = false;
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
// Junctions come from the shape detector (updateJunction, run just before).
bool routeStep(const int *cal, int32_t total) {
  bool d[8];
  for (int s = 0; s < 8; ++s) d[s] = cal[s] > EDGE_THRESHOLD;
  const bool center = d[3] || d[4];
  const int l1 = workingSensor(0, 1), r1 = workingSensor(7, -1);
  const bool leftSide = d[l1] && d[workingSensor(l1 + 1, 1)];
  const bool rightSide = d[r1] && d[workingSensor(r1 - 1, -1)];
  const uint32_t now = millis();
  const uint32_t inState = now - routeStateMs;
  const int fwd = basePwm();
  const int spin = pivotSpeed * 255 / 100;
  (void)total;

  switch (routeState) {
    case RS_FOLLOW:
    case RS_JUNCTION:
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
      if (jDoneType == JT_LEFT_CORNER || jDoneType == JT_RIGHT_CORNER) {
        routeStartTurn(jDoneType == JT_LEFT_CORNER ? -1 : 1, false);  // only one way: no step used
        return true;
      }
      if (jDoneType != JT_NONE) {       // branch, T, cross or box
        routeNextStep(jDoneStraight);   // next programmed step
        return routeState != RS_FOLLOW || !robotOn;
      }
      if (jActive) {
        if (routeState != RS_JUNCTION) enterRouteState(RS_JUNCTION);
        setMotors(fwd, fwd);            // roll straight across while it is classified
        return true;
      }
      if (routeState == RS_JUNCTION) enterRouteState(RS_FOLLOW);   // shape ignored (IR option)
      return false;

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

// ---------------- Line ends (dead end) ----------------
// The line disappears while it was near the center (not at an edge, where it
// is a turn): drive straight for P_DEAD_END_CONFIRM so a small gap is crossed,
// then, if the line is still gone: U-turn (default), stop, or the next route
// step. Junction shapes and edge losses (sharp turns) are not dead ends.
// Returns true when it drives the motors this cycle.
bool deadEndRule(int32_t weighted, int32_t total) {
  const uint8_t action = params[P_DEAD_END_ACTION];
  if (action == 3 || routeState != RS_FOLLOW || pivotDir != 0 || jActive) {
    deadEndChecking = false;
    return false;
  }
  const uint32_t now = millis();
  if (total > 0) {
    lastLinePos = weighted * 1000 / total;
    // Armed after following a centered line for the settle time (not in the start box).
    if (abs((int)(lastLinePos - 3500)) <= 1500) {
      if (deadEndArmMs == 0) deadEndArmMs = now;
      if (now - deadEndArmMs >= ARM_MS) deadEndArmed = true;
    } else {
      deadEndArmMs = 0;
    }
    deadEndChecking = false;
    return false;
  }
  deadEndArmMs = 0;
  if (!deadEndArmed || abs((int)(lastLinePos - 3500)) > 1500) return false;  // lost at an edge
  if (!deadEndChecking) { deadEndChecking = true; deadEndLostMs = now; }
  if (now - deadEndLostMs < (uint32_t)params[P_DEAD_END_CONFIRM] * 10U) {
    setMotors(basePwm(), basePwm());   // straight on: maybe only a gap
    return true;
  }
  deadEndChecking = false;
  deadEndArmed = false;
  if (action == 1) {
    Serial.println("Line ended: stop");
    stopRobot();
    BT.write(LINE_ENDED_STOP);
    sendStatusFrame();
    return true;
  }
  if (action == 2 && (ruleFlags & RULE_ROUTE)) {
    Serial.println("Line ended: next route step");
    routeNextStep(false);
    return true;
  }
  Serial.println("Line ended: U-turn");
  routeStartTurn(1, true);
  return true;
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
  resetJunction();
  lastJunctionType = JT_NONE;
  irEdge = 0;
  sharpIrEdge = 0;
}

// 0 stopped, 1/2 sharp turn L/R, 3/4 IR turn L/R, 5/6 route turn L/R,
// 7 crossing a junction, 8 waiting (route), 9 following the line (PID),
// 10 line disappeared: driving straight to check for a dead end.
uint8_t activityCode() {
  if (!robotOn) return 0;
  if (pivotDir != 0) return pivotSource == SRC_IR ? (pivotDir < 0 ? 3 : 4) : (pivotDir < 0 ? 1 : 2);
  if (routeState == RS_TURN_FWD || routeState == RS_TURN_LEAVE || routeState == RS_TURN_FIND ||
      routeState == RS_REVERSE) return turnDir < 0 ? 5 : 6;
  if (routeState == RS_WAIT) return 8;
  if (jActive || routeState == RS_JUNCTION) return 7;
  if (deadEndChecking) return 10;
  return 9;
}

void sendCountersFrame() {
  uint8_t p[18];
  p[0] = (uint8_t)(counters.irLeft & 255);  p[1] = (uint8_t)(counters.irLeft >> 8);
  p[2] = (uint8_t)(counters.irRight & 255); p[3] = (uint8_t)(counters.irRight >> 8);
  p[4] = counters.junctions;
  p[5] = counters.leftTurns;
  p[6] = counters.rightTurns;
  p[7] = counters.gaps;
  p[8] = (irLeftActive ? 1 : 0) | (irRightActive ? 2 : 0) | (lineInverted ? 4 : 0);
  p[9] = counters.inversions;
  p[10] = lastJunctionType;   // JT_* of the last shape the robot recognised
  p[11] = junctionSeq;        // +1 per recognised shape (app logs each one)
  p[12] = activityCode();     // what the robot is doing right now
  p[13] = (lastEdge < 0 ? 1 : 0) | (lastEdge > 0 ? 2 : 0) |      // which way it would turn
          (irEdge < 0 ? 4 : 0) | (irEdge > 0 ? 8 : 0) |          // if the line is lost now
          (sharpIrEdge < 0 ? 16 : 0) | (sharpIrEdge > 0 ? 32 : 0);
  p[14] = counters.tJunctions;
  p[15] = counters.leftTs;
  p[16] = counters.rightTs;
  p[17] = counters.crosses;
  sendFramed(PKT_COUNTERS, p, sizeof(p));
}

void sendParamsFrame() {
  sendFramed(PKT_PARAMS, &params[1], PARAM_COUNT);
}

// One IR channel: the output must stay at a new level for P_IR_STABLE ms
// before it is accepted. Flicker on the edge of a line or motor noise never
// lasts that long, so one line crossing = exactly one count.
// Returns true when a new black detection is accepted.
bool irFilterUpdate(IrFilter &f, bool black, uint32_t nowUs) {
  const uint32_t stableUs = (uint32_t)params[P_IR_STABLE] * 1000U;
  if (black != f.raw) { f.raw = black; f.sinceUs = nowUs; }
  if (f.raw != f.state && (uint32_t)(nowUs - f.sinceUs) >= stableUs) {
    f.state = f.raw;
    return f.state;
  }
  return false;
}

// Polled every loop pass (many times per millisecond).
void updateIR() {
  const uint32_t nowUs = micros();
  if (irFilterUpdate(irFiltL, digitalRead(IR_LEFT) == IR_BLACK, nowUs)) {
    irLeftRose = true;
    irLeftRiseMs = millis();
    robotEvent(EV_IR_LEFT);
  }
  if (irFilterUpdate(irFiltR, digitalRead(IR_RIGHT) == IR_BLACK, nowUs)) {
    irRightRose = true;
    irRightRiseMs = millis();
    robotEvent(EV_IR_RIGHT);
  }
  irLeftActive = irFiltL.state;
  irRightActive = irFiltR.state;
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
    if (!sensorOn(s)) continue;
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

// ---------------- Junction detection (QTR + side IR) ----------------
// Sensor 0 = robot's left (S1), 7 = right (S8). Center = S4 or S5.
//   right path : S7 + S8 dark while the center is dark, and IR-R sees the line
//   left path  : S1 + S2 dark while the center is dark, and IR-L sees the line
// When the side sensors are white again the robot drives on a few mm
// (look-ahead) in a straight line, then looks at the center:
//   center dark  -> the line goes on: junction (branch, or cross if both sides)
//   center white -> the line ended: 90-degree corner (or T if both sides)
// A curve moves the line to the edge with the center white, and (with the IR
// option) a side path also needs the side IR, so curves are not junctions.
// Route ON : left T / right T / T / cross / box -> next route step;
//            90-degree corner -> turns by itself (no step); curves -> PID.
// Route OFF: junctions are only counted (and shown in the app log).

const char *junctionName(uint8_t t) {
  switch (t) {
    case JT_LEFT_CORNER:  return "left corner";
    case JT_RIGHT_CORNER: return "right corner";
    case JT_LEFT_BRANCH:  return "left T (left + straight)";
    case JT_RIGHT_BRANCH: return "right T (right + straight)";
    case JT_T:            return "T junction";
    case JT_CROSS:        return "cross (+)";
    case JT_BOX:          return "box / stop zone";
    default:              return "none";
  }
}

void reportJunction(uint8_t type, bool straight) {
  jDoneType = type;
  jDoneStraight = straight;
  lastJunctionType = type;
  junctionSeq++;
  if (type == JT_LEFT_BRANCH || type == JT_RIGHT_BRANCH || type == JT_T || type == JT_CROSS)
    robotEvent(EV_JUNCTION);
  uint8_t *c = type == JT_T ? &counters.tJunctions : type == JT_LEFT_BRANCH ? &counters.leftTs
             : type == JT_RIGHT_BRANCH ? &counters.rightTs : type == JT_CROSS ? &counters.crosses : nullptr;
  if (c != nullptr && *c < 255) *c = *c + 1;
  Serial.printf("Junction: %s\n", junctionName(type));
}

void resetJunction() {
  jActive = false;
  jLooking = false;
  jWideCount = jClearCount = 0;
  jAllDarkSinceMs = 0;
  jDoneType = JT_NONE;
}

void updateJunction(const int *cal) {
  jDoneType = JT_NONE;
  // Only while following the line (not while turning, waiting or spinning).
  if (pivotDir != 0 || (routeState != RS_FOLLOW && routeState != RS_JUNCTION)) {
    resetJunction();
    return;
  }
  const int thr = EDGE_THRESHOLD;
  bool d[8];
  int dark = 0;
  for (int i = 0; i < 8; ++i) { d[i] = cal[i] > thr; if (d[i]) dark++; }
  const bool center = d[3] || d[4];
  const int l1 = workingSensor(0, 1), l2 = workingSensor(l1 + 1, 1);    // outermost working
  const int r1 = workingSensor(7, -1), r2 = workingSensor(r1 - 1, -1);
  const bool leftArm = d[l1] && d[l2] && center;
  const bool rightArm = d[r1] && d[r2] && center;
  const uint8_t confirm = params[P_JUNCTION_CONFIRM];
  const uint32_t now = millis();

  if (!jActive) {
    if (!leftArm && !rightArm) { jWideCount = 0; return; }
    if (jWideCount == 0) { jLeft = false; jRight = false; }
    jLeft = jLeft || leftArm;
    jRight = jRight || rightArm;
    if (++jWideCount < confirm) return;
    jActive = true;
    jLooking = false;
    jBoxReported = false;
    jClearCount = 0;
    jStartMs = now;
    jAllDarkSinceMs = 0;
    jIrL = irLeftActive;
    jIrR = irRightActive;
    Serial.printf("Junction start: %s%s\n", jLeft ? "LEFT " : "", jRight ? "RIGHT" : "");
    return;
  }

  jIrL = jIrL || irLeftActive;
  jIrR = jIrR || irRightActive;

  if (!jLooking) {
    // Crossing the side path(s): remember every side that had one.
    jLeft = jLeft || leftArm;
    jRight = jRight || rightArm;
    if (dark >= max(1, workingCount() - 1)) { if (jAllDarkSinceMs == 0) jAllDarkSinceMs = now; }
    else jAllDarkSinceMs = 0;
    if (!jBoxReported && jAllDarkSinceMs != 0 && now - jAllDarkSinceMs >= BOX_DETECT_MS) {
      jBoxReported = true;             // reported at once, while still inside the box
      reportJunction(JT_BOX, false);
    }
    const bool timedOut = now - jStartMs > JUNCTION_TIMEOUT_MS;
    const bool sideDark = d[l1] || d[r1];  // still on the side line
    if (sideDark && !timedOut) { jClearCount = 0; return; }
    if (!timedOut && ++jClearCount < confirm) return;
    if (jBoxReported) { resetJunction(); return; }
    jLooking = true;                   // side is clear: drive on a little
    jLookStartMs = now;
  }
  if (now - jLookStartMs < (uint32_t)params[P_LOOKAHEAD] * 10U) return;

  // Look ahead: does the line go on?
  jActive = false;
  jLooking = false;
  jWideCount = 0;
  const bool straight = center;
  bool left = jLeft, right = jRight;
  if (params[P_JUNCTION_USE_IR]) {
    const uint32_t since = jStartMs - JUNCTION_IR_EARLY_MS;
    const bool irL = jIrL || (int32_t)(irLeftRiseMs - since) >= 0;
    const bool irR = jIrR || (int32_t)(irRightRiseMs - since) >= 0;
    if (straight) {
      // A side path next to a straight line must be confirmed by that side's IR.
      left = left && irL;
      right = right && irR;
      if (!left && !right) { Serial.println("Side path not seen by IR: ignored (curve)"); return; }
    } else if (left && right) {
      // T needs both IRs; if only one saw a path it was a corner to that side.
      if (irL && !irR) right = false;
      else if (irR && !irL) left = false;
    }
  }
  uint8_t type;
  if (left && right) type = straight ? JT_CROSS : JT_T;
  else if (straight) type = left ? JT_LEFT_BRANCH : JT_RIGHT_BRANCH;
  else type = left ? JT_LEFT_CORNER : JT_RIGHT_CORNER;
  reportJunction(type, straight);
}

// Junctions (above) and gaps: line lost, then found again without a turn.
void updateLineEvents(const int *cal, int32_t total) {
  updateJunction(cal);
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
// Side IR (option): IR-R sees a line -> remember "right". When the QTR then
// loses the line, spin right (IR side has priority over the QTR edge). The
// spin always ends with the QTR as above, then PID continues. If the QTR
// keeps seeing the line straight on for P_IR_MEMORY, the IR side is forgotten.
// Returns true while turning (PID skipped this cycle).
bool edgeTurnRules(const int *cal, int32_t weighted, int32_t total) {
  (void)cal;
  const bool sharpOn = (ruleFlags & RULE_SHARP_TURN) != 0;
  const bool irOn = false;   // separate IR turn rule removed: the IR now only guides the sharp turn
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
      sharpIrEdge = 0;
      pidLast = false;   // no derivative kick when PID restarts
      iTerm = 0.0f;
      return false;
    }
    const int speedPct = pivotSource == SRC_IR ? params[P_IR_SPEED] : pivotSpeed;
    const int pwm = speedPct * 255 / 100;
    setMotors(pivotDir < 0 ? -pwm : pwm, pivotDir < 0 ? pwm : -pwm);
    return true;
  }

  // Edges are measured from the outermost WORKING sensor (one may be off).
  const bool leftEdge = seen && pos <= (int32_t)workingSensor(0, 1) * 1000 + zone;
  const bool rightEdge = seen && pos >= (int32_t)workingSensor(7, -1) * 1000 - zone;
  const bool centered = seen && abs((int)(pos - 3500)) < (int)(CENTERED_ERROR * 1000.0f);

  // Sharp turn memory: on which edge the line was last seen.
  if (sharpOn) {
    if (leftEdge) lastEdge = -1;
    else if (rightEdge) lastEdge = 1;
    else if (centered) lastEdge = 0;
    // Option: a side IR seeing a line also marks that side (the QTR still ends the spin).
    if (params[P_SHARP_USE_IR]) {
      const uint32_t nowMs = millis();
      if (leftRose && !rightRose) { sharpIrEdge = -1; sharpIrWhiteMs = 0; }
      else if (rightRose && !leftRose) { sharpIrEdge = 1; sharpIrWhiteMs = 0; }
      if (irLeftActive || irRightActive) {
        sharpIrWhiteMs = 0;
      } else if (sharpIrEdge != 0 && seen) {
        // The QTR still sees the line after the IR went white: it was a
        // side line next to a straight path; forget it after IR memory.
        if (sharpIrWhiteMs == 0) sharpIrWhiteMs = nowMs;
        if (nowMs - sharpIrWhiteMs >= (uint32_t)params[P_IR_MEMORY] * 10U) sharpIrEdge = 0;
      }
      if ((sharpIrEdge < 0 && rightEdge) || (sharpIrEdge > 0 && leftEdge)) sharpIrEdge = 0;
    } else {
      sharpIrEdge = 0;
    }
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
  } else if (!seen && sharpOn && (lastEdge != 0 || sharpIrEdge != 0)) {
    startPivot(sharpIrEdge != 0 ? sharpIrEdge : lastEdge, SRC_SHARP);   // IR side first, else QTR edge
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
    if (!sensorOn(s)) { calibrated[s] = 0; continue; }   // switched off in the app
    const int ch = qtrChannel(s);       // s = 0 is the robot's left side
    int raw = readRaw(QTR[ch]);
    int32_t span = black[ch] - white[ch]; // >= MIN_CAL_RANGE if calOk
    calibrated[s] = constrain((int32_t)(raw - white[ch]) * 1000 / span,
                              (int32_t)0, (int32_t)1000);
    if (lineInverted) calibrated[s] = 1000 - calibrated[s];  // white line on black
  }
  applySmoothing(calibrated);   // uneven floor: average bad readings out
  applyRelative(calibrated);    // uneven floor: line darker than the rest = black
  linePosition(calibrated, weighted, total);
  for (int s = 0; s < 8; ++s) lastProcessed[s] = calibrated[s];
  lastProcessedPos = total > 0 ? weighted * 1000 / total : -1;

  updateInversion(calibrated);
  updateLineEvents(calibrated, total);

  if (ruleFlags & RULE_ROUTE) {
    if (routeStep(calibrated, total)) return;   // route engine drives this cycle
    if (!robotOn) return;                       // route finished or stopped
    if (deadEndRule(weighted, total)) return;   // line ended
  } else {
    if (routeState != RS_FOLLOW && routeState != RS_JUNCTION) {
      if (routeStep(calibrated, total)) return; // U-turn after a dead end
      if (!robotOn) return;
    }
    if (edgeTurnRules(calibrated, weighted, total)) return;   // sharp / IR turn
    if (deadEndRule(weighted, total)) return;                 // line ended
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
