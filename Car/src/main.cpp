/*
 * Arduino Car Robot - Interactive Calibration + EEPROM + Friendly UI
 * Author: Dmytro Ageyev (updated by copilot)
 * Date: 2025-11-12
 *
 * - Input accepted from Bluetooth (BTSerial) and Serial.
 * - Output ONLY to Serial (Serial Monitor).
 * - Interactive calibration flow (CAL START / CAL_RUN / measurements / CAL SAVE).
 * - EEPROM persistence for calibration.
 * - Removed ultrasonic support; manual measurement used.
 * - Added startup instructions (friendly interface).
 * - Added command to restore factory calibration: "RESET CAL" (also "RESET_CAL").
 *
 * Summary of new user flows:
 * - On power-up the device prints a brief help / command summary to Serial.
 * - Use Serial or Bluetooth to send commands (Bluetooth receives but does not send).
 * - For auto-calibration: on Serial run "CAL START F <ms>" then trigger runs via BT (CAL_RUN BOTH/L/R),
 *   measure distance externally and type measurements into Serial when prompted, then "CAL SAVE".
 * - To restore factory defaults: type "RESET CAL" on Serial (or send that line via BT) — this sets
 *   coefficients to built-in defaults and saves them to EEPROM.
 *
 * Note: This file is an updated single-file example. For larger projects split into .h/.cpp modules.
 */

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

// ======================= CONFIG =======================
constexpr bool DEBUG = true;

// Motor Driver Pins
constexpr uint8_t MOTOR_LEFT_PWM   = 10;
constexpr uint8_t MOTOR_LEFT_DIR1  = 8;
constexpr uint8_t MOTOR_LEFT_DIR2  = 9;

constexpr uint8_t MOTOR_RIGHT_PWM  = 5;
constexpr uint8_t MOTOR_RIGHT_DIR1 = 7;
constexpr uint8_t MOTOR_RIGHT_DIR2 = 6;

// LED / Output pins
constexpr uint8_t LED_STATUS        = LED_BUILTIN;
constexpr uint8_t LED_HEADLIGHTS    = 13;
constexpr uint8_t LED_PARKING_FRONT = A1;
constexpr uint8_t LED_PARKING_REAR  = 11; // PWM
constexpr uint8_t LED_TURN_LEFT     = A2;
constexpr uint8_t LED_TURN_RIGHT    = A3;
constexpr uint8_t LED_REVERSE       = A5;

constexpr uint8_t BUZZER_PIN       = 4;

// Bluetooth pins (input-only)
constexpr uint8_t BT_RX_PIN = 2;
constexpr uint8_t BT_TX_PIN = 3;
SoftwareSerial BTSerial(BT_RX_PIN, BT_TX_PIN);

// ======================= CALIBRATION DEFAULTS & EEPROM =======================
constexpr float CAL_LEFT_FORWARD   = 1.00f;
constexpr float CAL_LEFT_BACKWARD  = 1.00f;
constexpr float CAL_RIGHT_FORWARD  = 1.00f;
constexpr float CAL_RIGHT_BACKWARD = 1.00f;

struct CalData {
  uint32_t magic;
  float lf;
  float lb;
  float rf;
  float rb;
};
constexpr uint32_t CAL_MAGIC = 0xCA11C0DE;
constexpr int EEPROM_ADDR = 0;

// ======================= CONSTANTS =======================
constexpr int MOTOR_SPEED_MAX = 255;
constexpr int MOTOR_SPEED_MIN = 0;
constexpr int MOTOR_SPEED_DEFAULT = 150;

constexpr unsigned long LOOP_DELAY = 10UL;             // ms
constexpr unsigned long SENSOR_READ_INTERVAL = 100UL;  // ms
constexpr unsigned long BRAKE_LIGHT_DURATION = 1000UL; // ms
constexpr unsigned long TURN_BLINK_INTERVAL = 500UL;   // ms
constexpr unsigned long ALARM_BLINK_INTERVAL = 350UL;  // ms
constexpr unsigned long ALARM_BEEP_INTERVAL = 700UL;   // ms

constexpr uint8_t LOW_PARKING = 32; // PWM low brightness

// ======================= TYPES =======================
struct Motor {
  uint8_t pwm;
  uint8_t dir1;
  uint8_t dir2;
  float forwardCoef;
  float backwardCoef;

  Motor(uint8_t pwmPin, uint8_t dirPin1, uint8_t dirPin2, float fCoef = 1.0f, float bCoef = 1.0f)
    : pwm(pwmPin), dir1(dirPin1), dir2(dirPin2), forwardCoef(fCoef), backwardCoef(bCoef) {}

  void begin() {
    pinMode(pwm, OUTPUT);
    pinMode(dir1, OUTPUT);
    pinMode(dir2, OUTPUT);
    stopRaw();
  }

  void stopRaw() {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
    analogWrite(pwm, 0);
  }

  void stop() { stopRaw(); }

  void setCalibration(float fCoef, float bCoef) {
    if (fCoef <= 0.0f) fCoef = 1.0f;
    if (bCoef <= 0.0f) bCoef = 1.0f;
    forwardCoef = fCoef;
    backwardCoef = bCoef;
  }

  void setSpeed(int speed) {
    if (speed >= 0) {
      float scaled = speed * forwardCoef;
      int s = constrain((int)roundf(scaled), 0, MOTOR_SPEED_MAX);
      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, LOW);
      analogWrite(pwm, s);
    } else {
      float scaled = (-speed) * backwardCoef;
      int s = constrain((int)roundf(scaled), 0, MOTOR_SPEED_MAX);
      digitalWrite(dir1, LOW);
      digitalWrite(dir2, HIGH);
      analogWrite(pwm, s);
    }
  }

  // Raw speed bypass calibration - used for measurement runs
  void setSpeedRaw(int speed) {
    if (speed >= 0) {
      int s = constrain(speed, 0, MOTOR_SPEED_MAX);
      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, LOW);
      analogWrite(pwm, s);
    } else {
      int s = constrain(-speed, 0, MOTOR_SPEED_MAX);
      digitalWrite(dir1, LOW);
      digitalWrite(dir2, HIGH);
      analogWrite(pwm, s);
    }
  }
};

struct LED {
  uint8_t pin;
  bool isPwm;
  void begin() const { pinMode(pin, OUTPUT); if (isPwm) analogWrite(pin, 0); else digitalWrite(pin, LOW); }
  void set(bool on) const { if (isPwm) analogWrite(pin, on ? 255 : 0); else digitalWrite(pin, on ? HIGH : LOW); }
  void writePWM(uint8_t v) const { if (isPwm) analogWrite(pin,v); else digitalWrite(pin, v?HIGH:LOW); }
};
Motor motorLeft(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR1, MOTOR_LEFT_DIR2);
Motor motorRight(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR1, MOTOR_RIGHT_DIR2);
// Motor motorLeft  = { MOTOR_LEFT_PWM, MOTOR_LEFT_DIR1, MOTOR_LEFT_DIR2, 1.0f, 1.0f};
// Motor motorRight = { MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR1, MOTOR_RIGHT_DIR2, 1.0f, 1.0f };

const LED ledStatus        = { LED_STATUS, false };
const LED ledHeadlights    = { LED_HEADLIGHTS, false };
const LED ledParkingFront  = { LED_PARKING_FRONT, false };
const LED ledParkingRear   = { LED_PARKING_REAR, true  };
const LED ledTurnLeft      = { LED_TURN_LEFT, false };
const LED ledTurnRight     = { LED_TURN_RIGHT, false };
const LED ledReverse       = { LED_REVERSE, false };
const LED buzzerOut        = { BUZZER_PIN, false  };

// State
unsigned long previousSensorMillis = 0;
unsigned long brakeStartMillis = 0;
unsigned long lastTurnBlinkMillis = 0;
unsigned long lastAlarmMillis = 0;
unsigned long lastAlarmBeepMillis = 0;

int currentSpeed = MOTOR_SPEED_DEFAULT;
bool systemEnabled = false;
bool headlightsOn = false;
bool parkingLightsOn = false;
bool parkingModeOn = false;
bool brakeActive = false;
bool hazardOn = false;
bool reverseActive = false;
bool leftTurnActive = false;
bool rightTurnActive = false;
bool turnBlinkState = false;
bool alarmOn = false;

// Horn
bool hornPlaying = false;
size_t hornIndex = 0;
unsigned long hornNoteStartMillis = 0;
const uint16_t HORN_NOTES[]     = { 523, 392, 330, 262, 392, 523 };
const uint16_t HORN_DURATIONS[] = { 160, 160, 200, 260, 180, 300 };
const size_t HORN_LENGTH = sizeof(HORN_NOTES) / sizeof(HORN_NOTES[0]);

// Windows startup melody (MS Windows XP startup sound approximation)
//const uint16_t WIN_STARTUP_NOTES[]     = { 1245, 622, 932, 880, 622, 1245, 932 }; // B4, D5, F#5, G#5
const uint16_t WIN_STARTUP_NOTES[]     = { 622, 311, 466, 440, 311, 622, 466 };
//const uint16_t WIN_STARTUP_DURATIONS[] = { 333, 167, 500, 333, 333, 333, 1000 };
const uint16_t WIN_STARTUP_DURATIONS[] = { 250, 125, 375, 250, 250, 250, 750 };
const size_t WIN_STARTUP_LENGTH = sizeof(WIN_STARTUP_NOTES) / sizeof(WIN_STARTUP_NOTES[0]);

// Command buffers
constexpr size_t CMD_BUF_SIZE = 120;
char btCmdBuf[CMD_BUF_SIZE]; size_t btCmdPos = 0;
char serCmdBuf[CMD_BUF_SIZE]; size_t serCmdPos = 0;

// ======================= CALIBRATION INTERACTIVE STATE =======================
enum CalDir { CAL_FORWARD, CAL_BACKWARD };
enum CalPhase {
  CAL_IDLE,
  CAL_WAIT_BOTH_RUN,
  CAL_WAIT_BOTH_MEASURE,
  CAL_WAIT_LEFT_RUN,
  CAL_WAIT_LEFT_MEASURE,
  CAL_WAIT_RIGHT_RUN,
  CAL_WAIT_RIGHT_MEASURE,
  CAL_APPLIED // applied in RAM, awaiting SAVE or CANCEL
};

CalPhase calPhase = CAL_IDLE;
CalDir calDir = CAL_FORWARD;
unsigned long calDurationMs = 0;
int calTestSpeed = 120; // default raw speed used for calibration runs
float measuredBoth = 0.0f;   // cm input by user
float measuredLeft = 0.0f;   // cm
float measuredRight = 0.0f;  // cm

// backup coefficients (if user cancels)
struct CalBackup { float lf, lb, rf, rb; } calBackup;

// ======================= DECLARATIONS =======================
void setupPins();
void initializeSystem();

// Motor helpers
void setBothMotors(int speed);
void stopMotors();
void forward(int speed);
void backward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void forwardLeft(int speed);
void forwardRight(int speed);
void backwardLeft(int speed);
void backwardRight(int speed);

// Motors raw run helper
void runMotorsRaw(int leftRaw, int rightRaw, unsigned long durationMs);

// Lighting / indicators
void setReverseIndicator(bool state);
void activateBrakeLight();
void updateBrakeLight();
void setHeadlights(bool state);
void setParkingLights(bool state);
void setParkingMode(bool state);
void setLeftTurn(bool state);
void setRightTurn(bool state);
void setHazard(bool state);
void updateTurnSignals();

// Alarm / horn
void setAlarm(bool state);
void updateAlarm();
void startHorn();
void updateHorn();

// Commands
void handleBluetoothInput();
void handleSerialInput();
void processCommandLine(const char *line);
void processSingleCharCommand(char c);

// EEPROM calibration
void loadCalibrationFromEEPROM();
void saveCalibrationToEEPROM();
void resetCalibrationToFactory();

// Calibration flow helpers
void calStart(CalDir dir, unsigned long durationMs);
void calAbort();
void calApplyAndCompute();
void calSave();
void calCancel();

// Utility
void blinkLED(const LED &l, int times, int delayMs);
void dbgPrint(const char *label, const String &value);
void printStatus();
void printStartupHelp();
void playWindowsStartup();

// ======================= EEPROM =======================
void loadCalibrationFromEEPROM() {
  CalData data;
  EEPROM.get(EEPROM_ADDR, data);
  if (data.magic == CAL_MAGIC) {
    motorLeft.setCalibration(data.lf, data.lb);
    motorRight.setCalibration(data.rf, data.rb);
    if (DEBUG) {
      Serial.println(F("Calibration loaded from EEPROM:"));
      Serial.print(F("  LF=")); Serial.println(data.lf);
      Serial.print(F("  LB=")); Serial.println(data.lb);
      Serial.print(F("  RF=")); Serial.println(data.rf);
      Serial.print(F("  RB=")); Serial.println(data.rb);
    }
  } else {
    motorLeft.setCalibration(CAL_LEFT_FORWARD, CAL_LEFT_BACKWARD);
    motorRight.setCalibration(CAL_RIGHT_FORWARD, CAL_RIGHT_BACKWARD);
    saveCalibrationToEEPROM();
    if (DEBUG) Serial.println(F("No EEPROM calibration found; using defaults and saving."));
  }
}

void saveCalibrationToEEPROM() {
  CalData data;
  data.magic = CAL_MAGIC;
  data.lf = motorLeft.forwardCoef;
  data.lb = motorLeft.backwardCoef;
  data.rf = motorRight.forwardCoef;
  data.rb = motorRight.backwardCoef;
  EEPROM.put(EEPROM_ADDR, data);
  if (DEBUG) {
    Serial.println(F("Calibration saved to EEPROM:"));
    Serial.print(F("  LF=")); Serial.println(data.lf);
    Serial.print(F("  LB=")); Serial.println(data.lb);
    Serial.print(F("  RF=")); Serial.println(data.rf);
    Serial.print(F("  RB=")); Serial.println(data.rb);
  }
}

void resetCalibrationToFactory() {
  motorLeft.setCalibration(CAL_LEFT_FORWARD, CAL_LEFT_BACKWARD);
  motorRight.setCalibration(CAL_RIGHT_FORWARD, CAL_RIGHT_BACKWARD);
  saveCalibrationToEEPROM();
  Serial.println(F("Factory calibration restored and saved to EEPROM."));
}

// ======================= SETUP =======================
void setup() {
  Serial.begin(9600);
  if (DEBUG) {
    Serial.println(F("================================="));
    Serial.println(F("Arduino Car Robot - Initializing"));
    Serial.println(F("================================="));
  }

  setupPins();
  initializeSystem();

  loadCalibrationFromEEPROM();

  BTSerial.begin(9600); // input-only; no BT outputs
  if (DEBUG) Serial.println(F("Bluetooth ready (9600)"));

  // Play Windows startup melody
  playWindowsStartup();

  blinkLED(ledStatus, 3, 200);
  if (DEBUG) Serial.println(F("System Ready!"));

  // Friendly startup instructions printed only to Serial
  printStartupHelp();
}

void setupPins() {
  motorLeft.begin();
  motorRight.begin();

  // LEDs / Outputs
  ledStatus.begin();
  ledHeadlights.begin();
  ledParkingFront.begin();
  ledParkingRear.begin();
  ledTurnLeft.begin();
  ledTurnRight.begin();
  ledReverse.begin();
  pinMode(BUZZER_PIN, OUTPUT);

  if (DEBUG) Serial.println(F("Pins configured successfully"));
}

void initializeSystem() {
  stopMotors();

  // Turn off LEDs
  ledStatus.set(false);
  ledHeadlights.set(false);
  ledParkingFront.set(false);
  ledParkingRear.writePWM(0);
  ledTurnLeft.set(false);
  ledTurnRight.set(false);
  ledReverse.set(false);

  buzzerOut.set(false);

  // reset states
  headlightsOn = false;
  parkingLightsOn = false;
  parkingModeOn = false;
  brakeActive = false;
  hazardOn = false;
  leftTurnActive = false;
  rightTurnActive = false;
  turnBlinkState = false;
  alarmOn = false;

  systemEnabled = true;

  if (DEBUG) Serial.println(F("System initialized"));
}

// ======================= MOTOR CONTROL =======================
void setBothMotors(int speed) {
  motorLeft.setSpeed(speed);
  motorRight.setSpeed(speed);
}

void stopMotors() {
  motorLeft.stop();
  motorRight.stop();
  activateBrakeLight();
}

void forward(int speed) {
  setBothMotors(constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX));
  setReverseIndicator(false);
}

void backward(int speed) {
  setBothMotors(-constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX));
  setReverseIndicator(true);
}

void turnLeft(int speed) {
  motorLeft.setSpeed(-constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX));
  motorRight.setSpeed(constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX));
  setLeftTurn(true); setRightTurn(false);
  setReverseIndicator(false);
}

void turnRight(int speed) {
  motorLeft.setSpeed(constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX));
  motorRight.setSpeed(-constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX));
  setLeftTurn(false); setRightTurn(true);
  setReverseIndicator(false);
}

void forwardLeft(int speed) {
  motorLeft.setSpeed((int)roundf(speed * 0.6f));
  motorRight.setSpeed(speed);
  setLeftTurn(true); setRightTurn(false);
  setReverseIndicator(false);
}

void forwardRight(int speed) {
  motorLeft.setSpeed(speed);
  motorRight.setSpeed((int)roundf(speed * 0.6f));
  setLeftTurn(false); setRightTurn(true);
  setReverseIndicator(false);
}

void backwardLeft(int speed) {
  motorLeft.setSpeed(-(int)roundf(speed * 0.6f));
  motorRight.setSpeed(-speed);
  setLeftTurn(true); setRightTurn(false);
  setReverseIndicator(true);
}

void backwardRight(int speed) {
  motorLeft.setSpeed(-speed);
  motorRight.setSpeed(-(int)roundf(speed * 0.6f));
  setLeftTurn(false); setRightTurn(true);
  setReverseIndicator(true);
}

// run motors raw for duration (blocking)
void runMotorsRaw(int leftRaw, int rightRaw, unsigned long durationMs) {
  motorLeft.setSpeedRaw(leftRaw);
  motorRight.setSpeedRaw(rightRaw);
  unsigned long start = millis();
  while (millis() - start < durationMs) {
    // allow USB/serial handling
    delay(5);
  }
  motorLeft.stopRaw();
  motorRight.stopRaw();
}

// ======================= LIGHTS / INDICATORS =======================
void setReverseIndicator(bool state) {
  reverseActive = state;
  ledReverse.set(state);
}

void activateBrakeLight() {
  if (parkingModeOn) return;
  brakeActive = true;
  brakeStartMillis = millis();
  ledParkingRear.writePWM(255);
}

void updateBrakeLight() {
  if (!brakeActive) return;
  if (millis() - brakeStartMillis >= BRAKE_LIGHT_DURATION) {
    brakeActive = false;
    if (parkingModeOn) return;
    if (parkingLightsOn) ledParkingRear.writePWM(LOW_PARKING);
    else ledParkingRear.writePWM(0);
  }
}

void setHeadlights(bool state) {
  headlightsOn = state;
  ledHeadlights.set(state);
  if (DEBUG) dbgPrint("Headlights", String(state ? "ON" : "OFF"));
}

void setParkingLights(bool state) {
  parkingLightsOn = state;
  ledParkingFront.set(state);
  if (state) ledParkingRear.writePWM(LOW_PARKING);
  else if (!brakeActive) ledParkingRear.writePWM(0);
  if (DEBUG) dbgPrint("Parking", String(state ? "ON" : "OFF"));
}

void setParkingMode(bool state) {
  parkingModeOn = state;
  if (state) {
    stopMotors();
    ledParkingRear.writePWM(255);
    if (DEBUG) Serial.println(F("Parking mode: ON (motors locked)"));
  } else {
    if (parkingLightsOn) ledParkingRear.writePWM(LOW_PARKING);
    else ledParkingRear.writePWM(0);
    if (DEBUG) Serial.println(F("Parking mode: OFF"));
  }
}

void setLeftTurn(bool state) {
  leftTurnActive = state;
  if (!state) ledTurnLeft.set(false);
}

void setRightTurn(bool state) {
  rightTurnActive = state;
  if (!state) ledTurnRight.set(false);
}

void setHazard(bool state) {
  hazardOn = state;
  if (!state) {
    ledTurnLeft.set(false);
    ledTurnRight.set(false);
  }
}

void updateTurnSignals() {
  unsigned long now = millis();
  if (hazardOn || leftTurnActive || rightTurnActive) {
    if (now - lastTurnBlinkMillis >= TURN_BLINK_INTERVAL) {
      lastTurnBlinkMillis = now;
      turnBlinkState = !turnBlinkState;
    }
    if (hazardOn) {
      ledTurnLeft.set(turnBlinkState);
      ledTurnRight.set(turnBlinkState);
    } else {
      ledTurnLeft.set(leftTurnActive ? turnBlinkState : false);
      ledTurnRight.set(rightTurnActive ? turnBlinkState : false);
    }
  } else {
    ledTurnLeft.set(false);
    ledTurnRight.set(false);
  }
}

// ======================= ALARM / HORN =======================
void setAlarm(bool state) {
  alarmOn = state;
  if (!state) noTone(BUZZER_PIN);
}

void updateAlarm() {
  if (!alarmOn || hornPlaying) return;
  unsigned long now = millis();
  if (now - lastAlarmMillis >= ALARM_BLINK_INTERVAL) {
    lastAlarmMillis = now;
    static bool alarmBlink = false;
    alarmBlink = !alarmBlink;
    ledHeadlights.set(alarmBlink);
    ledTurnLeft.set(alarmBlink);
    ledTurnRight.set(alarmBlink);
  }
  if (now - lastAlarmBeepMillis >= ALARM_BEEP_INTERVAL) {
    lastAlarmBeepMillis = now;
    tone(BUZZER_PIN, 1200, 120);
  }
}

void startHorn() {
  if (hornPlaying) return;
  hornPlaying = true;
  hornIndex = 0;
  hornNoteStartMillis = millis();
  tone(BUZZER_PIN, HORN_NOTES[hornIndex], HORN_DURATIONS[hornIndex]);
}

void updateHorn() {
  if (!hornPlaying) return;
  unsigned long now = millis();
  if (now - hornNoteStartMillis >= (unsigned long)HORN_DURATIONS[hornIndex] + 40) {
    hornIndex++;
    if (hornIndex >= HORN_LENGTH) {
      hornPlaying = false;
      noTone(BUZZER_PIN);
      return;
    }
    hornNoteStartMillis = now;
    tone(BUZZER_PIN, HORN_NOTES[hornIndex], HORN_DURATIONS[hornIndex]);
  }
}

// ======================= INPUT HANDLING =======================
// Bluetooth line parser (input-only)
void handleBluetoothInput() {
  while (BTSerial.available() > 0) {
    char c = (char)BTSerial.read();
    if (c == '\r') continue;
    if (c == '\n' || btCmdPos + 1 >= CMD_BUF_SIZE) {
      btCmdBuf[btCmdPos] = '\0';
      if (btCmdPos > 0) {
        processCommandLine(btCmdBuf);
      }
      btCmdPos = 0;
    } else {
      btCmdBuf[btCmdPos++] = c;
    }
  }
}

// Serial line parser (commands & calibration measurements / control)
void handleSerialInput() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n' || serCmdPos + 1 >= CMD_BUF_SIZE) {
      serCmdBuf[serCmdPos] = '\0';
      if (serCmdPos > 0) {
        processCommandLine(serCmdBuf);
      }
      serCmdPos = 0;
    } else {
      serCmdBuf[serCmdPos++] = c;
    }
  }
}

// Single-char compatibility (from Serial keyboard or other)
void processSingleCharCommand(char c) {
  if (c == 'u') { setHeadlights(false); return; }
  if (c == 'v') { setParkingLights(false); return; }
  if (c == 'w') { setParkingMode(false); return; }
  if (c == 'x') { setHazard(false); return; }
  if (c == 'z') { setAlarm(false); return; }

  c = toupper(static_cast<unsigned char>(c));
  switch (c) {
    case 'F': if (systemEnabled && !parkingModeOn) forward(currentSpeed); break;
    case 'B': if (systemEnabled && !parkingModeOn) backward(currentSpeed); break;
    case 'L': if (systemEnabled && !parkingModeOn) turnLeft(currentSpeed); break;
    case 'R': if (systemEnabled && !parkingModeOn) turnRight(currentSpeed); break;
    case 'G': if (systemEnabled && !parkingModeOn) forwardLeft(currentSpeed); break;
    case 'H': if (systemEnabled && !parkingModeOn) forwardRight(currentSpeed); break;
    case 'I': if (systemEnabled && !parkingModeOn) backwardLeft(currentSpeed); break;
    case 'J': if (systemEnabled && !parkingModeOn) backwardRight(currentSpeed); break;
    case 'U': setHeadlights(true); break;
    case 'V': setParkingLights(true); break;
    case 'X': setHazard(true); break;
    case 'Z': setAlarm(!alarmOn); break;
    case 'Y': startHorn(); break;
    case 'W': setParkingMode(true); break;
    case 'S': if (!parkingModeOn) stopMotors(); setLeftTurn(false); setRightTurn(false); break;
    case 'E':
      systemEnabled = !systemEnabled;
      if (!systemEnabled) stopMotors();
      if (!systemEnabled) setReverseIndicator(false);
      Serial.print(F("EN:")); Serial.println(systemEnabled ? 1 : 0);
      break;
    case 'Q':
      printStatus();
      break;
    default:
      if (c >= '0' && c <= '9') {
        int level = c - '0';
        int newSpeed = map(level, 0, 9, 0, MOTOR_SPEED_MAX);
        currentSpeed = newSpeed;
        Serial.print(F("SPD:")); Serial.println(currentSpeed);
      }
      break;
  }
}

// ======================= COMMAND LINE PROCESSING =======================
void processCommandLine(const char *line) {
  if (!line || *line == '\0') return;
  if (DEBUG) {
    Serial.print(F("CMD> "));
    Serial.println(line);
  }

  // If calibration expects a numeric measurement, handle that first:
  if (calPhase == CAL_WAIT_BOTH_MEASURE || calPhase == CAL_WAIT_LEFT_MEASURE || calPhase == CAL_WAIT_RIGHT_MEASURE) {
    float val = atof(line);
    if (val <= 0.0f) {
      Serial.println(F("Invalid measurement. Enter positive number (cm)."));
      return;
    }
    if (calPhase == CAL_WAIT_BOTH_MEASURE) {
      measuredBoth = val;
      Serial.print(F("Measured BOTH distance = ")); Serial.print(measuredBoth); Serial.println(F(" cm"));
      calPhase = CAL_WAIT_LEFT_RUN;
      Serial.println(F("Now trigger left-only run from Bluetooth or Serial by sending: CAL_RUN L"));
      return;
    } else if (calPhase == CAL_WAIT_LEFT_MEASURE) {
      measuredLeft = val;
      Serial.print(F("Measured LEFT distance = ")); Serial.print(measuredLeft); Serial.println(F(" cm"));
      calPhase = CAL_WAIT_RIGHT_RUN;
      Serial.println(F("Now trigger right-only run from Bluetooth or Serial by sending: CAL_RUN R"));
      return;
    } else if (calPhase == CAL_WAIT_RIGHT_MEASURE) {
      measuredRight = val;
      Serial.print(F("Measured RIGHT distance = ")); Serial.print(measuredRight); Serial.println(F(" cm"));
      calApplyAndCompute();
      return;
    }
  }

  // tokenize line
  char buf[CMD_BUF_SIZE];
  strncpy(buf, line, CMD_BUF_SIZE);
  buf[CMD_BUF_SIZE - 1] = '\0';
  char *saveptr = nullptr;
  char *tok = strtok_r(buf, " ", &saveptr);
  if (!tok) return;

  // HELP
  if (strcasecmp(tok, "HELP") == 0 || strcmp(tok, "?") == 0) {
    printStartupHelp();
    return;
  }

  // RESET CAL command (factory restore)
  if (strcasecmp(tok, "RESET") == 0) {
    char *sub = strtok_r(NULL, " ", &saveptr);
    if (sub && (strcasecmp(sub, "CAL") == 0 || strcasecmp(sub, "CAL\n") == 0)) {
      resetCalibrationToFactory();
      return;
    } else {
      Serial.println(F("Usage: RESET CAL  (restores factory calibration and saves to EEPROM)"));
      return;
    }
  }

  // top-level commands
  if (strcasecmp(tok, "CAL?") == 0) {
    Serial.println(F("Calibration (RAM):"));
    Serial.print(F("  LF=")); Serial.println(motorLeft.forwardCoef);
    Serial.print(F("  LB=")); Serial.println(motorLeft.backwardCoef);
    Serial.print(F("  RF=")); Serial.println(motorRight.forwardCoef);
    Serial.print(F("  RB=")); Serial.println(motorRight.backwardCoef);
    return;
  }

  if (strcasecmp(tok, "STATUS") == 0) {
    printStatus();
    return;
  }

  if (strcasecmp(tok, "CAL") == 0) {
    char *sub = strtok_r(NULL, " ", &saveptr);
    if (!sub) {
      Serial.println(F("CAL commands: START F|B <ms> | SAVE | CANCEL | ABORT | L|R F|B <value> | ?"));
      return;
    }
    if (strcasecmp(sub, "START") == 0) {
      char *dirTok = strtok_r(NULL, " ", &saveptr); // F or B
      char *durTok = strtok_r(NULL, " ", &saveptr); // duration ms
      if (!dirTok || !durTok) {
        Serial.println(F("Usage: CAL START F|B <duration_ms>"));
        return;
      }
      unsigned long dur = strtoul(durTok, NULL, 10);
      if (strcasecmp(dirTok, "F") == 0) calStart(CAL_FORWARD, dur);
      else if (strcasecmp(dirTok, "B") == 0) calStart(CAL_BACKWARD, dur);
      else Serial.println(F("CAL START direction must be F or B"));
      return;
    } else if (strcasecmp(sub, "SAVE") == 0) {
      calSave();
      return;
    } else if (strcasecmp(sub, "CANCEL") == 0 || strcasecmp(sub, "ABORT") == 0) {
      calCancel();
      return;
    } else {
      // manual coefficient set: CAL L F 0.95
      char *side = sub;
      char *dir = strtok_r(NULL, " ", &saveptr);
      char *valTok = strtok_r(NULL, " ", &saveptr);
      if (!dir || !valTok) {
        Serial.println(F("CAL usage: CAL L|R F|B <value> OR CAL START ... OR CAL SAVE/CANCEL"));
        return;
      }
      float v = atof(valTok);
      if (v <= 0.0f) {
        Serial.println(F("Invalid coefficient value (must be > 0)"));
        return;
      }
      if (strcasecmp(side, "L") == 0) {
        if (strcasecmp(dir, "F") == 0) motorLeft.forwardCoef = v;
        else if (strcasecmp(dir, "B") == 0) motorLeft.backwardCoef = v;
        else { Serial.println(F("Direction must be F or B")); return; }
      } else if (strcasecmp(side, "R") == 0) {
        if (strcasecmp(dir, "F") == 0) motorRight.forwardCoef = v;
        else if (strcasecmp(dir, "B") == 0) motorRight.backwardCoef = v;
        else { Serial.println(F("Direction must be F or B")); return; }
      } else {
        Serial.println(F("Side must be L or R"));
        return;
      }
      Serial.println(F("Calibration updated in RAM. Use CAL SAVE to persist to EEPROM."));
      return;
    }
  }

  // CAL_RUN commands (can arrive via BT or Serial)
  if (strcasecmp(tok, "CAL_RUN") == 0) {
    char *which = strtok_r(NULL, " ", &saveptr); // BOTH | L | R
    if (!which) {
      Serial.println(F("CAL_RUN usage: CAL_RUN BOTH|L|R (send via BT or Serial to trigger movement)"));
      return;
    }
    if (calPhase == CAL_IDLE) {
      Serial.println(F("No calibration session active. Start with: CAL START F|B <ms>"));
      return;
    }
    // Trigger the run matching current phase
    if (strcasecmp(which, "BOTH") == 0) {
      if (calPhase != CAL_WAIT_BOTH_RUN) {
        Serial.println(F("Unexpected CAL_RUN BOTH; not in BOTH_RUN phase."));
        return;
      }
      Serial.println(F("Running BOTH motors (raw) for configured duration..."));
      int s = calTestSpeed;
      if (calDir == CAL_BACKWARD) s = -s;
      runMotorsRaw(s, s, calDurationMs);
      Serial.println(F("Run complete. Enter measured distance (cm) on Serial:"));
      calPhase = CAL_WAIT_BOTH_MEASURE;
      return;
    } else if (strcasecmp(which, "L") == 0) {
      if (calPhase != CAL_WAIT_LEFT_RUN) {
        Serial.println(F("Unexpected CAL_RUN L; not in LEFT_RUN phase."));
        return;
      }
      Serial.println(F("Running LEFT motor only (raw) for configured duration..."));
      int s = calTestSpeed;
      if (calDir == CAL_BACKWARD) s = -s;
      runMotorsRaw(s, 0, calDurationMs);
      Serial.println(F("Run complete. Enter measured distance (cm) on Serial:"));
      calPhase = CAL_WAIT_LEFT_MEASURE;
      return;
    } else if (strcasecmp(which, "R") == 0) {
      if (calPhase != CAL_WAIT_RIGHT_RUN) {
        Serial.println(F("Unexpected CAL_RUN R; not in RIGHT_RUN phase."));
        return;
      }
      Serial.println(F("Running RIGHT motor only (raw) for configured duration..."));
      int s = calTestSpeed;
      if (calDir == CAL_BACKWARD) s = -s;
      runMotorsRaw(0, s, calDurationMs);
      Serial.println(F("Run complete. Enter measured distance (cm) on Serial:"));
      calPhase = CAL_WAIT_RIGHT_MEASURE;
      return;
    } else {
      Serial.println(F("CAL_RUN argument must be BOTH, L or R"));
      return;
    }
  }

  // If single-char command line like "F" or "S", handle via single-char processor
  if (strlen(line) == 1) {
    processSingleCharCommand(line[0]);
    return;
  }

  Serial.println(F("Unknown command. Type HELP for usage."));
}

// ======================= CALIBRATION HELPERS =======================
void calStart(CalDir dir, unsigned long durationMs) {
  if (calPhase != CAL_IDLE && calPhase != CAL_APPLIED) {
    Serial.println(F("Calibration already in progress. Use CAL ABORT to cancel."));
    return;
  }
  // backup current coeffs
  calBackup.lf = motorLeft.forwardCoef;
  calBackup.lb = motorLeft.backwardCoef;
  calBackup.rf = motorRight.forwardCoef;
  calBackup.rb = motorRight.backwardCoef;

  calDir = dir;
  calDurationMs = durationMs;
  calTestSpeed = currentSpeed > 0 ? currentSpeed : 120;
  measuredBoth = measuredLeft = measuredRight = 0.0f;

  calPhase = CAL_WAIT_BOTH_RUN;
  Serial.print(F("Calibration started ("));
  Serial.print(dir == CAL_FORWARD ? F("FORWARD") : F("BACKWARD"));
  Serial.print(F(", duration_ms=")); Serial.print(calDurationMs);
  Serial.println(F(")"));
  Serial.println(F("Trigger BOTH run from Bluetooth or Serial: CAL_RUN BOTH"));
  Serial.println(F("After run, enter measured distance (cm) on Serial."));
}

void calAbort() {
  // revert
  motorLeft.forwardCoef = calBackup.lf;
  motorLeft.backwardCoef = calBackup.lb;
  motorRight.forwardCoef = calBackup.rf;
  motorRight.backwardCoef = calBackup.rb;
  calPhase = CAL_IDLE;
  Serial.println(F("Calibration aborted. Restored previous coefficients."));
}

void calApplyAndCompute() {
  if (measuredBoth <= 0.0f || measuredLeft <= 0.0f || measuredRight <= 0.0f) {
    Serial.println(F("Measurements incomplete or invalid. Cannot compute coefficients."));
    calPhase = CAL_IDLE;
    return;
  }
  float target = measuredBoth / 2.0f;
  float factorL = (measuredLeft > 0.0f) ? (target / measuredLeft) : 1.0f;
  float factorR = (measuredRight > 0.0f) ? (target / measuredRight) : 1.0f;
  if (!isfinite(factorL) || factorL <= 0.0f) factorL = 1.0f;
  if (!isfinite(factorR) || factorR <= 0.0f) factorR = 1.0f;

  if (calDir == CAL_FORWARD) {
    motorLeft.forwardCoef *= factorL;
    motorRight.forwardCoef *= factorR;
  } else {
    motorLeft.backwardCoef *= factorL;
    motorRight.backwardCoef *= factorR;
  }

  calPhase = CAL_APPLIED;
  Serial.println(F("Calibration applied in RAM. New coefficients:"));
  Serial.print(F("  LF=")); Serial.println(motorLeft.forwardCoef);
  Serial.print(F("  LB=")); Serial.println(motorLeft.backwardCoef);
  Serial.print(F("  RF=")); Serial.println(motorRight.forwardCoef);
  Serial.print(F("  RB=")); Serial.println(motorRight.backwardCoef);
  Serial.println(F("Test the robot using Bluetooth. If OK, enter on Serial: CAL SAVE"));
  Serial.println(F("To discard changes and restore previous values enter: CAL CANCEL"));
}

void calSave() {
  if (calPhase != CAL_APPLIED) {
    Serial.println(F("No new calibration to save (use CAL START ... first)."));
    return;
  }
  saveCalibrationToEEPROM();
  calPhase = CAL_IDLE;
  Serial.println(F("Calibration saved to EEPROM."));
}

void calCancel() {
  motorLeft.forwardCoef = calBackup.lf;
  motorLeft.backwardCoef = calBackup.lb;
  motorRight.forwardCoef = calBackup.rf;
  motorRight.backwardCoef = calBackup.rb;
  calPhase = CAL_IDLE;
  Serial.println(F("Calibration canceled. Restored saved coefficients."));
}

// ======================= UTIL =======================
void blinkLED(const LED &l, int times, int delayMs) {
  for (int i = 0; i < times; ++i) {
    l.set(true); delay(delayMs);
    l.set(false); delay(delayMs);
  }
}

void dbgPrint(const char *label, const String &value) {
  if (!DEBUG) return;
  Serial.print(label); Serial.print(F(": ")); Serial.println(value);
}

void printStatus() {
  Serial.println(F("STATUS:"));
  Serial.print(F("  SPD: ")); Serial.println(currentSpeed);
  Serial.print(F("  ENABLED: ")); Serial.println(systemEnabled ? F("1") : F("0"));
  Serial.print(F("  HEADLIGHTS: ")); Serial.println(headlightsOn ? F("ON") : F("OFF"));
  Serial.print(F("  PARKING MODE: ")); Serial.println(parkingModeOn ? F("ON") : F("OFF"));
  Serial.println(F("  Calibration:"));
  Serial.print(F("    LF=")); Serial.println(motorLeft.forwardCoef);
  Serial.print(F("    LB=")); Serial.println(motorLeft.backwardCoef);
  Serial.print(F("    RF=")); Serial.println(motorRight.forwardCoef);
  Serial.print(F("    RB=")); Serial.println(motorRight.backwardCoef);
  Serial.print(F("  CAL Phase: "));
  switch (calPhase) {
    case CAL_IDLE: Serial.println(F("IDLE")); break;
    case CAL_WAIT_BOTH_RUN: Serial.println(F("WAIT_BOTH_RUN")); break;
    case CAL_WAIT_BOTH_MEASURE: Serial.println(F("WAIT_BOTH_MEASURE")); break;
    case CAL_WAIT_LEFT_RUN: Serial.println(F("WAIT_LEFT_RUN")); break;
    case CAL_WAIT_LEFT_MEASURE: Serial.println(F("WAIT_LEFT_MEASURE")); break;
    case CAL_WAIT_RIGHT_RUN: Serial.println(F("WAIT_RIGHT_RUN")); break;
    case CAL_WAIT_RIGHT_MEASURE: Serial.println(F("WAIT_RIGHT_MEASURE")); break;
    case CAL_APPLIED: Serial.println(F("APPLIED (await SAVE/CANCEL)")); break;
    default: Serial.println(F("UNKNOWN")); break;
  }
}

void printStartupHelp() {
  Serial.println(F(""));
  Serial.println(F("=== Car Robot - Quick Help ==="));
  Serial.println(F("Input via Bluetooth (BT) or Serial. Output only to Serial (monitor)."));
  Serial.println(F("Basic single-char commands (send via BT or Serial):"));
  Serial.println(F("  F - forward, B - back, L - turn left, R - turn right, S - stop"));
  Serial.println(F("  0..9 - set speed level (mapped to PWM)"));
  Serial.println(F("Calibration commands (interactive):"));
  Serial.println(F("  CAL?                 - show current calibration (RAM)"));
  Serial.println(F("  CAL START F|B <ms>   - start interactive calibration for FORWARD/BACKWARD"));
  Serial.println(F("  CAL_RUN BOTH|L|R     - trigger movement run (send via BT or Serial)"));
  Serial.println(F("  After a run completes, measure distance (cm) externally and type number in Serial."));
  Serial.println(F("  CAL SAVE             - save applied calibration to EEPROM"));
  Serial.println(F("  CAL CANCEL           - cancel calibration (restore previous values)"));
  Serial.println(F("  CAL L F|B <value>    - set coefficient manually in RAM"));
  Serial.println(F("Factory restore:"));
  Serial.println(F("  RESET CAL            - restore built-in factory calibration and save to EEPROM"));
  Serial.println(F("Utilities:"));
  Serial.println(F("  STATUS               - print status and current calibration"));
  Serial.println(F("  HELP or ?            - print this help"));
  Serial.println(F("=============================="));
  Serial.println(F(""));
}

void playWindowsStartup() {
  // Play Windows XP-style startup sound (blocking)
  for (size_t i = 0; i < WIN_STARTUP_LENGTH; i++) {
    tone(BUZZER_PIN, WIN_STARTUP_NOTES[i], WIN_STARTUP_DURATIONS[i]);
    delay(WIN_STARTUP_DURATIONS[i] ); // small gap between notes
  }
  noTone(BUZZER_PIN);
  delay(100); // brief pause after melody
}

// ======================= MAIN LOOP =======================
void loop() {
  unsigned long now = millis();

  // handle inputs
  handleBluetoothInput();
  handleSerialInput();

  // update state machines
  updateBrakeLight();
  updateTurnSignals();
  updateAlarm();
  updateHorn();

  delay(LOOP_DELAY);
}