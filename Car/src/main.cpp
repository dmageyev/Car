/*
 * Arduino Project Template
 * 
 * Project: Car Robot
 * Author: Dmytro Ageyev
 * Date: November 2, 2025
 * Description: Template for Arduino-based car robot project
 */

#include <Arduino.h>
#include <SoftwareSerial.h>

// ============================================
// PIN DEFINITIONS
// ============================================


// Motor Driver Pins (Example: L298P or similar)
#define MOTOR_LEFT_PWM    10    // PWM pin for left motor speed control
#define MOTOR_LEFT_DIR1   8    // Direction pin 1 for left motor
#define MOTOR_LEFT_DIR2   9    // Direction pin 2 for left motor

#define MOTOR_RIGHT_PWM  5    // PWM pin for right motor speed control
#define MOTOR_RIGHT_DIR1  7   // Direction pin 1 for right motor
#define MOTOR_RIGHT_DIR2  6   // Direction pin 2 for right motor
// Sensor Pins

// Ultrasonic Sensor Pins (Example: HC-SR04)
#ifdef ULTRASONIC_SENSOR_ENABLED
#define ULTRASONIC_TRIG   12   // Ultrasonic sensor trigger pin
#define ULTRASONIC_ECHO   13   // Ultrasonic sensor echo pin
#endif

// IR Sensor Pins (digital sensors)
#ifdef IR_SENSOR_ENABLED
// Toggle central IR sensor: 1 = enabled, 0 = disabled
#ifndef IR_SENSOR_CENTER_ENABLED
#define IR_SENSOR_CENTER_ENABLED 0
#endif

#define IR_SENSOR_LEFT     A4   // Left IR sensor (digital)
#if IR_SENSOR_CENTER_ENABLED
#define IR_SENSOR_CENTER   12   // Central IR sensor (digital)
#endif
#define IR_SENSOR_RIGHT    A0   // Right IR sensor moved to A4 to free D4 for buzzer
#endif

// LED Indicators
#define LED_STATUS        LED_BUILTIN  // Built-in LED for status indication
#define LED_HEADLIGHTS    13           // Headlights (shared with built-in LED)
#define LED_PARKING_FRONT A1           // Front parking lights
#define LED_PARKING_REAR  11           // Rear parking lights + brake light (PWM)
#define LED_TURN_LEFT     A2           // Left turn signal
#define LED_TURN_RIGHT    A3           // Right turn signal
#define LED_REVERSE       A5           // Reverse indicator (note: A7 is analog-only on classic Nano)


// Additional Pins (customize as needed)

// Bluetooth (HC-05/HC-06) pins
#define BT_RX_PIN          2    // Arduino RX (to BT TX)
#define BT_TX_PIN          3    // Arduino TX (to BT RX)

// Buzzer pin for horn/alarm
#define BUZZER_PIN         4    // Buzzer (horn + alarm) on digital pin D4
// Horn (melody) state
bool hornPlaying = false;
size_t hornIndex = 0;
unsigned long hornNoteStartMillis = 0;

// ============================================
// CONSTANTS
// ============================================

// Motor Speed Constants
#define MOTOR_SPEED_MAX   255  // Maximum PWM value
#define MOTOR_SPEED_MIN   0    // Minimum PWM value
#define MOTOR_SPEED_DEFAULT 150 // Default driving speed

// Sensor Thresholds
#define DISTANCE_THRESHOLD 20  // Distance threshold in cm for obstacle detection

// Timing Constants
#define LOOP_DELAY        10   // Main loop delay in milliseconds
#define SENSOR_READ_INTERVAL 100 // Sensor reading interval in ms
#define BRAKE_LIGHT_DURATION 1000 // Brake light duration in ms
#define TURN_BLINK_INTERVAL 500   // Turn/hazard blink interval in ms
#define ALARM_BLINK_INTERVAL 350  // Alarm blink interval for LEDs
#define ALARM_BEEP_INTERVAL 700   // Alarm beep interval for buzzer

#define LOW_PARKING 32 // Low brightness for parking lights (approx 12.5% of 255)

// ============================================
// GLOBAL VARIABLES
// ============================================

unsigned long previousMillis = 0;
unsigned long brakeStartMillis = 0;
unsigned long lastTurnBlinkMillis = 0;
unsigned long lastAlarmMillis = 0;
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
SoftwareSerial BTSerial(BT_RX_PIN, BT_TX_PIN);

void setReverseIndicator(bool state);
// ============================================
// FUNCTION DECLARATIONS
// ============================================

void setupPins();
void initializeSystem();

// Motor Control Functions
void motorStop();
void motorForward(int speed);
void motorBackward(int speed);
void motorTurnLeft(int speed);
void motorTurnRight(int speed);
void setMotorSpeed(int leftSpeed, int rightSpeed);

// Diagonal movement functions
void motorForwardLeft(int speed);
void motorForwardRight(int speed);
void motorBackwardLeft(int speed);
void motorBackwardRight(int speed);

// Sensor Functions
float readUltrasonicDistance();
int readIRSensor(int sensorPin);
void handleBluetoothInput();
void processBTCommand(char c);

// Utility Functions
void blinkLED(int pin, int times, int delayMs);
void setHeadlights(bool state);
void setParkingLights(bool state);
void setParkingMode(bool state);
void activateBrakeLight();
void updateBrakeLight();
void setHazard(bool state);
void updateTurnSignals();
void setLeftTurn(bool state);
void setRightTurn(bool state);
void setAlarm(bool state);
void startHorn();
void updateHorn();
void updateAlarm();

// ============================================
// SETUP FUNCTION
// ============================================

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  Serial.println("=================================");
  Serial.println("Arduino Car Robot - Initializing");
  Serial.println("=================================");
  
  // Configure pins
  setupPins();
  
  // Initialize system
  initializeSystem();

  // Initialize Bluetooth
  BTSerial.begin(9600);
  Serial.println("Bluetooth ready (9600)");
  
  // System ready indication
  blinkLED(LED_STATUS, 3, 300);
  Serial.println("System Ready!");
}

/**
 * Reverse indicator control
 */
void setReverseIndicator(bool state) {
  reverseActive = state;
  digitalWrite(LED_REVERSE, state ? HIGH : LOW);
}
// ============================================
// MAIN LOOP FUNCTION
// ============================================

void loop() {
  unsigned long currentMillis = millis();
  
  // Main program logic goes here
  
  // Example: Read sensors periodically
    if (currentMillis - previousMillis >= SENSOR_READ_INTERVAL) {
      previousMillis = currentMillis;
      
  #ifdef ULTRASONIC_SENSOR_ENABLED
      // Read distance sensor
      float distance = readUltrasonicDistance();
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
  #endif
      
  #ifdef IR_SENSOR_ENABLED
      // Read IR sensors (digital 0/1)
      int irLeft = readIRSensor(IR_SENSOR_LEFT);
      int irRight = readIRSensor(IR_SENSOR_RIGHT);
      Serial.print("IR Left: ");
      Serial.print(irLeft);
      Serial.print(" | IR Right: ");
      Serial.println(irRight);
  #endif
    }
  
  // Bluetooth command handling
  handleBluetoothInput();
  
  // Update timers/state machines
  updateBrakeLight();
  updateTurnSignals();
  updateAlarm();
  updateHorn();
  
  // Add your main control logic here
  
  delay(LOOP_DELAY);
}

// ============================================
// FUNCTION IMPLEMENTATIONS
// ============================================

/**
 * Configure all pins as INPUT or OUTPUT
 */
void setupPins() {
  // Motor pins
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);
  
  // Sensor pins
#ifdef ULTRASONIC_SENSOR_ENABLED
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
#endif

#ifdef IR_SENSOR_ENABLED
  pinMode(IR_SENSOR_LEFT, INPUT);
  #if IR_SENSOR_CENTER_ENABLED
  pinMode(IR_SENSOR_CENTER, INPUT);
  #endif
  pinMode(IR_SENSOR_RIGHT, INPUT);
#endif
  
  // LED pins
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_HEADLIGHTS, OUTPUT);
  pinMode(LED_PARKING_FRONT, OUTPUT);
  pinMode(LED_PARKING_REAR, OUTPUT);
  pinMode(LED_TURN_LEFT, OUTPUT);
  pinMode(LED_TURN_RIGHT, OUTPUT);
  pinMode(LED_REVERSE, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PARKING_FRONT, OUTPUT);
 
  
  // Additional pins removed: buzzer and button support
  
  Serial.println("Pins configured successfully");
}

/**
 * Initialize system components
 */
void initializeSystem() {
  // Stop all motors
  motorStop();
  
  // Turn off all LEDs
  digitalWrite(LED_STATUS, LOW);
  digitalWrite(LED_HEADLIGHTS, LOW);
  digitalWrite(LED_PARKING_FRONT, LOW);
  analogWrite(LED_PARKING_REAR, 0);
  digitalWrite(LED_TURN_LEFT, LOW);
  digitalWrite(LED_TURN_RIGHT, LOW);
  digitalWrite(LED_REVERSE, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  headlightsOn = false;
  parkingLightsOn = false;
  parkingModeOn = false;
  brakeActive = false;
  hazardOn = false;
  leftTurnActive = false;
  rightTurnActive = false;
  turnBlinkState = false;
  alarmOn = false;
  
  // Initialize variables
  systemEnabled = true;
  
  Serial.println("System initialized");
}

// ============================================
// MOTOR CONTROL FUNCTIONS
// ============================================

/**
 * Stop all motors
 */
void motorStop() {
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
  
  // Activate brake light for 1 second
  activateBrakeLight();
}

/**
 * Move forward at specified speed
 */
void motorForward(int speed) {
  speed = constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  digitalWrite(MOTOR_LEFT_DIR1, HIGH);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  analogWrite(MOTOR_LEFT_PWM, speed);
  analogWrite(MOTOR_RIGHT_PWM, speed);
  Serial.print("Motor Forward Speed: ");
  Serial.println(speed);
}

/**
 * Move backward at specified speed
 */
void motorBackward(int speed) {
  speed = constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
  analogWrite(MOTOR_LEFT_PWM, speed);
  analogWrite(MOTOR_RIGHT_PWM, speed);
  Serial.print("Motor Backward Speed: ");
  Serial.println(speed);
}

/**
 * Turn left at specified speed
 */
void motorTurnLeft(int speed) {
  speed = constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  analogWrite(MOTOR_LEFT_PWM, speed);
  analogWrite(MOTOR_RIGHT_PWM, speed);
  Serial.print("Motor Turn Left Speed: ");
  Serial.println(speed);
  setLeftTurn(true);
  setRightTurn(false);
}

/**
 * Turn right at specified speed
 */
void motorTurnRight(int speed) {
  speed = constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  digitalWrite(MOTOR_LEFT_DIR1, HIGH);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
  analogWrite(MOTOR_LEFT_PWM, speed);
  analogWrite(MOTOR_RIGHT_PWM, speed);
  Serial.print("Motor Turn Right Speed: ");
  Serial.println(speed);
  setLeftTurn(false);
  setRightTurn(true);
}

/**
 * Set individual motor speeds (for advanced control)
 */
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
  rightSpeed = constrain(rightSpeed, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
  
  // Left motor
  if (leftSpeed >= 0) {
    digitalWrite(MOTOR_LEFT_DIR1, HIGH);
    digitalWrite(MOTOR_LEFT_DIR2, LOW);
    analogWrite(MOTOR_LEFT_PWM, leftSpeed);
  } else {
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, HIGH);
    analogWrite(MOTOR_LEFT_PWM, -leftSpeed);
  }
  
  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
    analogWrite(MOTOR_RIGHT_PWM, rightSpeed);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
    analogWrite(MOTOR_RIGHT_PWM, -rightSpeed);
  }
}

/**
 * Move forward and left (diagonal) - left motor slower
 */
void motorForwardLeft(int speed) {
  speed = constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  int reducedSpeed = speed * 0.6;  // 60% speed for left motor
  digitalWrite(MOTOR_LEFT_DIR1, HIGH);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  analogWrite(MOTOR_LEFT_PWM, reducedSpeed);
  analogWrite(MOTOR_RIGHT_PWM, speed);
  Serial.print("Motor Forward-Left Speed: ");
  Serial.println(speed);
  setLeftTurn(true);
  setRightTurn(false);
}

/**
 * Move forward and right (diagonal) - right motor slower
 */
void motorForwardRight(int speed) {
  speed = constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  int reducedSpeed = speed * 0.6;  // 60% speed for right motor
  digitalWrite(MOTOR_LEFT_DIR1, HIGH);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  analogWrite(MOTOR_LEFT_PWM, speed);
  analogWrite(MOTOR_RIGHT_PWM, reducedSpeed);
  Serial.print("Motor Forward-Right Speed: ");
  Serial.println(speed);
  setLeftTurn(false);
  setRightTurn(true);
}

/**
 * Move backward and left (diagonal) - left motor slower
 */
void motorBackwardLeft(int speed) {
  speed = constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  int reducedSpeed = speed * 0.6;  // 60% speed for left motor
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
  analogWrite(MOTOR_LEFT_PWM, reducedSpeed);
  analogWrite(MOTOR_RIGHT_PWM, speed);
  Serial.print("Motor Backward-Left Speed: ");
  Serial.println(speed);
  setLeftTurn(true);
  setRightTurn(false);
}

/**
 * Move backward and right (diagonal) - right motor slower
 */
void motorBackwardRight(int speed) {
  speed = constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  int reducedSpeed = speed * 0.6;  // 60% speed for right motor
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
  analogWrite(MOTOR_LEFT_PWM, speed);
  analogWrite(MOTOR_RIGHT_PWM, reducedSpeed);
  Serial.print("Motor Backward-Right Speed: ");
  Serial.println(speed);
  setLeftTurn(false);
  setRightTurn(true);
}

// ============================================
// SENSOR FUNCTIONS
// ============================================

/**
 * Read distance from ultrasonic sensor (HC-SR04)
 * Returns distance in centimeters
 */
#ifdef ULTRASONIC_SENSOR_ENABLED
float readUltrasonicDistance() {
  // Send trigger pulse
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  // Read echo pulse
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000); // 30ms timeout
  
  // Calculate distance (speed of sound = 343 m/s)
  float distance = (duration * 0.0343) / 2.0;
  
  // Return 0 if no echo received
  return (duration == 0) ? 0 : distance;
}
#else
float readUltrasonicDistance() {
  return 0;
}
#endif

/**
 * Read IR sensor value
 * Returns digital value (0 or 1)
 */
#ifdef IR_SENSOR_ENABLED
int readIRSensor(int sensorPin) {
  return digitalRead(sensorPin);
}
#else
int readIRSensor(int sensorPin) {
  return 0;
}
#endif

// ============================================
// UTILITY FUNCTIONS
// ============================================

/**
 * Blink LED for indication
 */
void blinkLED(int pin, int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(delayMs);
    digitalWrite(pin, LOW);
    delay(delayMs);
  }
}

/**
 * Set headlights on/off
 */
void setHeadlights(bool state) {
  headlightsOn = state;
  digitalWrite(LED_HEADLIGHTS, state ? HIGH : LOW);
  BTSerial.print("Headlights: ");
  BTSerial.println(state ? "ON" : "OFF");
  Serial.print("Headlights: ");
  Serial.println(state ? "ON" : "OFF");
}

/**
 * Set parking lights on/off
 */
void setParkingLights(bool state) {
  parkingLightsOn = state;
  digitalWrite(LED_PARKING_FRONT, state ? HIGH : LOW);
  if (state) {
    analogWrite(LED_PARKING_REAR, LOW_PARKING); // Low brightness
  } else {
    analogWrite(LED_PARKING_REAR, 0);
  }
  BTSerial.print("Parking lights: ");
  BTSerial.println(state ? "ON" : "OFF");
  Serial.print("Parking lights: ");
  Serial.println(state ? "ON" : "OFF");
}

/**
 * Set parking mode on/off
 */
void setParkingMode(bool state) {
  parkingModeOn = state;
  if (state) {
    motorStop();
    analogWrite(LED_PARKING_REAR, 255); // Full brightness (brake/parking light)
    BTSerial.println("Parking mode: ON (motors locked)");
    Serial.println("Parking mode: ON (motors locked)");
  } else {
    // Return to parking lights state if they were on
    if (parkingLightsOn) {
      analogWrite(LED_PARKING_REAR, LOW_PARKING); // Low brightness
    } else {
      analogWrite(LED_PARKING_REAR, 0);
    }
    BTSerial.println("Parking mode: OFF");
    Serial.println("Parking mode: OFF");
  }
}

/**
 * Activate brake light for 1 second
 */
void activateBrakeLight() {
  if (!parkingModeOn) { // Don't override parking mode
    brakeActive = true;
    brakeStartMillis = millis();
    analogWrite(LED_PARKING_REAR, 255); // Full brightness
  }
}

/**
 * Update brake light timer (call in loop)
 */
void updateBrakeLight() {
  if (brakeActive && (millis() - brakeStartMillis >= BRAKE_LIGHT_DURATION)) {
    brakeActive = false;
    // Return to parking lights state if they were on
    if (parkingLightsOn && !parkingModeOn) {
      analogWrite(LED_PARKING_REAR, LOW_PARKING); // Low brightness
    } else if (!parkingModeOn) {
      analogWrite(LED_PARKING_REAR, 0);
    }
  }
}

/**
 * Turn signals and hazard handling
 */
void setLeftTurn(bool state) {
  leftTurnActive = state;
  if (!state) digitalWrite(LED_TURN_LEFT, LOW);
}

void setRightTurn(bool state) {
  rightTurnActive = state;
  if (!state) digitalWrite(LED_TURN_RIGHT, LOW);
}

void setHazard(bool state) {
  hazardOn = state;
  if (!state) {
    // Turn off both when hazard stops; resume based on active sides
    digitalWrite(LED_TURN_LEFT, LOW);
    digitalWrite(LED_TURN_RIGHT, LOW);
  }
}

void updateTurnSignals() {
  unsigned long now = millis();
  if (hazardOn || leftTurnActive || rightTurnActive) {
    if (now - lastTurnBlinkMillis >= TURN_BLINK_INTERVAL) {
      lastTurnBlinkMillis = now;
      turnBlinkState = !turnBlinkState;
    }
    // Hazard overrides individual
    if (hazardOn) {
      digitalWrite(LED_TURN_LEFT, turnBlinkState ? HIGH : LOW);
      digitalWrite(LED_TURN_RIGHT, turnBlinkState ? HIGH : LOW);
    } else {
      if (leftTurnActive) {
        digitalWrite(LED_TURN_LEFT, turnBlinkState ? HIGH : LOW);
      } else {
        digitalWrite(LED_TURN_LEFT, LOW);
      }
      if (rightTurnActive) {
        digitalWrite(LED_TURN_RIGHT, turnBlinkState ? HIGH : LOW);
      } else {
        digitalWrite(LED_TURN_RIGHT, LOW);
      }
    }
  } else {
    // Ensure off when inactive
    digitalWrite(LED_TURN_LEFT, LOW);
    digitalWrite(LED_TURN_RIGHT, LOW);
  }
}

/**
 * Alarm handling (blink LEDs and beep periodically)
 */
void setAlarm(bool state) {
  alarmOn = state;
  if (!state) {
    noTone(BUZZER_PIN);
  }
}

void updateAlarm() {
  if (!alarmOn || hornPlaying) return; // pause alarm sound while horn plays
  unsigned long now = millis();
  if (now - lastAlarmMillis >= ALARM_BLINK_INTERVAL) {
    lastAlarmMillis = now;
    // Blink headlights and both turn signals
    static bool alarmBlinkState = false;
    alarmBlinkState = !alarmBlinkState;
    digitalWrite(LED_HEADLIGHTS, alarmBlinkState ? HIGH : LOW);
    digitalWrite(LED_TURN_LEFT, alarmBlinkState ? HIGH : LOW);
    digitalWrite(LED_TURN_RIGHT, alarmBlinkState ? HIGH : LOW);
  }
  // Periodic short beep
  static unsigned long lastBeepMillis = 0;
  if (now - lastBeepMillis >= ALARM_BEEP_INTERVAL) {
    lastBeepMillis = now;
    tone(BUZZER_PIN, 1200, 120); // non-blocking short beep
  }
}

// Buzzer support removed: playTone deleted

// ============================================
// HORN (MELODY) IMPLEMENTATION
// ============================================
// Simple melody frequencies (Hz) and durations (ms)
const uint16_t HORN_NOTES[]     = { 523, 392, 330, 262, 392, 523 }; // C5 G4 E4 C4 G4 C5
const uint16_t HORN_DURATIONS[] = { 160, 160, 200, 260, 180, 300 };
const size_t HORN_LENGTH = sizeof(HORN_NOTES)/sizeof(HORN_NOTES[0]);

void startHorn() {
  if (hornPlaying) return; // already playing
  hornPlaying = true;
  hornIndex = 0;
  hornNoteStartMillis = millis();
  tone(BUZZER_PIN, HORN_NOTES[hornIndex], HORN_DURATIONS[hornIndex]);
}

void updateHorn() {
  if (!hornPlaying) return;
  unsigned long now = millis();
  if (now - hornNoteStartMillis >= (unsigned long)HORN_DURATIONS[hornIndex] + 40) { // 40ms gap
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

// ============================================
// BLUETOOTH HANDLING
// ============================================

void handleBluetoothInput() {
  while (BTSerial.available() > 0) {
    char c = (char)BTSerial.read();
    Serial.print("BT Received: ");
    Serial.println(c);
    processBTCommand(c);
  }
}

void processBTCommand(char c) {
  // Check for lowercase commands before converting to uppercase
  if (c == 'u') {
    setHeadlights(false);  // u = headlights OFF
    return;
  } else if (c == 'v') {
    setParkingLights(false);  // v = parking lights OFF
    return;
  } else if (c == 'w') {
    setParkingMode(false);  // w = parking mode OFF
    return;
  } else if (c == 'x') {
    setHazard(false);       // x = hazard OFF
    return;
  } else if (c == 'z') {
    setAlarm(false);        // z = alarm OFF
    return;
  }
  
  c = toupper(c);
  switch (c) {
    case 'F':
      if (systemEnabled && !parkingModeOn) motorForward(currentSpeed);
      setReverseIndicator(false);
      break;
    case 'B':
      if (systemEnabled && !parkingModeOn) motorBackward(currentSpeed);
      setReverseIndicator(true);
      break;
    case 'L':
      if (systemEnabled && !parkingModeOn) motorTurnLeft(currentSpeed);
      setLeftTurn(true); setRightTurn(false);
      setReverseIndicator(false);
      break;
    case 'R':
      if (systemEnabled && !parkingModeOn) motorTurnRight(currentSpeed);
      setLeftTurn(false); setRightTurn(true);
      setReverseIndicator(false);
      break;
    case 'G':
      if (systemEnabled && !parkingModeOn) motorForwardLeft(currentSpeed);
      setLeftTurn(true); setRightTurn(false);
      setReverseIndicator(false);
      break;
    case 'H':
      if (systemEnabled && !parkingModeOn) motorForwardRight(currentSpeed);
      setLeftTurn(false); setRightTurn(true);
      setReverseIndicator(false);
      break;
    case 'I':
      if (systemEnabled && !parkingModeOn) motorBackwardLeft(currentSpeed);
      setLeftTurn(true); setRightTurn(false);
      setReverseIndicator(true);
      break;
    case 'J':
      if (systemEnabled && !parkingModeOn) motorBackwardRight(currentSpeed);
      setLeftTurn(false); setRightTurn(true);
      setReverseIndicator(true);
      break;
    case 'U':
      setHeadlights(true);  // U = headlights ON
      break;
    case 'V':
      setParkingLights(true);  // V = parking lights ON
      break;
    case 'X':
      setHazard(true);       // X = hazard ON
      break;
    case 'Z':
      setAlarm(!alarmOn);    // Z = toggle alarm
      break;
    case 'Y':
      startHorn();           // Y = play horn melody once
      break;
    case 'W':
      setParkingMode(true);  // W = parking mode ON
      setReverseIndicator(false);
      break;
    case 'S':
      if (!parkingModeOn) motorStop();
      setLeftTurn(false); setRightTurn(false);
      setReverseIndicator(false);
      break;
    case 'E':
      systemEnabled = !systemEnabled;
      if (!systemEnabled) motorStop();
      if (!systemEnabled) setReverseIndicator(false);
      BTSerial.print("EN:"); BTSerial.println(systemEnabled ? 1 : 0);
      Serial.print("EN:"); Serial.println(systemEnabled ? 1 : 0);
      break;
    case 'Q':
      BTSerial.print("SPD:"); BTSerial.print(currentSpeed);
      BTSerial.print(" EN:"); BTSerial.println(systemEnabled ? 1 : 0);
      break;
    default:
      // digits 0-9 for speed
      if (c >= '0' && c <= '9') {
        int level = c - '0';
        int newSpeed = map(level, 0, 9, 0, MOTOR_SPEED_MAX);
        currentSpeed = newSpeed;
        BTSerial.print("SPD:"); BTSerial.println(currentSpeed);
      }
      break;
  }
}