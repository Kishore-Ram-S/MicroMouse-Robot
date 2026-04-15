/*******************************************************
* Micromouse Maze Solver - 10x10cm Grid
* Hardware: Arduino Nano, TB6612FNG, N20 Encoders, 5x VL53L0X
*******************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include <VL53L0X.h> // Make sure to install Pololu's VL53L0X library

// =====================================================
// PIN CONFIGURATION (Note: PWMB and XSHUT_LEFT swapped!)
// =====================================================
// Motor Driver (TB6612FNG)
#define STBY_PIN   9
#define AIN1_PIN   11   // Left motor
#define AIN2_PIN   4
#define PWMA_PIN   5    // Left motor PWM
#define BIN1_PIN   6    // Right motor
#define BIN2_PIN   7
#define PWMB_PIN   10   // Right motor PWM (Changed from 8 to 10 for PWM)

// Encoders (Pins 2 and 3 are hardware interrupts on Nano)
#define ENC_L_A    3    
#define ENC_L_B    A3
#define ENC_R_A    2    
#define ENC_R_B    A2

// VL53L0X XSHUT pins
#define XSHUT_FRONT        12
#define XSHUT_FRONT_LEFT   A1
#define XSHUT_FRONT_RIGHT  A0
#define XSHUT_LEFT         8    // (Changed from 10 to 8)
#define XSHUT_RIGHT        13

// =====================================================
// ROBOT PARAMETERS & MATH
// =====================================================
#define CELL_SIZE_MM   100
#define FORWARD_SPEED  70
#define MAX_SPEED      100
#define TURN_SPEED     60

// Tunable PID Constants for Wall Following
float Kp_wall = 0.8;
float Kd_wall = 2.5;

// Kinematics (Tune TRACK_WIDTH_MM based on your physical robot)
#define TICKS_PER_MM     4.244  
#define TRACK_WIDTH_MM   75.0   // <--- CHANGE THIS to your actual wheel-to-wheel distance

// =====================================================
// SENSORS & ENCODERS
// =====================================================
VL53L0X sensorF, sensorFL, sensorFR, sensorL, sensorR;

Encoder leftEncoder(ENC_L_A, ENC_L_B);
Encoder rightEncoder(ENC_R_A, ENC_R_B);

int distF = 500, distFL = 500, distFR = 500, distL = 500, distR = 500;

enum Action { MOVE_FORWARD, TURN_LEFT, TURN_RIGHT, TURN_BACK };

// =====================================================
// MOTOR CONTROL LOGIC
// =====================================================
void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(STBY_PIN, HIGH); // Enable TB6612FNG

  // Left Motor
  digitalWrite(AIN1_PIN, leftSpeed >= 0 ? HIGH : LOW);
  digitalWrite(AIN2_PIN, leftSpeed >= 0 ? LOW : HIGH);
  analogWrite(PWMA_PIN, constrain(abs(leftSpeed), 0, 255));

  // Right Motor
  digitalWrite(BIN1_PIN, rightSpeed >= 0 ? HIGH : LOW);
  digitalWrite(BIN2_PIN, rightSpeed >= 0 ? LOW : HIGH);
  analogWrite(PWMB_PIN, constrain(abs(rightSpeed), 0, 255));
}

void stopMotors() {
  setMotors(0, 0);
  digitalWrite(STBY_PIN, LOW); // Put driver in standby to save power/brake
}

// =====================================================
// SENSOR READING & DECISION
// =====================================================
void readSensors() {
  distF  = sensorF.readRangeContinuousMillimeters();
  distFL = sensorFL.readRangeContinuousMillimeters();
  distFR = sensorFR.readRangeContinuousMillimeters();
  distL  = sensorL.readRangeContinuousMillimeters();
  distR  = sensorR.readRangeContinuousMillimeters();
}

Action decideNextMove() {
  // Threshold for wall detection (adjust based on cell size 100mm)
  int threshold = 80; 
  
  // Use front three sensors to reliably detect a frontal wall
  bool wallFront = (distF < threshold || distFL < threshold || distFR < threshold);
  bool wallLeft  = (distL < threshold);
  bool wallRight = (distR < threshold);

  // Left-Hand Rule Maze Solving
  if (!wallLeft) return TURN_LEFT;       // Always turn left if open
  if (!wallFront) return MOVE_FORWARD;   // Go straight if left is blocked but front is open
  if (!wallRight) return TURN_RIGHT;     // Turn right if left and front are blocked
  return TURN_BACK;                      // Dead end
}

// =====================================================
// MOVEMENT FUNCTIONS
// =====================================================
void moveForwardPID(int mm) {
  leftEncoder.write(0);
  rightEncoder.write(0);
  int prevError = 0;
  long targetTicks = mm * TICKS_PER_MM;

  while (true) {
    readSensors();
    long traveledL = abs(leftEncoder.read());
    long traveledR = abs(rightEncoder.read());
    
    if ((traveledL + traveledR) / 2 >= targetTicks) break;

    int error = 0;
    // Only use PID if we are between two walls. Otherwise, rely on encoders for straightness.
    if (distL < 80 && distR < 80) {
      error = distL - distR; 
    } else {
      // Sync encoders if no walls to track
      error = traveledL - traveledR; 
    }

    int correction = (Kp_wall * error) + (Kd_wall * (error - prevError));
    prevError = error;

    int lSpeed = constrain(FORWARD_SPEED + correction, 0, MAX_SPEED);
    int rSpeed = constrain(FORWARD_SPEED - correction, 0, MAX_SPEED);

    setMotors(lSpeed, rSpeed);
  }
  stopMotors();
}

void rotate(int dir, int angle) {
  // dir: 1 for Left, -1 for Right
  leftEncoder.write(0);
  rightEncoder.write(0);
  
  // Calculate arc length for the wheel to travel
  float arcLength = (PI * TRACK_WIDTH_MM * angle) / 360.0;
  long targetTicks = arcLength * TICKS_PER_MM;

  while (abs(leftEncoder.read()) < targetTicks) {
    // Spin in place
    setMotors(-dir * TURN_SPEED, dir * TURN_SPEED);
  }
  stopMotors();
}

// =====================================================
// SETUP & INITIALIZATION
// =====================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Fast I2C

  // Motor Pins
  pinMode(STBY_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT); pinMode(AIN2_PIN, OUTPUT); pinMode(PWMA_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT); pinMode(BIN2_PIN, OUTPUT); pinMode(PWMB_PIN, OUTPUT);
  
  // Init XSHUT pins
  int xshutPins[] = {XSHUT_FRONT, XSHUT_FRONT_LEFT, XSHUT_FRONT_RIGHT, XSHUT_LEFT, XSHUT_RIGHT};
  for (int i = 0; i < 5; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW); // Turn off all sensors
  }
  delay(10);

  // Initialize Sensors sequentially with new addresses
  VL53L0X* sensors[] = {&sensorF, &sensorFL, &sensorFR, &sensorL, &sensorR};
  uint8_t addresses[] = {0x2A, 0x2B, 0x2C, 0x2D, 0x2E};

  for (int i = 0; i < 5; i++) {
    digitalWrite(xshutPins[i], HIGH);
    delay(10);
    sensors[i]->setTimeout(500);
    if (!sensors[i]->init()) {
      Serial.print("Failed to detect sensor "); Serial.println(i);
      while (1); 
    }
    sensors[i]->setAddress(addresses[i]);
    sensors[i]->startContinuous(20); // 20ms continuous reading
  }
  delay(1000); // Brief pause before starting
}

// =====================================================
// MAIN LOOP
// =====================================================
void loop() {
  readSensors();
  Action action = decideNextMove();

  switch (action) {
    case MOVE_FORWARD:
      moveForwardPID(CELL_SIZE_MM);
      break;
    case TURN_LEFT:
      rotate(1, 90);
      moveForwardPID(CELL_SIZE_MM); // Move into the cell after turning
      break;
    case TURN_RIGHT:
      rotate(-1, 90);
      moveForwardPID(CELL_SIZE_MM);
      break;
    case TURN_BACK:
      rotate(1, 180);
      moveForwardPID(CELL_SIZE_MM);
      break;
  }
  delay(150); // Small pause to stabilize sensor readings
}
