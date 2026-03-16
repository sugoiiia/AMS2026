///////////////////////////////////////////////////////////////////
//      Albertaloop Mechatronics Showdown
//      AMS Robot Car - Mega 2560 + MPU6050 PID + Clamped PWM
//
//      PURPOSE: Autonomously balance the robot on the balance beam
//      using a PID control loop reading pitch angle from the IMU.
//
//      HOW IT WORKS IN ONE SENTENCE:
//      Every loop, read tilt angle → calculate how far off balance
//      we are → run PID math → drive motors to correct.
///////////////////////////////////////////////////////////////////

#include <Wire.h>           // Built-in Arduino I2C library — required for IMU communication
#include <MPU6050_light.h>  // Lightweight MPU6050 library. Handles raw register reads,
                            // offset calibration, and angle fusion internally.
                            // Install via: Library Manager → search "MPU6050_light" by rfetick

MPU6050 mpu(Wire);          // Create IMU object. Wire = the I2C bus.
                            // SDA → Pin 20, SCL → Pin 21 on Mega 2560.

// ─────────────────────────────────────────────────────────────
// MOTOR DRIVER PINS
// Two L298N H-bridge drivers — one controls front pair, one controls back pair.
// Each motor needs 3 pins:
//   enable (EN) = PWM speed control via analogWrite()
//   IN1 + IN2   = direction control via digitalWrite()
//                 IN1 HIGH + IN2 LOW  = forward
//                 IN1 LOW  + IN2 HIGH = backward
// ─────────────────────────────────────────────────────────────
#define back_left_enable  7   // PWM — controls back left motor speed
#define back_left_1       6   // Direction bit A
#define back_left_2       5   // Direction bit B

#define back_right_enable 2   // PWM — controls back right motor speed
#define back_right_1      4   // Direction bit A
#define back_right_2      3   // Direction bit B

#define front_right_enable 8  // PWM — controls front right motor speed
#define front_right_1      9  // Direction bit A
#define front_right_2      10 // Direction bit B

#define front_left_enable  13 // PWM — controls front left motor speed
#define front_left_1       11 // Direction bit A
#define front_left_2       12 // Direction bit B

// ─────────────────────────────────────────────────────────────
// TUNING VARIABLES — adjust these to change robot behaviour
// ─────────────────────────────────────────────────────────────

// 1. Hardware limits
int maxPWM = 130;
// Hard cap on motor output. Robot runs on ~11V (two 6V packs in series).
// Full 255 PWM at 11V would be violent — 130 keeps movements controlled.
// Increase carefully if corrections feel too weak.

int motorDeadband = 72;
// Motors physically cannot overcome their own friction below this PWM on carpet.
// Any non-zero value below 72 just buzzes the motor without movement.
// Discovered empirically during carpet testing. Applied in step 5.

// 2. Calibration
float setpoint = 0.0;
// The target angle in degrees. 0 = perfectly level.
// If the robot balances at a slight lean, change this to match.

float mechanicalOffset = 1;
// Physical correction for centre-of-gravity not being exactly at 0°.
// Added to raw angle every loop. Tweak if robot always drifts one direction
// even when code says it should be balanced.

// 3. PID gains — the three numbers that define the controller's personality
float Kp = 30;
// PROPORTIONAL gain. The main correction force.
// Output = Kp × error. If tilt is 2°, correction = 30 × 2 = 60 PWM.
// Too low: sluggish, falls over slowly. Too high: oscillates violently.
// TUNE THIS FIRST — set Ki and Kd to 0, adjust Kp until robot nearly balances.

float Ki = 0.01;
// INTEGRAL gain. Fixes long-term drift.
// Accumulates error over time — if robot leans slightly for many seconds,
// integral builds up and pushes harder. Keep small to avoid "windup"
// (runaway integral if robot is stuck against something).
// The constrain(-100, 100) on cumError is the anti-windup guard.

float Kd = 3;
// DERIVATIVE gain. Dampens oscillation.
// Responds to the RATE of change of error, not the error itself.
// If the robot is tipping fast, Kd hits the brakes before it overshoots.
// Too high: twitchy/noisy response. Too low: overshoots and oscillates.

// ─────────────────────────────────────────────────────────────
// SYSTEM VARIABLES — don't touch these, they're runtime state
// ─────────────────────────────────────────────────────────────
int speed_back_left   = 0;  // Current PWM value sent to back left motor
int speed_back_right  = 0;  // Current PWM value sent to back right motor
int speed_front_right = 0;  // Current PWM value sent to front right motor
int speed_front_left  = 0;  // Current PWM value sent to front left motor

float cumError  = 0;        // Running sum of all past errors × time (integral term)
float lastError = 0;        // Error from the previous loop cycle (needed for derivative)

unsigned long previousTime  = 0;  // Timestamp of last loop (for dt calculation)
unsigned long lastPrintTime = 0;  // Timestamp of last Serial print (rate-limits debug output)

///////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);  // Open serial at 9600 baud — matches Serial Monitor setting
  Wire.begin();        // Start I2C bus on pins 20 (SDA) and 21 (SCL)

  Serial.println("Initializing MPU6050...");

  byte status = mpu.begin();
  // mpu.begin() attempts to wake the IMU from sleep mode and verify communication.
  // Returns 0 on success, non-zero error code on failure.

  while (status != 0) {
    // Loop forever if IMU not found — prevents robot running blind.
    // Common causes: loose SDA/SCL wire, wrong I2C address, no 5V to IMU VCC.
    Serial.print("MPU6050 Connection Error! Status Code: ");
    Serial.println(status);
    Serial.println("Check SDA (Pin 20) and SCL (Pin 21) on Mega 2560.");
    delay(1000);
  }

  Serial.println("MPU6050 connected! Calculating offsets...");
  Serial.println("DO NOT MOVE THE ROBOT!");
  delay(1000);

  mpu.calcOffsets(true, true);
  // Reads gyro and accelerometer for ~1 second while robot is stationary.
  // Measures the "resting" bias of each axis and stores correction values.
  // This zeros out manufacturing imperfections in the sensor.
  // CRITICAL: robot must be perfectly still during this — any movement
  // corrupts the offsets and the angle readings will be wrong all run.
  // true, true = calibrate both gyro AND accelerometer.

  Serial.println("Offsets calculated. Ready to balance.");

  // Set all motor control pins as outputs
  // Must be done before any digitalWrite/analogWrite calls
  pinMode(back_left_enable,  OUTPUT); pinMode(back_left_1,  OUTPUT); pinMode(back_left_2,  OUTPUT);
  pinMode(back_right_enable, OUTPUT); pinMode(back_right_1, OUTPUT); pinMode(back_right_2, OUTPUT);
  pinMode(front_right_enable,OUTPUT); pinMode(front_right_1,OUTPUT); pinMode(front_right_2,OUTPUT);
  pinMode(front_left_enable, OUTPUT); pinMode(front_left_1, OUTPUT); pinMode(front_left_2, OUTPUT);

  previousTime = millis();  // Seed the timer so first dt calculation is valid
}

///////////////////////////////////////////////////////////////////
void loop() {
  // ── STEP 0: Update IMU ──────────────────────────────────────
  mpu.update();
  // MPU6050_light does NOT auto-update. This must be called every loop.
  // Internally it reads raw gyro/accel registers, applies the offsets from
  // calcOffsets(), and recalculates the fused angle using a complementary
  // filter (98% gyro + 2% accel). getAngleY() won't change until this runs.

  // ── STEP 1: Calculate dt ────────────────────────────────────
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - previousTime) / 1000.0;
  // dt in seconds. PID math requires real time, not loop count.
  // A fast loop running in 5ms has dt=0.005 — same error produces
  // smaller integral and derivative terms than a slow 50ms loop.
  if (elapsedTime <= 0.0) elapsedTime = 0.001;
  // Guard against divide-by-zero if two loops execute in the same millisecond.

  // ── STEP 2: Read angle ──────────────────────────────────────
  float rawAngle = mpu.getAngleY();
  // Y-axis = pitch (nose up/nose down) when IMU is mounted flat on robot.
  // Positive = nose up, negative = nose down (or reversed — check with Serial Monitor).

  float currentAngle = rawAngle + mechanicalOffset;
  // Apply physical correction. If robot's centre of mass is slightly forward
  // of geometric centre, it will naturally lean — offset compensates for this.

  // ── STEP 3: PID calculation ─────────────────────────────────
  float error = currentAngle - setpoint;
  // How far off balance are we right now?
  // Positive error = leaning too far one way, negative = the other.

  cumError += error * elapsedTime;
  cumError = constrain(cumError, -100, 100);
  // Integral term: sum of (error × time) over every past loop.
  // Multiplying by elapsedTime makes it time-accurate regardless of loop speed.
  // constrain() is the anti-windup guard — without it, if the robot is stuck
  // (e.g. against a wall), cumError grows unbounded and the robot launches
  // itself when it breaks free.

  float rateError = (error - lastError) / elapsedTime;
  // Derivative term: how fast is the error changing?
  // If error is 2° and was 1° last loop, rateError = positive → tipping faster.
  // If error is 2° and was 3° last loop, rateError = negative → recovering.

  float output = (Kp * error) + (Ki * cumError) + (Kd * rateError);
  // Full PID output. Sign determines direction, magnitude determines speed.
  // Positive output = drive forward, negative = drive backward.

  lastError    = error;        // Save for next loop's derivative calculation
  previousTime = currentTime;  // Save for next loop's dt calculation

  // ── STEP 4: Clamp PID output ────────────────────────────────
  int pidOutput = constrain((int)output, -maxPWM, maxPWM);
  // Cap at ±130 PWM. Prevents violent overcorrection at 11V.
  // Also converts float → int since analogWrite takes integers.

  // ── STEP 5: Set motor directions and speeds ─────────────────
  if (pidOutput < 0) {
    // Robot needs to move BACKWARD to correct tilt

    // Left side — backward
    digitalWrite(back_left_1,   LOW);  digitalWrite(back_left_2,   HIGH);
    digitalWrite(front_left_1,  HIGH); digitalWrite(front_left_2,  LOW);

    // Right side — INVERTED wiring requires opposite direction bits
    // These motors are physically mounted facing the opposite direction,
    // so HIGH/LOW is swapped to achieve the same physical movement.
    digitalWrite(back_right_1,  HIGH); digitalWrite(back_right_2,  LOW);
    digitalWrite(front_right_1, LOW);  digitalWrite(front_right_2, HIGH);

    int mappedSpeed = abs(pidOutput);  // abs() because speed is always positive
    speed_back_left = speed_back_right = speed_front_right = speed_front_left = mappedSpeed;

  } else if (pidOutput > 0) {
    // Robot needs to move FORWARD to correct tilt

    // Left side — forward
    digitalWrite(back_left_1,   HIGH); digitalWrite(back_left_2,   LOW);
    digitalWrite(front_left_1,  LOW);  digitalWrite(front_left_2,  HIGH);

    // Right side — INVERTED
    digitalWrite(back_right_1,  LOW);  digitalWrite(back_right_2,  HIGH);
    digitalWrite(front_right_1, HIGH); digitalWrite(front_right_2, LOW);

    speed_back_left = speed_back_right = speed_front_right = speed_front_left = pidOutput;

  } else {
    // pidOutput == 0: perfectly balanced, stop all motors
    speed_back_left = speed_back_right = speed_front_right = speed_front_left = 0;
  }

  // ── STEP 6: Apply motor deadband ────────────────────────────
  // Any speed below motorDeadband (72) buzzes the motor without movement.
  // More honest to just stop than to waste power making noise.
  // Note: this clips small corrections — a side effect is the robot makes
  // small micro-adjustments in discrete steps rather than continuously.
  if (speed_back_left   < motorDeadband) speed_back_left   = 0;
  if (speed_back_right  < motorDeadband) speed_back_right  = 0;
  if (speed_front_right < motorDeadband) speed_front_right = 0;
  if (speed_front_left  < motorDeadband) speed_front_left  = 0;

  // ── STEP 7: Send PWM to motors ───────────────────────────────
  // analogWrite generates a PWM signal on the enable pin.
  // 0 = motor off, 255 = full speed. Direction is already set by IN1/IN2.
  // The L298N's internal H-bridge uses IN1/IN2 to route current either
  // direction through the motor coils.
  analogWrite(back_left_enable,   speed_back_left);
  analogWrite(back_right_enable,  speed_back_right);
  analogWrite(front_right_enable, speed_front_right);
  analogWrite(front_left_enable,  speed_front_left);

  // ── DEBUG: Serial output (rate-limited to every 100ms) ───────
  if (currentTime - lastPrintTime >= 100) {
    // Printing every loop would spam Serial at ~500+ lines/sec and
    // slow down the loop itself. 100ms gives 10 updates/sec — readable.
    Serial.print("Raw Angle: ");      Serial.print(rawAngle);
    Serial.print("  |  Corrected: "); Serial.print(currentAngle);
    Serial.print("  |  PID Out: ");   Serial.print(pidOutput);
    Serial.print("  |  Motor PWM: "); Serial.println(speed_back_left);
    lastPrintTime = currentTime;
  }
}