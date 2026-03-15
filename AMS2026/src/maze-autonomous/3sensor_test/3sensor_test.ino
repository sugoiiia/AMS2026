///////////////////////////////////////////////////////////////////
//      Albertaloop Mechatronics Showdown 2026
//      Autonomous Maze Navigation
//      Version: 2.0 — Gyro-controlled turning
//
//      Strategy:
//      - Left-hand wall following using ultrasonic sensor
//      - Front collision detection using IR sensor
//      - Right wall detection using IR sensor
//      - turnRight uses IMU gyro to measure exactly 85 degrees
//
//      Sensor Wiring:
//      - Ultrasonic TRIG  --> Pin 22
//      - Ultrasonic ECHO  --> Pin 23
//      - Front IR OUT     --> Pin 24
//      - Right IR OUT     --> Pin 25
//      - IMU SDA          --> Pin 20
//      - IMU SCL          --> Pin 21
///////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <MPU6050.h>

// ---------------------------------------------------------------
// TUNING PARAMETERS
// ---------------------------------------------------------------
#define BASE_SPEED         250
#define TURN_SPEED         250
#define REVERSE_SPEED      0

#define WALL_FOLLOW_DIST   5.5
#define WALL_TOO_CLOSE     3
#define WALL_TOO_FAR       10
#define FRONT_CLEAR_DIST   20

#define REVERSE_TIME       100
#define LEFT_TURN_TIME     300

// Gyro turn target — 85° undershoots slightly, wall following corrects the rest
// If robot overshoots, decrease this. If it undershoots too much, increase it.
#define TURN_TARGET_DEG    85.0

// Safety timeout — if gyro turn takes longer than this, stop anyway
// Prevents robot getting stuck if IMU glitches
#define TURN_TIMEOUT_MS    3000
// ---------------------------------------------------------------

// --- Motor Pins ---
#define back_left_enable   7
#define back_left_1        6
#define back_left_2        5
#define back_right_enable  2
#define back_right_1       4
#define back_right_2       3
#define front_right_enable 8
#define front_right_1      9
#define front_right_2      10
#define front_left_enable  13
#define front_left_1       11
#define front_left_2       12

// --- Sensor Pins ---
#define TRIG_PIN           22
#define ECHO_PIN           23
#define FRONT_IR_PIN       24
#define RIGHT_IR_PIN       25

// --- IMU ---
MPU6050 mpu;

///////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
  Wire.begin();

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 FAILED — check wiring");
    while (true);
  }
  Serial.println("MPU6050 OK");

  // Motor pins
  pinMode(back_left_enable,   OUTPUT);
  pinMode(back_left_1,        OUTPUT);
  pinMode(back_left_2,        OUTPUT);
  pinMode(back_right_enable,  OUTPUT);
  pinMode(back_right_1,       OUTPUT);
  pinMode(back_right_2,       OUTPUT);
  pinMode(front_right_enable, OUTPUT);
  pinMode(front_right_1,      OUTPUT);
  pinMode(front_right_2,      OUTPUT);
  pinMode(front_left_enable,  OUTPUT);
  pinMode(front_left_1,       OUTPUT);
  pinMode(front_left_2,       OUTPUT);

  // Sensor pins
  pinMode(TRIG_PIN,     OUTPUT);
  pinMode(ECHO_PIN,     INPUT);
  pinMode(FRONT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);

  stopMotors();
  delay(2000);
  Serial.println("Autonomous maze starting...");
}

///////////////////////////////////////////////////////////////////
void loop() {
  bool frontWall = (digitalRead(FRONT_IR_PIN) == LOW);
  bool rightWall = (digitalRead(RIGHT_IR_PIN) == LOW);
  int  leftDist  = getUltrasonicDistance();

  Serial.print("Front: "); Serial.print(frontWall ? "WALL" : "clear");
  Serial.print(" | Right: "); Serial.print(rightWall ? "WALL" : "clear");
  Serial.print(" | Left dist: "); Serial.print(leftDist);
  Serial.println(" cm");

  // PRIORITY 1: Front wall — stop, turn right by gyro
  if (frontWall) {
    Serial.println("ACTION: Front wall — gyro turn right 85 degrees");
    stopMotors();
    delay(500);
    turnRightGyro();
    stopMotors();
    delay(100);
    return;
  }

  // PRIORITY 2: Left wall too close — steer right
  if (leftDist > 0 && leftDist < WALL_TOO_CLOSE) {
    Serial.println("ACTION: Too close to left wall — steering right");
    steerRight();
    return;
  }

  // PRIORITY 3: Left wall in range — drive straight
  if (leftDist >= WALL_TOO_CLOSE && leftDist <= WALL_TOO_FAR) {
    Serial.println("ACTION: Wall in range — forward");
    moveForward(BASE_SPEED);
    return;
  }

  // PRIORITY 4: Left wall lost — nudge left to find it
  if (leftDist > WALL_TOO_FAR || leftDist == 0) {
    if (rightWall) {
      Serial.println("ACTION: Left wall lost but right wall present — forward");
      moveForward(BASE_SPEED);
    } else {
      Serial.println("ACTION: Left wall lost — nudging left");
      steerLeft();
      delay(LEFT_TURN_TIME);
    }
    return;
  }
}

///////////////////////////////////////////////////////////////////
// GYRO TURN — right, target degrees
///////////////////////////////////////////////////////////////////

void turnRightGyro() {
  float angleTurned = 0.0;
  unsigned long lastTime = millis();
  unsigned long startTime = millis();

  // Start turning
  setTurnRightDirections();
  setSpeed(TURN_SPEED, TURN_SPEED, TURN_SPEED, TURN_SPEED);

  while (angleTurned < TURN_TARGET_DEG) {
    // Safety timeout
    if (millis() - startTime > TURN_TIMEOUT_MS) {
      Serial.println("WARN: Turn timeout — gyro may have glitched");
      break;
    }

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // Read gyro Z axis (yaw rate in degrees/second)
    // MPU6050 Z gyro = yaw when flat on robot
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Default sensitivity: 131 LSB per deg/s
    float yawRate = (float)gz / 131.0;

    // Integrate to get angle — take absolute value since direction is known
    angleTurned += abs(yawRate) * dt;

    Serial.print("Turning: "); Serial.print(angleTurned, 1);
    Serial.print(" / "); Serial.print(TURN_TARGET_DEG); Serial.println(" deg");
  }

  stopMotors();
  Serial.println("Turn complete");
}

// Set motor directions for right turn (separated so loop can call setSpeed directly)
void setTurnRightDirections() {
  // Left side backward
  digitalWrite(back_left_1,   LOW);  digitalWrite(back_left_2,   HIGH);
  digitalWrite(front_left_1,  HIGH); digitalWrite(front_left_2,  LOW);
  // Right side forward
  digitalWrite(back_right_1,  LOW);  digitalWrite(back_right_2,  HIGH);
  digitalWrite(front_right_1, HIGH); digitalWrite(front_right_2, LOW);
}

///////////////////////////////////////////////////////////////////
// MOTOR CONTROL FUNCTIONS
///////////////////////////////////////////////////////////////////

void moveForward(int spd) {
  digitalWrite(back_left_1,   HIGH); digitalWrite(back_left_2,   LOW);
  digitalWrite(back_right_1,  LOW);  digitalWrite(back_right_2,  HIGH);
  digitalWrite(front_left_1,  LOW);  digitalWrite(front_left_2,  HIGH);
  digitalWrite(front_right_1, HIGH); digitalWrite(front_right_2, LOW);
  setSpeed(spd, spd, spd, spd);
}

void moveBackward(int spd) {
  digitalWrite(back_left_1,   LOW);  digitalWrite(back_left_2,   HIGH);
  digitalWrite(back_right_1,  HIGH); digitalWrite(back_right_2,  LOW);
  digitalWrite(front_left_1,  HIGH); digitalWrite(front_left_2,  LOW);
  digitalWrite(front_right_1, LOW);  digitalWrite(front_right_2, HIGH);
  setSpeed(spd, spd, spd, spd);
}

void turnRight(int spd) {
  setTurnRightDirections();
  setSpeed(spd, spd, spd, spd);
}

void turnLeft(int spd) {
  digitalWrite(back_left_1,   LOW);  digitalWrite(back_left_2,   HIGH);
  digitalWrite(front_left_1,  HIGH); digitalWrite(front_left_2,  LOW);
  digitalWrite(back_right_1,  LOW);  digitalWrite(back_right_2,  HIGH);
  digitalWrite(front_right_1, HIGH); digitalWrite(front_right_2, LOW);
  setSpeed(spd, spd, spd, spd);
}

void steerRight() {
  digitalWrite(back_left_1,   HIGH); digitalWrite(back_left_2,   LOW);
  digitalWrite(back_right_1,  LOW);  digitalWrite(back_right_2,  HIGH);
  digitalWrite(front_left_1,  LOW);  digitalWrite(front_left_2,  HIGH);
  digitalWrite(front_right_1, HIGH); digitalWrite(front_right_2, LOW);
  setSpeed(150, 80, 150, 80);
}

void steerLeft() {
  digitalWrite(back_left_1,   HIGH); digitalWrite(back_left_2,   LOW);
  digitalWrite(back_right_1,  LOW);  digitalWrite(back_right_2,  HIGH);
  digitalWrite(front_left_1,  LOW);  digitalWrite(front_left_2,  HIGH);
  digitalWrite(front_right_1, HIGH); digitalWrite(front_right_2, LOW);
  setSpeed(80, 150, 80, 150);
}

void stopMotors() {
  setSpeed(0, 0, 0, 0);
}

void setSpeed(int bl, int br, int fl, int fr) {
  if (bl > 0 && bl < 72) bl = 72;
  if (br > 0 && br < 72) br = 72;
  if (fl > 0 && fl < 72) fl = 72;
  if (fr > 0 && fr < 72) fr = 72;
  analogWrite(back_left_enable,   bl);
  analogWrite(back_right_enable,  br);
  analogWrite(front_left_enable,  fl);
  analogWrite(front_right_enable, fr);
}

///////////////////////////////////////////////////////////////////
// ULTRASONIC SENSOR
///////////////////////////////////////////////////////////////////

int getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 0;
  return duration * 0.034 / 2;
}