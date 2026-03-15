///////////////////////////////////////////////////////////////////
//      Albertaloop Mechatronics Showdown 2026
//      Autonomous Maze Navigation - FINAL V4.0
//      Left Wall Follow + Gyro Turns + Backup Clearance
///////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <MPU6050_light.h>

// ---------------------------------------------------------------
// TUNING PARAMETERS
// ---------------------------------------------------------------
#define BASE_SPEED         220  // Max PWM for 11V safe cruising
#define TURN_SPEED         255  // Slightly higher for carpet friction
#define STEER_SPEED_FAST   255
#define STEER_SPEED_SLOW   170
#define BACKUP_SPEED       100
// Distance thresholds (in cm)
#define WALL_TOO_CLOSE     4
#define WALL_TOO_FAR       12

// Gyro and Timing Limits
#define TURN_TARGET_DEG    88.0 // Undershoot slightly to account for momentum
#define TURN_TIMEOUT_MS    3000 // Safety timeout if IMU glitches
#define BACKUP_DELAY_MS    300  // Time to backup when front wall is hit
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
MPU6050 mpu(Wire);

///////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
  Wire.begin();

  Serial.println("Initializing MPU6050...");
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("MPU6050 FAILED! Status: ");
    Serial.println(status);
    while (true);
  }
  
  Serial.println("MPU6050 OK. Calculating offsets. DO NOT MOVE ROBOT!");
  delay(1000);
  mpu.calcOffsets(true, true);
  Serial.println("Offsets set.");

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
  delay(2000); // Wait 2 seconds before the run starts
  Serial.println("Autonomous maze starting...");
}

///////////////////////////////////////////////////////////////////
void loop() {
  // MUST update gyro constantly
  mpu.update();

  // Read sensors (IRs output LOW when detecting an object)
  bool frontWall = (digitalRead(FRONT_IR_PIN) == LOW);
  bool rightWallIR = (digitalRead(RIGHT_IR_PIN) == LOW);
  int  rightDist  = getUltrasonicDistance();

// -------------------------------------------------------------
  // PRIORITY 1: Front wall — stop, backup, turn LEFT by gyro
  // -------------------------------------------------------------
  if (frontWall) {
    stopMotors();
    delay(100); // Settle forward momentum
    
    // Backup slowly to clear the swing radius
    moveBackward(BACKUP_SPEED); // <--- CHANGED THIS LINE
    delay(BACKUP_DELAY_MS);
    
    stopMotors();
    delay(100); // Settle backward momentum before turning

    // Execute precision 90-degree turn
    turnLeftGyro();
    stopMotors();
    delay(100);
    return;
  }

  // -------------------------------------------------------------
  // PRIORITY 2: Right wall too close — steer left
  // -------------------------------------------------------------
  if ((rightDist > 0 && rightDist < WALL_TOO_CLOSE) || rightWallIR) {
    steerLeft();
    return;
  }

  // -------------------------------------------------------------
  // PRIORITY 3: Right wall perfectly in range — drive straight
  // -------------------------------------------------------------
  if (rightDist >= WALL_TOO_CLOSE && rightDist <= WALL_TOO_FAR) {
    moveForward(BASE_SPEED);
    return;
  }

  // -------------------------------------------------------------
  // PRIORITY 4: Right wall lost — steer right to find it
  // -------------------------------------------------------------
  if (rightDist > WALL_TOO_FAR || rightDist == 0) {
    steerRight();
    return;
  }
  
  delay(20); // Small loop stability delay
}

///////////////////////////////////////////////////////////////////
// GYRO TURN — LEFT, target degrees
///////////////////////////////////////////////////////////////////

void turnLeftGyro() {
  mpu.update();
  float startAngle = mpu.getAngleZ();
  unsigned long startTime = millis();

  // Start turning Left
  setTurnLeftDirections();
  setSpeed(TURN_SPEED, TURN_SPEED, TURN_SPEED, TURN_SPEED);

  // MPU6050_light angleZ uses absolute difference to track rotation
  while (abs(mpu.getAngleZ() - startAngle) < TURN_TARGET_DEG) {
    // Safety timeout in case of sensor freeze
    if (millis() - startTime > TURN_TIMEOUT_MS) {
      Serial.println("WARN: Turn timeout — gyro may have glitched");
      break;
    }
    mpu.update(); 
  }

  stopMotors();
  Serial.println("Left turn complete");
}

void setTurnLeftDirections() {
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
  // Left Side Forward
  digitalWrite(back_left_1,   HIGH); digitalWrite(back_left_2,   LOW);
  digitalWrite(front_left_1,  LOW);  digitalWrite(front_left_2,  HIGH);
  // Right Side Forward
  digitalWrite(back_right_1,  LOW);  digitalWrite(back_right_2,  HIGH);
  digitalWrite(front_right_1, HIGH); digitalWrite(front_right_2, LOW);
  
  setSpeed(spd, spd, spd, spd);
}

void moveBackward(int spd) {
  // Left Side Backward
  digitalWrite(back_left_1,   LOW);  digitalWrite(back_left_2,   HIGH);
  digitalWrite(front_left_1,  HIGH); digitalWrite(front_left_2,  LOW);
  // Right Side Backward 
  digitalWrite(back_right_1,  HIGH); digitalWrite(back_right_2,  LOW);
  digitalWrite(front_right_1, LOW);  digitalWrite(front_right_2, HIGH);
  
  setSpeed(spd, spd, spd, spd);
}

void steerLeft() {
  // Forward directions, but slow down left side to veer left
  moveForward(BASE_SPEED);
  setSpeed(STEER_SPEED_SLOW, STEER_SPEED_FAST, STEER_SPEED_SLOW, STEER_SPEED_FAST);
}

void steerRight() {
  // Forward directions, but slow down right side to veer right
  moveForward(BASE_SPEED);
  setSpeed(STEER_SPEED_FAST, STEER_SPEED_SLOW, STEER_SPEED_FAST, STEER_SPEED_SLOW);
}

void stopMotors() {
  setSpeed(0, 0, 0, 0);
}

void setSpeed(int bl, int br, int fl, int fr) {
  // Apply deadband
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
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout (~5m max)
  if (duration == 0) return 999; // Return high number if no echo
  return duration * 0.034 / 2;
}
