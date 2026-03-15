///////////////////////////////////////////////////////////////////
//      Albertaloop Mechatronics Showdown 2026
//      Autonomous Maze Navigation - FINAL V4.0
//      Right Wall Follow + Gyro Turns + Backup Clearance
///////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <MPU6050_light.h>

#define BASE_SPEED         220
#define TURN_SPEED         255
#define STEER_SPEED_FAST   255
#define STEER_SPEED_SLOW   170
#define BACKUP_SPEED       100
#define WALL_TOO_CLOSE     4
#define WALL_TOO_FAR       12
#define TURN_TARGET_DEG    88.0
#define TURN_TIMEOUT_MS    3000
#define BACKUP_DELAY_MS    300

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

#define TRIG_PIN           22
#define ECHO_PIN           23
#define FRONT_IR_PIN       24
#define RIGHT_IR_PIN       25

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

  pinMode(back_left_enable,   OUTPUT); pinMode(back_left_1,        OUTPUT);
  pinMode(back_left_2,        OUTPUT); pinMode(back_right_enable,  OUTPUT);
  pinMode(back_right_1,       OUTPUT); pinMode(back_right_2,       OUTPUT);
  pinMode(front_right_enable, OUTPUT); pinMode(front_right_1,      OUTPUT);
  pinMode(front_right_2,      OUTPUT); pinMode(front_left_enable,  OUTPUT);
  pinMode(front_left_1,       OUTPUT); pinMode(front_left_2,       OUTPUT);

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
  mpu.update();

  bool frontWall   = (digitalRead(FRONT_IR_PIN) == LOW);
  bool rightWallIR = (digitalRead(RIGHT_IR_PIN) == LOW);
  int  rightDist   = getUltrasonicDistance();

  Serial.print("Front: "); Serial.print(frontWall ? "WALL" : "clear");
  Serial.print(" | RightIR: "); Serial.print(rightWallIR ? "WALL" : "clear");
  Serial.print(" | RightDist: "); Serial.print(rightDist);
  Serial.print(" cm | AngleZ: "); Serial.println(mpu.getAngleZ());

  if (frontWall) {
    Serial.println(">>> ENTERING TURN LEFT");
    stopMotors();
    delay(100);
    moveBackward(BACKUP_SPEED);
    delay(BACKUP_DELAY_MS);
    stopMotors();
    delay(100);
    turnLeftGyro();
    stopMotors();
    delay(100);
    return;
  }

  if ((rightDist > 0 && rightDist < WALL_TOO_CLOSE) || rightWallIR) {
    Serial.println(">>> STEER LEFT");
    steerLeft();
    return;
  }

  if (rightDist >= WALL_TOO_CLOSE && rightDist <= WALL_TOO_FAR) {
    Serial.println(">>> FORWARD");
    moveForward(BASE_SPEED);
    return;
  }

  if (rightDist > WALL_TOO_FAR || rightDist == 0) {
    Serial.println(">>> STEER RIGHT");
    steerRight();
    return;
  }

  delay(20);
}

///////////////////////////////////////////////////////////////////
void turnLeftGyro() {
  mpu.update();
  float startAngle = mpu.getAngleZ();
  unsigned long startTime = millis();

  Serial.print("TURN START — startAngle: "); Serial.println(startAngle);

  setTurnLeftDirections();
  setSpeed(TURN_SPEED, TURN_SPEED, TURN_SPEED, TURN_SPEED);

  while (abs(mpu.getAngleZ() - startAngle) < TURN_TARGET_DEG) {
    if (millis() - startTime > TURN_TIMEOUT_MS) {
      Serial.println("WARN: Turn timeout — gyro may have glitched");
      break;
    }
    mpu.update();
    Serial.print("  angleZ: "); Serial.print(mpu.getAngleZ());
    Serial.print(" | delta: "); Serial.println(abs(mpu.getAngleZ() - startAngle));
  }

  stopMotors();
  Serial.println("Left turn complete");
}

void setTurnLeftDirections() {
  digitalWrite(back_left_1,   LOW);  digitalWrite(back_left_2,   HIGH);
  digitalWrite(front_left_1,  HIGH); digitalWrite(front_left_2,  LOW);
  digitalWrite(back_right_1,  LOW);  digitalWrite(back_right_2,  HIGH);
  digitalWrite(front_right_1, HIGH); digitalWrite(front_right_2, LOW);
}

///////////////////////////////////////////////////////////////////
void moveForward(int spd) {
  digitalWrite(back_left_1,   HIGH); digitalWrite(back_left_2,   LOW);
  digitalWrite(front_left_1,  LOW);  digitalWrite(front_left_2,  HIGH);
  digitalWrite(back_right_1,  LOW);  digitalWrite(back_right_2,  HIGH);
  digitalWrite(front_right_1, HIGH); digitalWrite(front_right_2, LOW);
  setSpeed(spd, spd, spd, spd);
}

void moveBackward(int spd) {
  digitalWrite(back_left_1,   LOW);  digitalWrite(back_left_2,   HIGH);
  digitalWrite(front_left_1,  HIGH); digitalWrite(front_left_2,  LOW);
  digitalWrite(back_right_1,  HIGH); digitalWrite(back_right_2,  LOW);
  digitalWrite(front_right_1, LOW);  digitalWrite(front_right_2, HIGH);
  setSpeed(spd, spd, spd, spd);
}

void steerLeft() {
  moveForward(BASE_SPEED);
  setSpeed(STEER_SPEED_SLOW, STEER_SPEED_FAST, STEER_SPEED_SLOW, STEER_SPEED_FAST);
}

void steerRight() {
  moveForward(BASE_SPEED);
  setSpeed(STEER_SPEED_FAST, STEER_SPEED_SLOW, STEER_SPEED_FAST, STEER_SPEED_SLOW);
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
int getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}