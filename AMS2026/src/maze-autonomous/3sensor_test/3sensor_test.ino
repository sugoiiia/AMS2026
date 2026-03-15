///////////////////////////////////////////////////////////////////
//      Albertaloop Mechatronics Showdown 2026
//      Autonomous Maze Navigation
//      Version: 1.0
//
//      Strategy:
//      - Left-hand wall following using ultrasonic sensor
//      - Front collision detection using IR sensor
//      - Right wall detection using IR sensor
//
//      Sensor Wiring:
//      - Ultrasonic TRIG  --> Pin 22
//      - Ultrasonic ECHO  --> Pin 23
//      - Front IR OUT     --> Pin 24
//      - Right IR OUT     --> Pin 25
//
//      All IR sensors: LOW = obstacle detected, HIGH = clear
///////////////////////////////////////////////////////////////////

// ---------------------------------------------------------------
// TUNING PARAMETERS — adjust these during testing
// ---------------------------------------------------------------
#define BASE_SPEED         250   // Default drive speed (0-255). Start here, increase if carpet is too slow.
#define TURN_SPEED         250   // Speed during turns. Slightly higher than base to overcome carpet friction.
#define REVERSE_SPEED      0   // Speed when reversing away from front collision.

#define WALL_FOLLOW_DIST   5.5    // Target distance (cm) to maintain from left wall.
#define WALL_TOO_CLOSE     3     // Left wall too close — steer right.
#define WALL_TOO_FAR       10    // Left wall too far / lost — steer left.
#define FRONT_CLEAR_DIST   20    // (unused — IR is digital, kept for reference)

#define REVERSE_TIME       100   // ms to reverse after front collision before turning.
#define TURN_TIME          1500   // ms to turn right after front collision. Tune this for 90 degrees on carpet.
#define LEFT_TURN_TIME     300   // ms to nudge left when left wall is lost.
// ---------------------------------------------------------------

// --- Motor Pins (from starter code — do not change) ---
#define back_left_enable 7
#define back_left_1 6
#define back_left_2 5
#define back_right_enable 2
#define back_right_1 4
#define back_right_2 3
#define front_right_enable 8
#define front_right_1 9
#define front_right_2 10
#define front_left_enable 13
#define front_left_1 11
#define front_left_2 12

// --- Sensor Pins ---
#define TRIG_PIN           22
#define ECHO_PIN           23
#define FRONT_IR_PIN       24   // LOW = wall detected in front
#define RIGHT_IR_PIN       25   // LOW = wall detected on right

///////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);

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
  delay(2000); // 2 second pause before starting — gives you time to place robot
  Serial.println("Autonomous maze starting...");
}

///////////////////////////////////////////////////////////////////
void loop() {
  bool frontWall = (digitalRead(FRONT_IR_PIN) == LOW);  // IR LOW = wall detected
  bool rightWall = (digitalRead(RIGHT_IR_PIN) == LOW);
  int  leftDist  = getUltrasonicDistance();

  // Debug output — open Serial Monitor to watch live values
  Serial.print("Front: "); Serial.print(frontWall ? "WALL" : "clear");
  Serial.print(" | Right: "); Serial.print(rightWall ? "WALL" : "clear");
  Serial.print(" | Left dist: "); Serial.print(leftDist);
  Serial.println(" cm");

  // ---------------------------------------------------------------
  // DECISION TREE
  // ---------------------------------------------------------------

  // PRIORITY 1: Front wall — stop, reverse, turn right
  // PRIORITY 1: Front wall — stop, turn right in place
  if (frontWall) {
    Serial.println("ACTION: Front wall — stopping and turning right");
    stopMotors();
    delay(500);
    turnRight(TURN_SPEED);
    delay(TURN_TIME);
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
    // Check if right wall present — if so, we're in a corridor, just go straight
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

// Full turn right in place (left wheels forward, right wheels backward)
void turnRight(int spd) {
  // Left side backward
  digitalWrite(back_left_1,   LOW);  digitalWrite(back_left_2,   HIGH);
  digitalWrite(front_left_1,  HIGH); digitalWrite(front_left_2,  LOW);
  // Right side forward
  digitalWrite(back_right_1,  LOW);  digitalWrite(back_right_2,  HIGH);
  digitalWrite(front_right_1, HIGH); digitalWrite(front_right_2, LOW);
  setSpeed(spd, spd, spd, spd);
}

void turnLeft(int spd) {
  // Left side backward (from moveBackward)
  digitalWrite(back_left_1,   LOW);  digitalWrite(back_left_2,   HIGH);
  digitalWrite(front_left_1,  HIGH); digitalWrite(front_left_2,  LOW);
  // Right side forward (from moveForward)
  digitalWrite(back_right_1,  LOW);  digitalWrite(back_right_2,  HIGH);
  digitalWrite(front_right_1, HIGH); digitalWrite(front_right_2, LOW);
  setSpeed(spd, spd, spd, spd);
}

// Gentle steer right — left side faster than right
void steerRight() {
  // Left side forward, right side slower forward
  digitalWrite(back_left_1,   HIGH); digitalWrite(back_left_2,   LOW);
  digitalWrite(back_right_1,  LOW);  digitalWrite(back_right_2,  HIGH);
  digitalWrite(front_left_1,  LOW);  digitalWrite(front_left_2,  HIGH);
  digitalWrite(front_right_1, HIGH); digitalWrite(front_right_2, LOW);
  setSpeed(BASE_SPEED, BASE_SPEED / 2, BASE_SPEED, BASE_SPEED / 2);
}

void steerLeft() {
  // Right side forward, left side slower forward
  digitalWrite(back_left_1,   HIGH); digitalWrite(back_left_2,   LOW);
  digitalWrite(back_right_1,  LOW);  digitalWrite(back_right_2,  HIGH);
  digitalWrite(front_left_1,  LOW);  digitalWrite(front_left_2,  HIGH);
  digitalWrite(front_right_1, HIGH); digitalWrite(front_right_2, LOW);
  setSpeed(BASE_SPEED / 2, BASE_SPEED, BASE_SPEED / 2, BASE_SPEED);
}

void stopMotors() {
  setSpeed(0, 0, 0, 0);
}

// spd values: back_left, back_right, front_left, front_right
void setSpeed(int bl, int br, int fl, int fr) {
  // Enforce minimum threshold from starter code to prevent buzzing
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
  // Send pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo — timeout after 30ms (equivalent to ~500cm, filters bad readings)
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) return 0; // 0 = no reading / out of range

  int distance = duration * 0.034 / 2;
  return distance;
}
