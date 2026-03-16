///////////////////////////////////////////////////////////////////
//      Albertaloop Mechatronics Showdown 2026
//      Autonomous Maze Navigation - FINAL V4.0
//      Left Wall Follow + Gyro Turns + Backup Clearance
//
//      HOW IT WORKS IN ONE SENTENCE:
//      Every loop, read three sensors → act on the highest-priority
//      condition → when a wall is hit, back up and turn left using
//      the IMU gyroscope to measure exactly 88° of rotation.
///////////////////////////////////////////////////////////////////

#include <Wire.h>           // Built-in Arduino I2C library — required for IMU
#include <MPU6050_light.h>  // Lightweight MPU6050 driver. Handles offset calibration
                            // and angle fusion. Install via Library Manager → "MPU6050_light" by rfetick.

// ─────────────────────────────────────────────────────────────
// TUNING PARAMETERS — all adjustable values live here.
// Change these during testing without touching the logic below.
// ─────────────────────────────────────────────────────────────

#define BASE_SPEED       220  // Normal cruising speed. Safe ceiling for ~11V (series batteries).
                              // 255 is full power — 220 leaves headroom to prevent brownouts.
#define TURN_SPEED       255  // Full power during gyro turns. Carpet friction demands maximum
                              // torque to maintain consistent rotation speed.
#define STEER_SPEED_FAST 255  // Fast side during differential steering
#define STEER_SPEED_SLOW 170  // Slow side during differential steering.
                              // Ratio 255:170 gives a gentle arc rather than a sharp pivot.
#define BACKUP_SPEED     100  // Slow reverse before turning. Fast backup would carry too much
                              // momentum into the turn — robot needs to be nearly still first.

#define WALL_TOO_CLOSE   4    // cm — left wall closer than this → steer away
#define WALL_TOO_FAR     12   // cm — left wall farther than this → steer toward it
                              // Target corridor: 4–12cm from left wall = drive straight

#define TURN_TARGET_DEG  88.0 // Target rotation in degrees. We use 88° not 90° because
                              // rotational momentum carries the robot ~2° after motors stop.
                              // Net result on carpet at TURN_SPEED 255 ≈ true 90°.
#define TURN_TIMEOUT_MS  3000 // Safety exit from turn loop. If IMU freezes or
                              // getAngleZ() stops updating, robot would spin forever.
                              // 3 seconds is generous — a real 88° turn takes ~0.5–1s.
#define BACKUP_DELAY_MS  300  // How long to reverse before turning (ms).
                              // Must be long enough to clear the robot's own turning radius
                              // from the wall it just hit. Too short = robot clips the wall mid-turn.

// ─────────────────────────────────────────────────────────────
// MOTOR DRIVER PINS
// Two L298N H-bridge drivers. Each motor needs:
//   enable (EN) → PWM speed via analogWrite()
//   IN1 + IN2   → direction via digitalWrite()
//     IN1 HIGH + IN2 LOW  = one direction
//     IN1 LOW  + IN2 HIGH = opposite direction
// Note: right-side motors are physically mounted inverted on the
// chassis, so their HIGH/LOW pairs are swapped vs left side.
// ─────────────────────────────────────────────────────────────
#define back_left_enable   7   // PWM speed — back left motor
#define back_left_1        6   // Direction bit A
#define back_left_2        5   // Direction bit B

#define back_right_enable  2   // PWM speed — back right motor
#define back_right_1       4   // Direction bit A
#define back_right_2       3   // Direction bit B

#define front_right_enable 8   // PWM speed — front right motor
#define front_right_1      9   // Direction bit A
#define front_right_2      10  // Direction bit B

#define front_left_enable  13  // PWM speed — front left motor
#define front_left_1       11  // Direction bit A
#define front_left_2       12  // Direction bit B

// ─────────────────────────────────────────────────────────────
// SENSOR PINS
// ─────────────────────────────────────────────────────────────
#define TRIG_PIN     22  // HC-SR04 trigger — send ultrasonic pulse (OUTPUT)
#define ECHO_PIN     23  // HC-SR04 echo — measure return time (INPUT)
#define FRONT_IR_PIN 24  // Front IR obstacle sensor — LOW = wall detected
#define RIGHT_IR_PIN 25  // Left wall IR sensor (named RIGHT for historical reasons,
                         // physically mounted on the left side of the chassis)
                         // LOW = wall detected

MPU6050 mpu(Wire);       // IMU object on I2C bus. SDA → Pin 20, SCL → Pin 21.

///////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);  // Open serial — must match baud rate in Serial Monitor
  Wire.begin();        // Start I2C on Mega hardware pins 20/21

  Serial.println("Initializing MPU6050...");
  byte status = mpu.begin();
  // Wakes IMU from sleep, verifies I2C communication.
  // Returns 0 on success. Any other value = wiring or addressing problem.

  if (status != 0) {
    Serial.print("MPU6050 FAILED! Status: ");
    Serial.println(status);
    while (true);
    // Halt execution. Running the maze without IMU would mean turnLeftGyro()
    // never exits its while loop — robot spins indefinitely.
  }

  Serial.println("MPU6050 OK. Calculating offsets. DO NOT MOVE ROBOT!");
  delay(1000);
  mpu.calcOffsets(true, true);
  // Samples gyro + accelerometer for ~1 second while robot is perfectly still.
  // Measures each axis's resting bias and stores correction values internally.
  // true, true = calibrate BOTH gyro and accelerometer.
  // Any movement during this corrupts offsets → wrong angles all run.
  // This is why setup() waits for "DO NOT MOVE" before proceeding.

  Serial.println("Offsets set.");

  // Set all motor pins as outputs — required before any analogWrite/digitalWrite
  pinMode(back_left_enable,   OUTPUT); pinMode(back_left_1,   OUTPUT); pinMode(back_left_2,   OUTPUT);
  pinMode(back_right_enable,  OUTPUT); pinMode(back_right_1,  OUTPUT); pinMode(back_right_2,  OUTPUT);
  pinMode(front_right_enable, OUTPUT); pinMode(front_right_1, OUTPUT); pinMode(front_right_2, OUTPUT);
  pinMode(front_left_enable,  OUTPUT); pinMode(front_left_1,  OUTPUT); pinMode(front_left_2,  OUTPUT);

  // Set sensor pins
  pinMode(TRIG_PIN,     OUTPUT);  // We send pulses out
  pinMode(ECHO_PIN,     INPUT);   // We listen for the return pulse
  pinMode(FRONT_IR_PIN, INPUT);   // We read HIGH/LOW from IR module
  pinMode(RIGHT_IR_PIN, INPUT);   // We read HIGH/LOW from IR module

  stopMotors();   // Ensure motors are off during setup
  delay(2000);    // 2-second pause after calibration — gives operator time
                  // to step back and place robot at start before it moves
  Serial.println("Autonomous maze starting...");
}

///////////////////////////////////////////////////////////////////
void loop() {
  // ── ALWAYS FIRST: update IMU ─────────────────────────────────
  mpu.update();
  // MPU6050_light does NOT self-update. getAngleZ() returns stale data
  // unless this is called. Must run every loop cycle — including inside
  // the turnLeftGyro() while loop — or angle tracking breaks.

  // ── READ ALL SENSORS ─────────────────────────────────────────
  bool frontWall  = (digitalRead(FRONT_IR_PIN) == LOW);
  // IR modules output LOW when they detect a reflected object.
  // HIGH = clear path. LOW = obstacle within potentiometer-set range.

  bool rightWallIR = (digitalRead(RIGHT_IR_PIN) == LOW);
  // Binary left-wall confirmation. Used in P2 as a secondary check —
  // if ultrasonic misreads but IR is HIGH, we avoid a false steer.

  int rightDist = getUltrasonicDistance();
  // Distance to left wall in cm. Returns 999 if no echo received
  // (open space or sensor timeout). 999 deliberately chosen over 0
  // to avoid ambiguity — 0 could mean "reading zero cm" vs "no echo".

  // ─────────────────────────────────────────────────────────────
  // DECISION TREE — checked top to bottom, first true condition wins.
  // The 'return' at the end of each block is critical — it skips all
  // lower-priority checks and restarts the loop immediately.
  // ─────────────────────────────────────────────────────────────

  // PRIORITY 1: Front wall — highest urgency, always checked first
  if (frontWall) {
    stopMotors();
    delay(100);
    // Brief pause to let forward momentum dissipate before reversing.
    // Without this, robot is still moving forward when backward PWM starts
    // — effective reverse distance is unpredictable.

    moveBackward(BACKUP_SPEED);
    delay(BACKUP_DELAY_MS);
    // Reverse slowly to create clearance for the turn.
    // The robot's turning radius on carpet means if it pivots while
    // touching the wall, the rear swings into the wall behind it.
    // 300ms at BACKUP_SPEED 100 typically clears ~5–8cm.

    stopMotors();
    delay(100);
    // Second pause — settle backward momentum before spinning.
    // A robot still moving backward during a pivot turn will arc
    // unpredictably instead of rotating in place.

    turnLeftGyro();
    // Execute precision 88° left turn using gyro feedback.
    // Does NOT return until turn is complete (or timeout fires).

    stopMotors();
    delay(100);
    return;  // Restart loop from top — re-read all sensors fresh
  }

  // PRIORITY 2: Left wall too close — risk of collision on left side
  if ((rightDist > 0 && rightDist < WALL_TOO_CLOSE) || rightWallIR) {
    // Two conditions combined with OR:
    // - Ultrasonic reads < 4cm: wall dangerously close
    // - IR module triggered: binary confirmation of very close wall
    // Either alone is sufficient to steer away.
    // Note: rightDist > 0 guard prevents triggering on 0 (edge case
    // where sensor returns 0 instead of 999 on timeout).
    steerLeft();
    return;
  }

  // PRIORITY 3: Left wall in target range — ideal navigation state
  if (rightDist >= WALL_TOO_CLOSE && rightDist <= WALL_TOO_FAR) {
    // 4cm ≤ distance ≤ 12cm = wall is exactly where it should be.
    // Drive straight at full cruise speed.
    moveForward(BASE_SPEED);
    return;
  }

  // PRIORITY 4: Left wall lost — need to find it again
  if (rightDist > WALL_TOO_FAR || rightDist == 0) {
    // Distance > 12cm or 0: wall has moved away (junction, open space)
    // or sensor returned edge-case 0. Steer right to reacquire wall.
    // The || rightDist == 0 handles the rare case where pulseIn
    // returns 0 despite the function returning 999 in theory.
    steerRight();
    return;
  }

  delay(20);
  // Small delay at end of loop if no condition fired (shouldn't happen
  // given the four conditions cover all possible rightDist values,
  // but prevents runaway loop speed if logic ever falls through).
}

///////////////////////////////////////////////////////////////////
// GYRO TURN — left, 88 degrees
///////////////////////////////////////////////////////////////////

void turnLeftGyro() {
  mpu.update();
  float startAngle = mpu.getAngleZ();
  // Snapshot the robot's current absolute heading before turning.
  // getAngleZ() returns cumulative yaw in degrees since calcOffsets().
  // We track the DELTA from this starting point, not absolute heading —
  // so it doesn't matter what direction the robot was facing before.

  unsigned long startTime = millis();

  setTurnLeftDirections();
  setSpeed(TURN_SPEED, TURN_SPEED, TURN_SPEED, TURN_SPEED);
  // Set motor directions then apply full speed simultaneously.
  // Separated into two calls because setTurnLeftDirections() only
  // sets digitalWrite pins — speed must be set separately via setSpeed().

  while (abs(mpu.getAngleZ() - startAngle) < TURN_TARGET_DEG) {
    // abs() handles both clockwise and counterclockwise drift in getAngleZ().
    // Loop continues until the robot has rotated at least 88° from start.
    // mpu.update() inside the loop is CRITICAL — without it, getAngleZ()
    // returns the same stale value forever and the loop never exits.

    if (millis() - startTime > TURN_TIMEOUT_MS) {
      Serial.println("WARN: Turn timeout — gyro may have glitched");
      break;
      // Escape hatch. If after 3 seconds the angle hasn't reached 88°,
      // something is wrong (IMU freeze, I2C dropout, stuck motor).
      // Better to exit with a partial turn than spin forever.
    }
    mpu.update();
    // Refresh angle reading every iteration. This is the core of
    // gyro-controlled turning — continuous feedback until target reached.
  }

  stopMotors();
  Serial.println("Left turn complete");
  // Motor momentum will carry ~2° more rotation after stop — combined
  // with the 88° target, this lands consistently at ~90° on carpet.
}

void setTurnLeftDirections() {
  // Left side BACKWARD + right side FORWARD = robot pivots left in place.
  // Both sides spinning opposite directions means zero net linear movement —
  // the robot rotates around its own centre point.

  // Left side backward
  digitalWrite(back_left_1,   LOW);  digitalWrite(back_left_2,   HIGH);
  digitalWrite(front_left_1,  HIGH); digitalWrite(front_left_2,  LOW);
  // Right side forward
  digitalWrite(back_right_1,  LOW);  digitalWrite(back_right_2,  HIGH);
  digitalWrite(front_right_1, HIGH); digitalWrite(front_right_2, LOW);
}

///////////////////////////////////////////////////////////////////
// MOTOR CONTROL
///////////////////////////////////////////////////////////////////

void moveForward(int spd) {
  // All four wheels driving in the same direction = straight line.
  // Left and right sides have inverted HIGH/LOW due to opposing
  // physical mounting orientation on the chassis.

  // Left side forward
  digitalWrite(back_left_1,   HIGH); digitalWrite(back_left_2,   LOW);
  digitalWrite(front_left_1,  LOW);  digitalWrite(front_left_2,  HIGH);
  // Right side forward (inverted pins vs left)
  digitalWrite(back_right_1,  LOW);  digitalWrite(back_right_2,  HIGH);
  digitalWrite(front_right_1, HIGH); digitalWrite(front_right_2, LOW);

  setSpeed(spd, spd, spd, spd);
}

void moveBackward(int spd) {
  // All four wheels reversed from moveForward. Same inversion logic applies.

  // Left side backward
  digitalWrite(back_left_1,   LOW);  digitalWrite(back_left_2,   HIGH);
  digitalWrite(front_left_1,  HIGH); digitalWrite(front_left_2,  LOW);
  // Right side backward
  digitalWrite(back_right_1,  HIGH); digitalWrite(back_right_2,  LOW);
  digitalWrite(front_right_1, LOW);  digitalWrite(front_right_2, HIGH);

  setSpeed(spd, spd, spd, spd);
}

void steerLeft() {
  // Differential steering — all wheels forward, but left side slower.
  // Left side at STEER_SPEED_SLOW (170), right at STEER_SPEED_FAST (255).
  // Right moves faster → robot arcs left. No wheels reverse, so this is
  // a gradual curve rather than a pivot turn.

  moveForward(BASE_SPEED);
  // First call sets all wheel DIRECTIONS to forward.
  // The BASE_SPEED passed here is immediately overridden by setSpeed() below.

  setSpeed(STEER_SPEED_SLOW, STEER_SPEED_FAST, STEER_SPEED_SLOW, STEER_SPEED_FAST);
  // Order: back_left, back_right, front_left, front_right
  // Left pair = SLOW, right pair = FAST → curves left
}

void steerRight() {
  // Mirror of steerLeft — right side slower, curves right.
  moveForward(BASE_SPEED);
  setSpeed(STEER_SPEED_FAST, STEER_SPEED_SLOW, STEER_SPEED_FAST, STEER_SPEED_SLOW);
  // Left pair = FAST, right pair = SLOW → curves right
}

void stopMotors() {
  setSpeed(0, 0, 0, 0);
  // Passing 0 to all four. The deadband check in setSpeed() only
  // activates for values 1–71, so 0 passes through unchanged.
}

void setSpeed(int bl, int br, int fl, int fr) {
  // Enforce motor deadband — any non-zero value below 72 buzzes
  // the motor coils without overcoming static friction on carpet.
  // Discovered at PWM ~72 during testing: motors begin spinning reliably.
  // Snapping low values up to 72 ensures every non-zero command = movement.
  // Values of exactly 0 are left unchanged so stopMotors() still works.
  if (bl > 0 && bl < 72) bl = 72;
  if (br > 0 && br < 72) br = 72;
  if (fl > 0 && fl < 72) fl = 72;
  if (fr > 0 && fr < 72) fr = 72;

  // analogWrite sends a PWM signal to the L298N enable pin.
  // 0 = motor off, 255 = full voltage. Direction already set by IN1/IN2.
  // The L298N's H-bridge uses IN1/IN2 to route current through the motor
  // in the chosen direction at the voltage level set by PWM.
  analogWrite(back_left_enable,   bl);
  analogWrite(back_right_enable,  br);
  analogWrite(front_left_enable,  fl);
  analogWrite(front_right_enable, fr);
}

///////////////////////////////////////////////////////////////////
// ULTRASONIC SENSOR
///////////////////////////////////////////////////////////////////

int getUltrasonicDistance() {
  // HC-SR04 protocol: pull TRIG low briefly to clear it, then pulse
  // HIGH for exactly 10 microseconds to fire an ultrasonic burst.
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  // pulseIn() waits for ECHO to go HIGH (burst sent), measures how long
  // it stays HIGH (burst returning), returns duration in microseconds.
  // 30000µs timeout = ~30ms = max range of ~510cm. Any object beyond
  // that or no echo received returns 0.

  if (duration == 0) return 999;
  // Return 999 (not 0) on timeout. This distinguishes "no wall detected"
  // from a genuine near-zero distance reading. The P4 decision checks
  // rightDist > WALL_TOO_FAR — 999 satisfies this cleanly.
  // If we returned 0, it would also satisfy rightDist == 0 checks
  // but could be confused with a sensor malfunction reading 0cm.

  return duration * 0.034 / 2;
  // Speed of sound = 0.034 cm/µs. Divide by 2 for round trip
  // (pulse travels to wall AND back). Result = distance in cm.
}