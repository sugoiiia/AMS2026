# Autonomous Maze Navigation — Final V4.0

**Worth 30 points** — highest challenge score available.

## Strategy

Left-hand wall following with gyro-controlled turns and backup clearance.

The robot reads three sensors every loop cycle and acts on the highest-priority condition:

| Priority | Condition | Action |
|---|---|---|
| P1 | Front IR triggered | Stop → reverse 450ms → turn left 88° via gyro |
| P2 | Left wall < 4cm OR left IR high | `steerLeft()` — slow left wheels to 100 PWM |
| P3 | Left wall 4–12cm | `moveForward(220)` — wall in range, drive straight |
| P4 | Left wall > 12cm or 999 | `steerRight()` — veer left to reacquire wall |

> Note: The code variable names say "right" for historical reasons, but the robot physically follows the **left wall**. The ultrasonic sensor is mounted on the left side of the chassis.

## Sensor wiring

| Sensor | Pins | Purpose |
|---|---|---|
| HC-SR04 Ultrasonic | TRIG 22, ECHO 23 | Left wall distance |
| IR sensor (front) | Pin 24 | Front collision — LOW = wall |
| IR sensor (left) | Pin 25 | Left wall binary confirm — LOW = wall |
| GY-521 IMU | SDA 20, SCL 21 | Gyro Z-axis for turn measurement |

## Tuning parameters

```cpp
#define BASE_SPEED         220   // Cruise speed — safe for 11V
#define TURN_SPEED         255   // Full power during gyro turn
#define STEER_SPEED_FAST   255   // Fast side during steering
#define STEER_SPEED_SLOW   100   // Slow side during steering
#define BACKUP_SPEED       110   // Reverse speed before turn
#define WALL_TOO_CLOSE     4     // cm — steer away from wall
#define WALL_TOO_FAR       12    // cm — steer toward wall
#define TURN_TARGET_DEG    88.0  // Undershoot — momentum covers the rest
#define TURN_TIMEOUT_MS    3000  // Safety exit if gyro glitches
#define BACKUP_DELAY_MS    450   // ms to reverse before turning
```

## How the gyro turn works

`mpu.calcOffsets()` in setup zeros out manufacturing bias. Every loop calls `mpu.update()` — required by MPU6050_light, it does not auto-update.

When a front wall is detected:
1. Record `startAngle = mpu.getAngleZ()`
2. Spin motors in turn-left configuration
3. Loop calling `mpu.update()` until `abs(getAngleZ() - startAngle) >= 88.0`
4. Stop motors — momentum carries the remaining ~2°

88° is used instead of 90° to account for rotational momentum on carpet.

## Important: ultrasonic returns 999 on timeout

`pulseIn()` returns 0 if no echo received. We return 999 instead to distinguish "no echo" from "zero centimetres." The P4 condition checks `> WALL_TOO_FAR || rightDist == 0` — the `== 0` case catches any edge cases where 0 still slips through.

## How to upload

1. Open `maze_autonomous.ino` in Arduino IDE
2. Install `MPU6050_light` via Library Manager
3. Board → Arduino Mega 2560
4. Upload → open Serial Monitor at 9600 baud
5. Robot will calibrate for ~1 second ("DO NOT MOVE"), then wait 2 seconds before starting