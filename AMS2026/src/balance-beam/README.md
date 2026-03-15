# Balance Beam — Creep and React

**Worth 15 points.**

## Strategy

No PID. A simple state machine that creeps forward, reads tilt angle, and reacts.

The robot drives forward onto the beam until it finds the balance point, then holds position for 5 seconds to score.

## States

| State | Condition | Action |
|---|---|---|
| Tipping forward (emergency) | angle > +3.0° | Reverse immediately |
| Tipping back (emergency) | angle < -9.0° | Drive forward immediately |
| Balanced | abs(error) ≤ 1.5° | Stop motors, start 5s hold timer |
| Creep forward | error > threshold | Drive forward slowly |
| Creep back | error < -threshold | Reverse slowly |

## Tuning parameters

```cpp
#define CREEP_SPEED       90     // Very slow — increase if too sluggish on beam
#define REVERSE_SPEED     90     // Reverse speed for corrections
#define BALANCE_ANGLE    -2.5    // Midpoint of measured beam range (2° to -7°)
#define TILT_THRESHOLD    1.5    // Degrees from target before reacting
#define TIPPING_FORWARD   3.0    // Emergency reverse threshold
#define TIPPING_BACK     -9.0    // Emergency forward threshold
#define TILT_SIGN         1      // Set to -1 if nose-down gives negative angle
#define HOLD_TIME_MS   5000      // Must hold for 5 seconds to score
```

## Calibration steps

1. Upload and open Serial Monitor at 9600 baud
2. Tilt robot nose-down by hand — angle should go **positive**
   - If it goes negative, change `TILT_SIGN` to `-1`
3. `BALANCE_ANGLE` is set to -2.5° — the measured midpoint of the beam range
4. Place robot at start position (back wheels off beam edge)
5. Robot calibrates for 1 second ("DO NOT MOVE"), then starts after 2 second delay

## Sensor wiring

| Component | Pins |
|---|---|
| GY-521 IMU SDA | Pin 20 |
| GY-521 IMU SCL | Pin 21 |

## Library required

`MPU6050_light` by rfetick — install via Library Manager.

Uses `mpu.getAngleY()` for pitch (nose-up/nose-down rotation axis).