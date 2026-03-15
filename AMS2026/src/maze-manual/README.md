# Manual Maze Navigation

**Worth 20 points.**

## Strategy

Joystick-controlled navigation through the maze using the provided controller.

Based on the AlbertaLoop starter code (v2.0). Joystick X-axis controls left/right, Y-axis controls forward/backward.

## Controls

| Input | Action |
|---|---|
| Y-axis forward (> 550) | Drive forward |
| Y-axis backward (< 470) | Drive backward |
| X-axis right (> 550) | Turn right — left wheels forward, right wheels backward |
| X-axis left (< 470) | Turn left — right wheels forward, left wheels backward |
| Joystick centred | Stop |

## Wiring

| Component | Pins |
|---|---|
| Joystick X-axis | A0 |
| Joystick Y-axis | A1 |

## Tuning notes

- The joystick deadzone is 470–550. If the robot drifts when the joystick is centred, adjust these thresholds.
- Motor directions may need to be swapped depending on chassis orientation — change `HIGH`/`LOW` in the forward/backward blocks for the affected motor.
- Minimum PWM of 72 applied to prevent motor buzzing on carpet without movement.

## How to upload

1. Open `ams_2026_starter_code.ino` in Arduino IDE
2. Board → Arduino Mega 2560
3. Upload — no additional libraries required