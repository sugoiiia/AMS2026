# Autonomous Maze Navigation

## Challenge
Navigate the AMS 2026 maze autonomously. Worth **30 points** — the highest challenge score available.

## Hardware Setup
| Sensor | Pin | Notes |
|---|---|---|
| Ultrasonic TRIG | 22 | Left wall distance |
| Ultrasonic ECHO | 23 | Left wall distance |
| Front IR sensor | 24 | LOW = wall detected |
| Right IR sensor | 25 | LOW = wall detected |

## Algorithm
Left-hand wall following with dual IR collision detection:
1. **Front wall detected** → stop, reverse, turn right
2. **Left wall too close (<6cm)** → steer right
3. **Left wall in range (6-15cm)** → drive straight
4. **Left wall lost (>15cm)** → nudge left to reacquire. If right wall present, drive straight.

## Tuning Guide
All tunable values are at the top of `maze_autonomous.ino` under **TUNING PARAMETERS**.

- `BASE_SPEED` — start at 150, increase if robot is too slow on carpet
- `TURN_TIME` — increase if robot doesn't complete a full 90° turn
- `WALL_FOLLOW_DIST` — target distance from left wall in cm
- `REVERSE_TIME` — how long to back up before turning away from front wall

## How to Upload
1. Open `maze_autonomous.ino` in Arduino IDE
2. Select **Board: Arduino Mega 2560**
3. Select the correct **Port**
4. Click **Upload**
5. Open **Serial Monitor** at 9600 baud to watch live sensor readings
