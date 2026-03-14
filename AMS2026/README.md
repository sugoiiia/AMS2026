# AMS 2026 — Albertaloop Mechatronics Showdown

## Team
**[Your Team Name]** — University of Alberta

## Competition Overview
Three challenges + design evaluation. Total: 125 possible points.

| Challenge | Points | Status |
|---|---|---|
| Balance Beam | 15 | 🔧 In Progress |
| Manual Maze | 20 | 🔧 In Progress |
| Autonomous Maze | 30 | ✅ Code Ready |
| Design Evaluation | 60 | 🔧 In Progress |

## Repo Structure
```
AMS2026/
├── docs/
│   ├── competition/       # Official challenge docs and rubric
│   ├── wiring-diagrams/   # Schematic images
│   └── notes/             # Design decisions, meeting notes
├── presentation/          # Slide deck
├── src/
│   ├── balance-beam/      # Challenge 1 code
│   ├── maze-manual/       # Challenge 2 — manual controller code
│   └── maze-autonomous/   # Challenge 3 — autonomous navigation code
└── tests/                 # Sensor calibration and test sketches
```

## Hardware
- Arduino Mega 2560
- 2x L298N H-Bridge Motor Driver
- 4x DC Yellow Motors (4WD differential drive)
- HC-SR04 Ultrasonic Sensor (left wall)
- 2x Digital IR Obstacle Sensors (front + right)
- GY-521 IMU (balance beam)
- 2x 6V AA battery packs

## How to Upload Code
1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Open the `.ino` file from the relevant `src/` folder
3. Go to **Tools → Board → Arduino Mega 2560**
4. Go to **Tools → Port** and select your COM port
5. Click the **Upload** button (→)
6. Open **Serial Monitor** (Tools → Serial Monitor) at **9600 baud** to debug

## Git Workflow (keep it simple)
- `main` branch = stable, working code only
- Create your own branch before making changes: `git checkout -b your-name/what-youre-doing`
- When it works, ask a teammate to review, then merge to main
- **Never push broken code to main the night before competition**

## Tuning Notes
See the README inside each `src/` subfolder for challenge-specific tuning instructions.
