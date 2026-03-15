# AMS 2026 — Albertaloop Mechatronics Showdown

**Team Keiron** · Group 10 · University of Alberta · March 2026

| Member | Role |
|---|---|
| Keiron Street | Electrical, IMU/Wokwi simulation, PID/balance logic |
| Param Vyas | Mechanical, chassis, rewiring |
| Kumaran Vaitheeswaran | Electrical, documentation |
| Aahil Ansarr | Software, integration |
| Alexander Macaloney | Mechanical, chassis |

---

## Challenges & points

| Challenge | Points | Status |
|---|---|---|
| Balance Beam | 15 | ✅ Complete |
| Manual Maze | 20 | ✅ Complete |
| Autonomous Maze | 30 | ✅ Complete |
| Design Evaluation | 60 | ✅ Complete |

---

## Repo structure

```
AMS2026/
├── docs/
│   ├── competition/        # Official challenge doc and marking rubric (PDFs)
│   ├── wiring-diagrams/    # Schematic images
│   └── notes/              # Design decisions, meeting notes
├── presentation/           # Slide deck
├── src/
│   ├── balance-beam/       # Creep-and-react balance beam code
│   ├── maze-manual/        # Starter code — manual joystick control
│   └── maze-autonomous/    # Final V4.0 autonomous maze navigation
└── tests/                  # Sensor calibration and test sketches
```

---

## Hardware

| Component | Detail |
|---|---|
| Controller | Arduino Mega 2560 |
| Drive | 4WD differential — 4× DC yellow motors |
| Motor drivers | 2× L298N H-Bridge |
| Power | 2× 6V AA battery packs (series) + 9V for Arduino |
| Left wall | HC-SR04 Ultrasonic — pins 22/23 |
| Front detect | Digital IR obstacle sensor — pin 24 |
| Right detect | Digital IR obstacle sensor — pin 25 |
| IMU | GY-521 MPU6050 — I2C pins 20/21 |

---

## How to upload code

1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Open the `.ino` file from the relevant `src/` folder
3. Tools → Board → **Arduino Mega 2560**
4. Tools → Port → select your COM port
5. Click **Upload**
6. Tools → Serial Monitor → set baud to **9600**

### Libraries required

Install via Library Manager (Sketch → Include Library → Manage Libraries):

- `MPU6050_light` by rfetick

---

## Key design decisions

**Left-hand wall following** — the maze is simply connected, so left-hand following guarantees completion. The robot uses the HC-SR04 on its left side as the primary wall reference, with the front IR for collision detection and the right IR as secondary confirmation.

**Gyro-controlled turns** — `mpu.getAngleZ()` tracks absolute heading. The robot turns until the delta from its start angle reaches 88°, then stops. This eliminates the drift and inconsistency of time-based turning on carpet.

**MPU6050_light library** — `mpu.calcOffsets()` zeros the sensor on startup, eliminating manufacturing bias. `mpu.update()` must be called every loop cycle — the library does not auto-update.

**PWM deadband of 72** — motors buzz but do not move below this value on ECERF carpet. All `setSpeed()` calls clamp non-zero values to a minimum of 72.

**Series battery configuration** — parallel wiring delivered only 4.6V combined from the 6V packs. Series delivers the full voltage to each motor driver. Arduino powered separately via 9V to avoid brownouts under motor load.

---

## Git workflow

- `main` = stable, working code only
- Branch for any new work: `git checkout -b your-name/what-youre-doing`
- Never push broken code to main the night before competition