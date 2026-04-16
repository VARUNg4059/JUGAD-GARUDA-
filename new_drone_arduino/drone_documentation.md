# Arduino Uno + MPU-6050 Drone Flight Controller — Complete Documentation

## Table of Contents
1. [Hardware Overview](#1-hardware-overview)
2. [Wiring & Circuit Connections](#2-wiring--circuit-connections)
3. [Component Reference](#3-component-reference)
4. [MPU-6050 Mounting & Axis Convention](#4-mpu-6050-mounting--axis-convention)
5. [Motor Layout & Spin Directions](#5-motor-layout--spin-directions)
6. [RC Transmitter & Receiver Channel Mapping](#6-rc-transmitter--receiver-channel-mapping)
7. [Operating Modes](#7-operating-modes)
8. [Startup Sequence (Power-On to Armed)](#8-startup-sequence-power-on-to-armed)
9. [Calibration — How It Works](#9-calibration--how-it-works)
10. [RC Input — Pin Change Interrupts](#10-rc-input--pin-change-interrupts)
11. [IMU Processing — Complementary Filter](#11-imu-processing--complementary-filter)
12. [PID Controller — Roll / Pitch / Yaw](#12-pid-controller--roll--pitch--yaw)
13. [Motor Mixing Table](#13-motor-mixing-table)
14. [Serial Monitor Output — Decoding Guide](#14-serial-monitor-output--decoding-guide)
15. [Safety Features](#15-safety-features)
16. [PID Tuning Guide](#16-pid-tuning-guide)
17. [Troubleshooting](#17-troubleshooting)
18. [Full Pin Reference Table](#18-full-pin-reference-table)

---

## 1. Hardware Overview

| Component | Model / Spec |
|-----------|-------------|
| Flight Controller | Arduino Uno (ATmega328P, 16 MHz) |
| IMU Sensor | MPU-6050 (6-axis: 3-axis accel + 3-axis gyro) |
| RC Transmitter | FlySky FS-CT6B (6-channel, 2.4 GHz AFHDS) |
| RC Receiver | FlySky FS-R6B (6-channel, 2.4 GHz) |
| Custom PCB | "Birdroid Solutions" shield (MPU socket, ESC headers, LED) |
| ESC Protocol | Standard PWM 1000–2000 µs |
| Motor Config | X-configuration quadcopter (4 motors) |

---

## 2. Wiring & Circuit Connections

### 2.1 MPU-6050 → Arduino Uno

```
MPU-6050 Pin    Arduino Uno Pin    Notes
─────────────────────────────────────────────────────
VCC             5V                 MPU-6050 is 5V tolerant on Uno
GND             GND
SDA             A4                 I2C Data (hardware fixed on Uno)
SCL             A5                 I2C Clock (hardware fixed on Uno)
AD0             NOT CONNECTED      Floats LOW internally → address 0x68
INT             NOT CONNECTED      Interrupt not used in this firmware
XDA             NOT CONNECTED      Magnetometer aux (not used)
XCL             NOT CONNECTED      Magnetometer aux (not used)
```

> **I2C Address:** AD0 floating = treated as LOW → I2C address = **0x68**

### 2.2 FlySky FS-R6B Receiver → Arduino Uno

```
Receiver Channel    Arduino Uno Pin    Function
──────────────────────────────────────────────────────────
CH1 (signal wire)   Digital Pin 8      Roll     (right stick L/R)
CH2 (signal wire)   Digital Pin 9      Pitch    (right stick U/D)
CH3 (signal wire)   Digital Pin 10     Throttle (left stick U/D)
CH4 (signal wire)   Digital Pin 11     Yaw      (left stick L/R)
+ (power row)       5V                 Receiver power
- (ground row)      GND               Receiver ground
CH5, CH6            NOT CONNECTED      Unused in this firmware
```

> Each channel connector has 3 pins: **Signal** (top), **+5V** (middle), **GND** (bottom)

### 2.3 Arduino Uno → ESC / Motor Controllers

```
Arduino Uno Pin    ESC / Motor    Position    Spin
──────────────────────────────────────────────────
Digital Pin 4      M1 ESC         Front Left  CW
Digital Pin 5      M2 ESC         Front Right CCW
Digital Pin 6      M3 ESC         Back Right  CW
Digital Pin 7      M4 ESC         Back Left   CCW
```

> Each ESC signal wire plugs into the Arduino digital pin. ESC power (+/−) comes from the LiPo battery through the power distribution board, NOT from the Arduino.

### 2.4 Status LED

```
Arduino Uno Pin    Connection
──────────────────────────────────────────────────
Digital Pin 13     Custom PCB LED control line
                   HIGH = armed and running
                   LOW  = not yet armed / fault
```

> The custom PCB has a built-in current-limiting resistor on the LED circuit. Pin 13 drives it directly.

### 2.5 ASCII Circuit Diagram

```
                    ┌─────────────────────────┐
  LiPo Battery ─── ┤ Power Distribution Board ├─── 5V BEC ──► Receiver VCC
        │           └─────────┬───────────────┘              ► Arduino Vin
        │                     │ (battery voltage)
        │           ┌─────────▼───┐  ┌─────────┐  ┌─────────┐  ┌─────────┐
        └──────────►│  ESC M1(FL) │  │ESC M2FR │  │ESC M3BR │  │ESC M4BL │
                    └──────┬──────┘  └────┬────┘  └────┬────┘  └────┬────┘
                    Signal │        Signal│       Signal│       Signal│
                           │             │             │             │
              ┌────────────▼─────────────▼─────────────▼─────────────▼──────────┐
              │              A R D U I N O    U N O                              │
              │                                                                  │
              │  Digital 4 (FL)   Digital 5 (FR)   Digital 6 (BR)   Digital 7(BL)│
              │                                                                  │
              │  Digital 8 (CH1-Roll)    A4 (SDA) ──────────────► MPU-6050 SDA  │
              │  Digital 9 (CH2-Pitch)   A5 (SCL) ──────────────► MPU-6050 SCL  │
              │  Digital 10 (CH3-Thr)    5V ──────────────────── ► MPU-6050 VCC  │
              │  Digital 11 (CH4-Yaw)    GND ─────────────────── ► MPU-6050 GND  │
              │  Digital 13 (LED)                                                │
              └────────────────────────┬─────────────────────────────────────────┘
                                       │ CH1-CH4 signal wires
                            ┌──────────▼────────────┐
                            │  FlySky FS-R6B (Rx)   │
                            │  CH1 CH2 CH3 CH4 ...  │
                            └──────────┬────────────┘
                                       │ 2.4 GHz RF
                            ┌──────────▼────────────┐
                            │  FlySky FS-CT6B (Tx)  │
                            │  (Handheld Controller) │
                            └───────────────────────┘
```

---

## 3. Component Reference

### Arduino Uno — Key Specs
- CPU: ATmega328P @ 16 MHz
- Flash: 32 KB, SRAM: 2 KB, EEPROM: 1 KB
- Digital I/O: 14 pins (6 PWM capable: 3,5,6,9,10,11)
- Analog Input: 6 pins (A0–A5), A4=SDA, A5=SCL (hardware I2C)
- Pin Change Interrupts: available on all pins (PCINT0–PCINT23)
- Operating voltage: 5V, recommended input: 7–12V via Vin

### MPU-6050 — Key Specs
- 3-axis accelerometer: ±2/4/8/16g selectable
- 3-axis gyroscope: ±250/500/1000/2000°/s selectable
- **Firmware uses:** Accel ±2g (16384 LSB/g), Gyro ±500°/s (65.5 LSB/°/s)
- Digital Low Pass Filter (DLPF): set to 42 Hz to reject motor vibration
- Sample rate: 1 kHz (SMPLRT_DIV = 0)
- Interface: I2C (up to 400 kHz Fast Mode)
- I2C address: 0x68 (AD0 LOW), 0x69 (AD0 HIGH)
- WHO_AM_I register (0x75): returns **0x68** for MPU-6050

### FlySky FS-CT6B Transmitter — Mode 2 Stick Layout
```
Left Stick:                    Right Stick:
  ↑ = Throttle increase          ↑ = Pitch forward (nose down)
  ↓ = Throttle decrease          ↓ = Pitch backward (nose up)
  ← = Yaw left (CCW)             ← = Roll left
  → = Yaw right (CW)             → = Roll right
```

### FlySky FS-R6B Receiver
- 6 channels: CH1–CH6
- Output: standard PWM 1000–2000 µs per channel at ~50 Hz
- Power: 4.0–6.5V via any channel's + pin
- Bind: hold BIND button while powering on

---

## 4. MPU-6050 Mounting & Axis Convention

### Physical Mounting Position
```
         FRONT OF DRONE
               ↑
        ┌──────┼──────┐
        │      Y      │
        │      ↑      │
        │      │      │
        │  ────┼────► X (→ right, toward FR motor)
        │      │      │
        │   MPU-6050  │
        │   (face UP) │
        └─────────────┘
```

- **Y-axis arrow** on the MPU module points toward the **FRONT** of the drone
- **X-axis arrow** on the MPU module points toward **FR (Front Right)** — i.e. to the RIGHT
- The chip face (component side) is **facing UP**
- Z-axis implicitly points **UP** (out of the board, by right-hand rule)

### Axis-to-Motion Mapping
| Axis | Points Toward | Controls | Positive Direction |
|------|--------------|----------|-------------------|
| X | Right (FR) | Pitch rate (gx) | Nose pitches down |
| Y | Front | Roll rate (gy) | Drone rolls right |
| Z | Up | Yaw rate (gz) | Drone rotates CW from above |

### Angle Calculations
```
Roll  = atan2(ax, az) × (180/π)            — left/right tilt in degrees
Pitch = atan2(-ay, √(ax² + az²)) × (180/π) — front/back tilt in degrees
```
- Roll positive = tilting right
- Pitch positive = nose up

### No Axis Remapping Required
Because the MPU is mounted in standard orientation (Y→front, X→right, face up), no SWAP_XY or INVERT flags are needed in the firmware. The gyro and accel axes naturally map to the correct roll/pitch/yaw axes.

---

## 5. Motor Layout & Spin Directions

```
                    FRONT
           FL (M1)       FR (M2)
            Pin 4         Pin 5
              ↗ CW    CCW ↖
                  \    /
                   \  /
                    \/  ← Center of drone
                    /\
                   /  \
                  /    \
              ↙ CCW    CW ↘
            Pin 7         Pin 6
           BL (M4)       BR (M3)
                    BACK
```

| Motor | Label | Arduino Pin | Position | Spin | Role |
|-------|-------|-------------|----------|------|------|
| M1 | FL | Pin 4 | Front Left | **CW** | Counter yaw CCW |
| M2 | FR | Pin 5 | Front Right | **CCW** | Counter yaw CW |
| M3 | BR | Pin 6 | Back Right | **CW** | Counter yaw CCW |
| M4 | BL | Pin 7 | Back Left | **CCW** | Counter yaw CW |

> **Why alternating spin?** CW and CCW motors produce equal and opposite torque, so at equal throttle the drone doesn't rotate (yaw). The PID can then spin CW motors up slightly or CCW motors down to control heading.

---

## 6. RC Transmitter & Receiver Channel Mapping

| Channel | Receiver Pin | Arduino Pin | Stick | Function | Range |
|---------|-------------|-------------|-------|----------|-------|
| CH1 | CH1 | Pin 8 | Right L/R | Roll | 1000–2000 µs |
| CH2 | CH2 | Pin 9 | Right U/D | Pitch | 1000–2000 µs |
| CH3 | CH3 | Pin 10 | Left U/D | Throttle | 1000–2000 µs |
| CH4 | CH4 | Pin 11 | Left L/R | Yaw | 1000–2000 µs |

> **Note:** This firmware currently uses **CH3 (Throttle) only** for motor speed in stabilisation mode. CH1/CH2/CH4 values are read and printed but not yet used to command attitude setpoints (i.e., you can't steer — the drone only self-levels). Setpoint inputs can be added once PID gains are tuned.

---

## 7. Operating Modes

Set these `#define` values at the **top of the .ino file** before uploading:

### Mode 1: ORIENTATION_TEST_MODE = true
- Motors stay at 1000 µs (completely off)
- Calibration still runs (for accurate display)
- Serial prints IMU angles and RC values every 100 ms
- Use this to verify MPU wiring and axis directions are correct
- **Expected behaviour:**
  - Tilt RIGHT → Roll increases (positive)
  - Tilt LEFT → Roll decreases (negative)
  - Tilt NOSE DOWN → Pitch decreases (negative)
  - Tilt NOSE UP → Pitch increases (positive)

### Mode 2: THRUST_ONLY_MODE = true, ORIENTATION_TEST_MODE = false
- Calibration runs normally
- ESCs arm (1000 µs for 3 seconds)
- ALL motors receive identical throttle from CH3
- PID completely bypassed — no stabilisation
- Use this to test motors and ESC response safely with no feedback
- **Safety:** throttle below 1050 µs stops all motors

### Mode 3: Both = false (FULL STABILISATION — Normal Flight)
- Calibration runs
- ESCs arm
- PID loop runs continuously
- Drone self-levels using roll and pitch PID
- Yaw rate is held at zero (heading hold)
- Throttle below 1050 µs → all motors stop + integrals reset

---

## 8. Startup Sequence (Power-On to Armed)

```
Power ON
    │
    ▼
Serial monitor starts (115200 baud)
    │
    ▼
Pin 13 (LED) = LOW
    │
    ▼
Pin Change Interrupts enabled (pins 8,9,10,11)
    │
    ▼
I2C started at 400 kHz (SDA=A4, SCL=A5)
    │
    ▼
Wait 200 ms (MPU-6050 power-up)
    │
    ▼
Read WHO_AM_I register (expect 0x68)
    │    ╔══════╗
    ├─NO─╢ 0x68?╠─► HALT + blink LED fast
    │    ╚══════╝         (check wiring!)
    │YES
    ▼
initMPU() — configure DLPF, sample rate, gyro/accel scale
    │
    ▼
calibrateMPU() — 2000 samples × 2 ms = ~4 seconds
    │             drone must be flat and still!
    ▼
Compute rollOffset, pitchOffset
    │
    ▼
Attach motor Servos (immediately outputs 1000 µs)
    │
    ▼
ORIENTATION_TEST_MODE?
    ├─YES─► skip armESCs(), LED ON, enter test loop
    │
    └─NO──► armESCs() — holds 1000 µs for 3 seconds
                │        (ESC beeps during this)
                ▼
            LED = HIGH (Pin 13)
                │
                ▼
            Enter main flight loop
```

---

## 9. Calibration — How It Works

### Purpose
The calibration establishes the drone's **zero reference** — the attitude that the PID considers "perfectly level." Wherever the drone is placed when it powers on, that surface becomes the zero point.

### Process
1. 2000 IMU readings are taken over ~4 seconds (2 ms between each)
2. For each reading, roll and pitch angles are calculated using:
   ```
   rollSample  = atan2(ax, az) × (180/π)
   pitchSample = atan2(-ay, √(ax²+az²)) × (180/π)
   ```
3. All samples are averaged:
   ```
   rollOffset  = sum of all rollSamples  / 2000
   pitchOffset = sum of all pitchSamples / 2000
   ```
4. These offsets are subtracted in every subsequent loop iteration

### In the Flight Loop
```
accRoll  = atan2(ax, az) × (180/π) − rollOffset
accPitch = atan2(-ay, √(ax²+az²)) × (180/π) − pitchOffset
```

### Why This Is Good Enough
- Removes the physical tilt of the mounting surface
- Removes any permanent tilt from the MPU being slightly un-level on the custom PCB
- Removes any small sensor bias at rest
- 2000 samples averages out thermal noise and random offset

### Requirement
**The drone must be completely still and on the intended flying surface during calibration.** Any movement during the 4-second window will corrupt the offsets and cause the drone to drift.

---

## 10. RC Input — Pin Change Interrupts

### Why Not pulseIn()?
The original ESP32 code used `pulseIn()` which **blocks the CPU** for up to 25 ms per channel. With 4 channels that's up to 100 ms of blocking per loop — at 50 Hz RC, the drone would miss sensor updates and PID cycles.

### How Pin Change Interrupts Work
The ATmega328P has **Pin Change Interrupt** hardware. Any logic-level change on a monitored pin fires the `ISR(PCINT0_vect)` function instantly, even in the middle of other code.

### Implementation
```
Pins 8–11 are on PORTB (PB0–PB3)
PCICR  |= (1 << PCIE0)   — enable PORTB change interrupt group
PCMSK0 |= bits for PB0,PB1,PB2,PB3  — enable specific pins
```

When any pin changes:
1. `micros()` is recorded
2. The pin that changed is identified by comparing current vs previous PORTB state
3. **Rising edge** (0→1): save the timestamp as `rcRiseTime[ch]`
4. **Falling edge** (1→0): pulse width = `now - rcRiseTime[ch]`, stored if 900–2100 µs

### Atomic Read in Main Loop
Because `rcRaw[]` is written by an ISR and read by the main loop, a brief `noInterrupts()` / `interrupts()` block ensures a consistent read (no torn read across 16-bit values on 8-bit AVR).

---

## 11. IMU Processing — Complementary Filter

### Raw Data
Each loop reads 14 bytes burst from register 0x3B:
```
Bytes 0–1:  Accel X (ax)
Bytes 2–3:  Accel Y (ay)
Bytes 4–5:  Accel Z (az)
Bytes 6–7:  Temperature (ignored)
Bytes 8–9:  Gyro X (gx)
Bytes 10–11: Gyro Y (gy)
Bytes 12–13: Gyro Z (gz)
```

### Gyro Rate Conversion
```
Gyro ±500°/s setting → sensitivity = 65.5 LSB per °/s
gyroRoll  = gy / 65.5   (°/s, rotation around Y-axis = roll)
gyroPitch = gx / 65.5   (°/s, rotation around X-axis = pitch)
gyroYaw   = gz / 65.5   (°/s, rotation around Z-axis = yaw)
```

### Complementary Filter
```
roll  = 0.98 × (roll  + gyroRoll  × dt) + 0.02 × accRoll
pitch = 0.98 × (pitch + gyroPitch × dt) + 0.02 × accPitch
yaw  += gyroYaw × dt   (no magnetometer, gyro only)
```

- **98% gyro:** Fast, low-noise, tracks rapid changes well — but drifts over time
- **2% accelerometer:** Slow but absolute reference — corrects gyro drift
- **Balance:** 0.98/0.02 is typical for ~400 Hz loop rate; tune if loop rate changes significantly

---

## 12. PID Controller — Roll / Pitch / Yaw

### Structure (same for all three axes)
```
error    = setpoint − measured_value
integral = constrain(integral + error × dt, −200, +200)
derivative = (error − prev_error) / dt
PID_output = Kp × error + Ki × integral + Kd × derivative
```

### Setpoints
| Axis | Setpoint | What it means |
|------|----------|--------------|
| Roll | 0° | Drone should be perfectly level left/right |
| Pitch | 0° | Drone should be perfectly level front/back |
| Yaw | 0 °/s | Drone should have zero rotation rate (heading hold) |

### Default PID Gains
| Axis | Kp | Ki | Kd |
|------|----|----|-----|
| Roll | 1.5 | 0.0 | 0.04 |
| Pitch | 1.5 | 0.0 | 0.04 |
| Yaw | 2.0 | 0.0 | 0.0 |

> Ki = 0 initially. Enable only after Kp and Kd are tuned well.

### Anti-Windup
`INTEGRAL_LIMIT = 200` — the integral is clamped to ±200 to prevent runaway when the error is sustained (e.g., drone stuck against something).

---

## 13. Motor Mixing Table

```
X-Configuration mixing (all values in µs, added to base throttle):

         + Roll (right)   − Roll (left)
         ─────────────────────────────────
+Pitch   FL = Thr+P+R-Y   FR = Thr+P-R+Y
(nose up)
−Pitch   BL = Thr-P+R+Y   BR = Thr-P-R-Y
(nose dn)
```

Full formula:
```
mFL = throttle + pitchPID + rollPID - yawPID
mFR = throttle + pitchPID - rollPID + yawPID
mBR = throttle - pitchPID - rollPID - yawPID
mBL = throttle - pitchPID + rollPID + yawPID
```

All values constrained to 1000–2000 µs.

### Logic Verification
| Situation | Error Sign | PID Sign | Motor Response | Result |
|-----------|-----------|----------|---------------|--------|
| Tilts RIGHT (roll > 0) | rollError < 0 | rollPID < 0 | FL,BL get more power | Levels right side ✓ |
| Tilts LEFT (roll < 0) | rollError > 0 | rollPID > 0 | FR,BR get more power | Levels left side ✓ |
| Nose DOWN (pitch < 0) | pitchError > 0 | pitchPID > 0 | FL,FR get more power | Lifts nose ✓ |
| Nose UP (pitch > 0) | pitchError < 0 | pitchPID < 0 | BL,BR get more power | Pushes nose down ✓ |
| Yaw CW (gz > 0) | yawError < 0 | yawPID < 0 | FL,BR get more (CW motors) | Counters CW rotation ✓ |

---

## 14. Serial Monitor Output — Decoding Guide

### Setup: 115200 baud, no line ending required

### Mode: STABILISATION (normal flight)
```
[STAB] R:0.12 P:-0.34 Y:2.10 dt:2.45ms rPID:0.18 pPID:-0.51 yPID:0.00 FL:1320 FR:1285 BR:1310 BL:1345 Thr:1300
```
| Field | Meaning | Good Range |
|-------|---------|-----------|
| R | Roll angle (°) | ±2° when hovering level |
| P | Pitch angle (°) | ±2° when hovering level |
| Y | Yaw angle (°, cumulative) | drifts slowly if no correction |
| dt | Loop time in milliseconds | 2–5 ms typical |
| rPID | Roll PID output (µs correction) | ±50 when level |
| pPID | Pitch PID output (µs correction) | ±50 when level |
| yPID | Yaw PID output (µs correction) | ≈0 when still |
| FL/FR/BR/BL | Actual µs sent to each motor | 1000–2000 µs |
| Thr | Raw throttle from CH3 receiver | 1000–2000 µs |

### Mode: THRUST ONLY
```
[THST] Thr:1450 FL:1450 FR:1450 BR:1450 BL:1450 R:0.10 P:-0.22
```
All motors get same value. R/P shown for reference only.

### Mode: ORIENTATION TEST
```
[TEST] R:0.12 P:-0.34 Yaw:0.00 Gx:0.10 Gy:-0.05 Gz:0.00 CH1:1500 CH2:1500 CH3(Thr):1000 CH4:1500
```
| Field | Meaning |
|-------|---------|
| Gx | Pitch gyro rate (°/s) |
| Gy | Roll gyro rate (°/s) |
| Gz | Yaw gyro rate (°/s) |
| CH1–CH4 | Raw RC pulse widths (µs) |

### Status / Event Messages
| Message | Meaning |
|---------|---------|
| `[MPU] MPU-6050 detected OK!` | WHO_AM_I = 0x68, chip found |
| `[MPU] *** ERROR: MPU-6050 NOT FOUND! ***` | Wiring fault — check SDA/SCL/VCC/GND |
| `[CAL] Progress: X%` | Calibration running, X% done |
| `[CAL] === CALIBRATION COMPLETE ===` | Offsets computed, safe to proceed |
| `[ARM] Sending 1000us to all ESCs...` | ESC arming beep sequence |
| `[ARM] ESCs ARMED` | Ready to accept throttle |
| `[DISARMED] Throttle=1000us...` | Throttle below 1050 µs, motors off |
| `[I2C] Write error reg=0x...` | I2C communication problem |

### Diagnosing Problems from Serial Output
| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| WHO_AM_I = 0xFF | SDA/SCL shorted or wrong wire | Check A4/A5 connections |
| WHO_AM_I = 0x00 | MPU not powered | Check 5V and GND |
| Roll/Pitch wildly jumping during still | Bad calibration (drone moved) | Reset and keep still |
| dt > 20ms | I2C timeout or Wire hangup | Check connections, reduce baud rate |
| All motors same despite tilt | THRUST_ONLY_MODE = true | Set to false and re-upload |
| CH3 always 1000 | RC not bound to transmitter | Bind transmitter and receiver |

---

## 15. Safety Features

| Safety Feature | How It Works |
|---------------|-------------|
| MPU detection halt | If WHO_AM_I ≠ 0x68 at startup, code halts and blinks LED — motors never arm |
| Calibration before arm | `calibrateMPU()` completes before `motorX.attach()` is called — ESC signal line not even active yet |
| Motors off during calibration | `Servo.attach()` called AFTER calibration — no accidental spin |
| ESC arming at 1000 µs | 3-second hold at 1000 µs before any throttle accepted |
| Throttle gate (ARMED_THRESHOLD) | If CH3 < 1050 µs at any time, all motors go to 1000 µs and integrals reset |
| RC signal validation | Only pulses 900–2100 µs accepted; outside range defaults to last known good |
| Integral anti-windup | Integral clamped to ±200 prevents runaway in sustained error |
| dt guard | Loop dt > 500 ms or ≤ 0 is replaced with 10 ms to prevent PID spike |
| Non-blocking RC read | Pin Change ISR — RC failure never freezes the PID loop |

---

## 16. PID Tuning Guide

### Golden Rule: Always test with props OFF first

### Step 1 — Verify orientation (ORIENTATION_TEST_MODE = true)
- Tilt right → R increases ✓
- Tilt left  → R decreases ✓
- Tilt nose down → P decreases ✓
- Tilt nose up   → P increases ✓

### Step 2 — Motor test (THRUST_ONLY_MODE = true)
- Slowly raise throttle — all motors spin up evenly
- All should start at roughly same throttle
- Listen for any motor significantly louder/quieter

### Step 3 — Tune Kp Roll/Pitch (THRUST_ONLY_MODE = false, props OFF)
- Start with Kp = 0.5, Ki = 0, Kd = 0
- Hold drone and tilt it — motors on the low side should spin faster
- Increase Kp until response is snappy but not oscillating
- Typical final value: 1.0–2.5

### Step 4 — Tune Kd Roll/Pitch
- Kd damps oscillation
- Add small Kd (0.01–0.1) — too high causes high-frequency buzz
- Typical: 0.03–0.08

### Step 5 — Tune Ki Roll/Pitch (props ON, careful!)
- Ki removes steady-state drift (drone that always leans one way)
- Start at 0.01, increase slowly
- Too high causes slow oscillation (integral windup)
- Typical: 0.01–0.05

### Step 6 — Tune Kp/Kd Yaw
- Current yaw is heading hold (zero rate), not angle control
- Kp_yaw = 2.0 is a starting point — may need less

---

## 17. Troubleshooting

### "MPU-6050 NOT FOUND" at startup
1. Check SDA → A4, SCL → A5
2. Check VCC → 5V, GND → GND
3. Verify AD0 is floating (or grounded), not accidentally pulled high
4. Try different I2C speed: change `Wire.setClock(400000)` to `Wire.setClock(100000)`
5. Check for solder bridges on the custom PCB

### Drone spins on the spot (yaw drift)
- Props not balanced
- Motor speeds not matched — use THRUST_ONLY_MODE to check
- Kp_yaw too low or prop directions wrong

### Drone always tilts one direction even after calibration
- IMU calibration was run on a non-level surface
- Roll/pitch offsets are incorrect — reset and recalibrate on a level surface
- Add small Ki (0.02) to correct steady-state error

### RC channels read as 1000 even with transmitter on
1. Bind the receiver to the transmitter (hold BIND button on Rx while powering on with Tx in bind mode)
2. Check signal wire from Rx CH3 to Arduino pin 10
3. Verify Rx has 5V power

### Motors start at wrong throttle or jerk
- ESC arming failed — power cycle everything
- Some ESCs need a specific throttle calibration sequence — check ESC manual

### Loop dt is very high (>10ms)
- I2C reading is slow — check SDA/SCL connections and pullup resistors
- Reduce Serial print rate in code

---

## 18. Full Pin Reference Table

| Arduino Uno Pin | Connected To | Signal Type | Notes |
|----------------|-------------|------------|-------|
| A4 (SDA) | MPU-6050 SDA | I2C Data | Hardware I2C, 400 kHz |
| A5 (SCL) | MPU-6050 SCL | I2C Clock | Hardware I2C, 400 kHz |
| D4 | M1 FL ESC | PWM 1000–2000 µs | Front Left, CW |
| D5 | M2 FR ESC | PWM 1000–2000 µs | Front Right, CCW |
| D6 | M3 BR ESC | PWM 1000–2000 µs | Back Right, CW |
| D7 | M4 BL ESC | PWM 1000–2000 µs | Back Left, CCW |
| D8 | RC CH1 | PWM input ~50 Hz | Roll, PCINT0 |
| D9 | RC CH2 | PWM input ~50 Hz | Pitch, PCINT1 |
| D10 | RC CH3 | PWM input ~50 Hz | Throttle, PCINT2 |
| D11 | RC CH4 | PWM input ~50 Hz | Yaw, PCINT3 |
| D13 | Custom PCB LED | Digital HIGH/LOW | HIGH = armed/running |
| 5V | MPU-6050 VCC | Power | Also receiver + pin |
| GND | MPU-6050 GND | Ground | Also receiver − pin |

---

*Generated for: Arduino Uno + MPU-6050 + FlySky FS-CT6B/R6B Quadcopter*
*Firmware version: 2.0 (Uno port, MPU-6050, PCINT RC, full safety)*
