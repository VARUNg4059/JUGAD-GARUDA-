# Drone wiring and MPU9250 mounting

## 1. Core wiring (ESP32 + MPU9250 + ESCs + RX)

- **MPU9250 VCC -> ESP32 3V3**
- **MPU9250 GND -> ESP32 GND**
- **MPU9250 SDA -> ESP32 GPIO21**
- **MPU9250 SCL -> ESP32 GPIO22**
- **MPU9250 AD0 -> GND** (I2C address `0x68`; if AD0 is 3V3 use `0x69` in code)

- **RC throttle signal -> ESP32 GPIO32**
- **RC receiver GND -> ESP32 GND** (common ground is mandatory)
- **RC receiver power** from 5V/BEC only if your receiver supports it

- **ESC FL signal -> ESP32 GPIO25**
- **ESC FR signal -> ESP32 GPIO26**
- **ESC BL signal -> ESP32 GPIO27**
- **ESC BR signal -> ESP32 GPIO14**
- **All ESC grounds -> ESP32 GND** (shared reference)

## 2. Power and grounding rules (important)

- Do **not** power ESP32 3V3 from an ESC BEC output directly unless your regulator path is known safe.
- Keep one clean **common ground** between battery/ESCs/ESP32/receiver/MPU.
- Twist motor leads and keep high-current wires away from MPU wiring.
- Keep I2C wires short and routed away from ESC power lines.

## 3. MPU9250 physical mounting

- Mount MPU close to frame center (near CG), flat and rigid.
- Arrow/front marking on MPU should point to drone front if possible.
- Use vibration damping: thin foam gel + zip tie, not a soft wobbling mount.
- Keep MPU at least a few cm away from power distribution and ESC switching nodes.

### Exact orientation for your current frame (from `drone-with-mpu.jpeg`)

- Front is the side with your pen labels **FL** (front-left) and **FR** (front-right).
- Mount MPU so **Y axis points to the FL/FR side (front)**.
- MPU **X axis points to the drone right side** (toward FR arm).
- MPU **Z axis points up** (component side facing upward).
- Keep the sensor board centered and level with the frame top plate.

## 4. Axis and orientation checks (with current sketch)

With `ORIENTATION_TEST_MODE = true`, motors stay safe and serial prints R/P:

- Tilt **right**: `R` should go **positive**
- Tilt **left**: `R` should go **negative**
- Tilt **nose down (forward)**: `P` should go **negative**
- Tilt **nose up (backward)**: `P` should go **positive**

If any direction is wrong, update in `.ino`:

- `SWAP_XY`
- `INVERT_X`
- `INVERT_Y`
- `INVERT_Z`

Change one flag at a time, reboot, and re-check.

## 5. First safe bring-up sequence

1. Remove props.
2. Flash firmware and open serial at 115200.
3. Verify MPU detection (`WHO_AM_I`) and orientation signs.
4. Verify RC throttle values change correctly.
5. Set `ORIENTATION_TEST_MODE = false` only after orientation is correct.
6. Keep `THRUST_ONLY_MODE = true` for your up/down-only tests.
7. Reinstall props and do short low-altitude hover checks.

## 6. Latest serial observation (your current mounted setup)

- `WHO_AM_I: 0x70` -> MPU is detected correctly.
- Current flashed firmware is still old behavior (it prints `Arming ESCs...`).
- While stationary, drift was seen (`R` about `-3`, `P` about `-4.6`), which is more consistent with calibration/level bias than a 90° axis swap.
- Conclusion: your physical orientation appears consistent with FL/FR frame direction; stabilize further by level calibration and using `ORIENTATION_TEST_MODE` in the updated sketch before flight.
