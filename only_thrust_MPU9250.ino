#include <Wire.h>
#include <ESP32Servo.h>

// ─────────────────────────────────────────
//  MPU-9250 I2C address
//  AD0 LOW  → 0x68  (default, AD0 = GND)
//  AD0 HIGH → 0x69  (AD0 = 3.3V)
// ─────────────────────────────────────────
#define MPU9250_ADDR   0x68

// ─────────────────────────────────────────
//  Flight mode & RC input
// ─────────────────────────────────────────
// true  = throttle-only testing (all motors same output)
// false = full stabilization PID mode
#define THRUST_ONLY_MODE  false

// true  = do not arm ESCs and only print orientation telemetry
// false = normal flight loop
#define ORIENTATION_TEST_MODE  false

// RC throttle PWM input pin (receiver signal wire)
#define RC_THROTTLE_PIN   32

// ── MPU-9250 Register Map ─────────────────
#define WHO_AM_I       0x75
#define PWR_MGMT_1     0x6B
#define SMPLRT_DIV     0x19
#define CONFIG         0x1A   // DLPF config
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_XOUT_H   0x3B   // 14 bytes: Accel + Temp + Gyro

// ─────────────────────────────────────────
//  Angle & offset state
// ─────────────────────────────────────────
float roll  = 0, pitch = 0, yaw = 0;
float rollOffset = 0, pitchOffset = 0;
bool calibrationComplete = false;

// ─────────────────────────────────────────
//  MPU mounting correction
//  Set these based on your physical module orientation.
// ─────────────────────────────────────────
#define SWAP_XY   false
#define INVERT_X  false
#define INVERT_Y  false
#define INVERT_Z  false

// ─────────────────────────────────────────
//  Motor Pins  —  X-Configuration
//
//    FL(25) ↖   ↗ FR(26)
//            \ /
//            / \
//    BL(27) ↙   ↘ BR(14)
// ─────────────────────────────────────────
#define MOTOR_FL_PIN  25
#define MOTOR_FR_PIN  26
#define MOTOR_BL_PIN  27
#define MOTOR_BR_PIN  14

Servo motorFL, motorFR, motorBL, motorBR;

// ─────────────────────────────────────────
//  PID Constants
//  ⚠ Start with kp = 0.5 on bench (props
//  OFF) and tune up slowly.
// ─────────────────────────────────────────
float kp_roll  = 1.5,  ki_roll  = 0.0,  kd_roll  = 0.04;
float kp_pitch = 1.5,  ki_pitch = 0.0,  kd_pitch = 0.04;
float kp_yaw   = 2.0,  ki_yaw   = 0.0,  kd_yaw   = 0.0;

float rollError,  rollPrevError  = 0, rollIntegral  = 0;
float pitchError, pitchPrevError = 0, pitchIntegral = 0;
float yawError,   yawPrevError   = 0, yawIntegral   = 0;

// ─────────────────────────────────────────
//  Throttle & Timing
// ─────────────────────────────────────────
int throttle = 1000;
unsigned long prevTime = 0;

// ─────────────────────────────────────────
//  Safety constants
// ─────────────────────────────────────────
#define MIN_THROTTLE    1000
#define MAX_THROTTLE    2000
#define ARMED_THRESHOLD 1050
#define INTEGRAL_LIMIT   200

// ══════════════════════════════════════════
//  Low-level I2C helpers
// ══════════════════════════════════════════
void writeByte(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readByte(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.read();
}

void readBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, count);
  for (uint8_t i = 0; i < count; i++) {
    dest[i] = Wire.read();
  }
}

int16_t toInt16(uint8_t high, uint8_t low) {
  return (int16_t)((high << 8) | low);
}

void applyAxisMapping(float &ax, float &ay, float &az,
                      float &gx, float &gy, float &gz) {
  if (SWAP_XY) {
    float tA = ax; ax = ay; ay = tA;
    float tG = gx; gx = gy; gy = tG;
  }
  if (INVERT_X) { ax = -ax; gx = -gx; }
  if (INVERT_Y) { ay = -ay; gy = -gy; }
  if (INVERT_Z) { az = -az; gz = -gz; }
}

int readThrottleUs() {
  if (!calibrationComplete) return MIN_THROTTLE;
  // 25ms timeout covers 50Hz RC frame; invalid/missing signal -> safe minimum.
  unsigned long pulse = pulseIn(RC_THROTTLE_PIN, HIGH, 25000);
  if (pulse >= 900 && pulse <= 2100) {
    return constrain((int)pulse, MIN_THROTTLE, MAX_THROTTLE);
  }
  return MIN_THROTTLE;
}

// ══════════════════════════════════════════
//  MPU-9250 Initialisation
//  Mirrors the MPU6050 settings from the
//  original file as closely as possible:
//    • Gyro  ±500 °/s  (GYRO_CONFIG = 0x08)
//    • Accel ±2g       (ACCEL_CONFIG = 0x00)
//    • DLPF  ~42 Hz    (CONFIG = 0x03)
//    • Sample rate = 1 kHz / (1+0) = 1 kHz
// ══════════════════════════════════════════
void initMPU() {
  // Wake up — clear sleep bit, use internal 20MHz oscillator
  writeByte(MPU9250_ADDR, PWR_MGMT_1, 0x00);
  delay(100);

  // PLL from gyro (best clock stability) — same as original
  writeByte(MPU9250_ADDR, PWR_MGMT_1, 0x01);
  delay(10);

  // DLPF bandwidth ~42Hz (CONFIG bits[2:0] = 3) — filters motor vibration
  writeByte(MPU9250_ADDR, CONFIG, 0x03);

  // Sample rate divider = 0 → 1kHz output rate
  writeByte(MPU9250_ADDR, SMPLRT_DIV, 0x00);

  // Gyro ±500 °/s → sensitivity = 65.5 LSB/(°/s)
  writeByte(MPU9250_ADDR, GYRO_CONFIG, 0x08);

  // Accel ±2g → sensitivity = 16384 LSB/g
  writeByte(MPU9250_ADDR, ACCEL_CONFIG, 0x00);

  delay(50);
}

// ══════════════════════════════════════════
//  Calibration
//  ⚠ Keep drone perfectly flat and still.
//
//  Axis convention (right-side-up, Y = front):
//    Roll  → tilt along X-axis (left/right)
//    Pitch → tilt along Y-axis (front/back)
// ══════════════════════════════════════════
void calibrateMPU() {
  Serial.println("Calibrating... Keep drone flat and still!");
  Serial.println("Throttle input ignored during calibration.");
  uint8_t rawData[14];
  float rollSum  = 0;
  float pitchSum = 0;

  for (int i = 0; i < 2000; i++) {
    readBytes(MPU9250_ADDR, ACCEL_XOUT_H, 14, rawData);

    float ax = (float)toInt16(rawData[0], rawData[1]);
    float ay = (float)toInt16(rawData[2], rawData[3]);
    float az = (float)toInt16(rawData[4], rawData[5]);
    float gx = 0, gy = 0, gz = 0;
    applyAxisMapping(ax, ay, az, gx, gy, gz);

    // Module is RIGHT-SIDE UP, Y-axis faces FRONT of drone:
    //   Roll  = rotation around Y-axis → uses ax and az
    //   Pitch = rotation around X-axis → uses ay and az
    rollSum  += atan2((float)ax, (float)az) * 180.0 / PI;
    pitchSum += atan2(-(float)ay,
                  sqrt((float)ax * ax + (float)az * az))
                * 180.0 / PI;
    delay(2);
  }

  rollOffset  = rollSum  / 2000.0;
  pitchOffset = pitchSum / 2000.0;

  Serial.print("Roll offset:  "); Serial.println(rollOffset);
  Serial.print("Pitch offset: "); Serial.println(pitchOffset);
  Serial.println("Calibration complete.");
  calibrationComplete = true;
}

// ══════════════════════════════════════════
//  ESC Arming Sequence
//  Sends 1000µs for 3 seconds — required
//  by BLHeli / SimonK ESCs before they
//  accept any throttle signal.
// ══════════════════════════════════════════
void armESCs() {
  Serial.println("Arming ESCs — do not move drone...");
  motorFL.writeMicroseconds(1000);
  motorFR.writeMicroseconds(1000);
  motorBL.writeMicroseconds(1000);
  motorBR.writeMicroseconds(1000);
  delay(3000);
  Serial.println("ESCs armed.");
}

// ══════════════════════════════════════════
//  Safety Stop
// ══════════════════════════════════════════
void stopMotors() {
  motorFL.writeMicroseconds(1000);
  motorFR.writeMicroseconds(1000);
  motorBL.writeMicroseconds(1000);
  motorBR.writeMicroseconds(1000);
}

// ══════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  pinMode(RC_THROTTLE_PIN, INPUT);

  // I2C Fast Mode — 400kHz | SDA = GPIO21, SCL = GPIO22 (unchanged)
  Wire.begin(21, 22);
  Wire.setClock(400000);

  delay(200); // Let MPU9250 power up

  // Verify chip identity
  // WHO_AM_I: 0x71 = MPU-9250, 0x70 = MPU-9255
  uint8_t whoAmI = readByte(MPU9250_ADDR, WHO_AM_I);
  Serial.print("WHO_AM_I: 0x"); Serial.println(whoAmI, HEX);

  if (whoAmI == 0x71 || whoAmI == 0x70) {
    Serial.println("MPU-9250 detected!");
  } else {
    Serial.println("ERROR: MPU-9250 not found!");
    Serial.println("Check: VCC=3.3V, SDA=GPIO21, SCL=GPIO22");
    Serial.println("Check: AD0 pin — change MPU9250_ADDR to 0x69 if AD0 is HIGH");
    while (1) { delay(500); }
  }

  initMPU();

  // Hard safety gate: do not attach/arm motors until MPU calibration is complete.
  calibrateMPU();

  // Attach motors only after calibration
  motorFL.attach(MOTOR_FL_PIN, 1000, 2000);
  motorFR.attach(MOTOR_FR_PIN, 1000, 2000);
  motorBL.attach(MOTOR_BL_PIN, 1000, 2000);
  motorBR.attach(MOTOR_BR_PIN, 1000, 2000);

  if (!ORIENTATION_TEST_MODE) {
    Serial.println("Calibration done. Enabling ESC outputs.");
    armESCs();
  } else {
    Serial.println("ORIENTATION_TEST_MODE = true");
    Serial.println("Motors will stay at 1000us (safe).");
    Serial.println("Tilt RIGHT  -> R should increase (+)");
    Serial.println("Tilt LEFT   -> R should decrease (-)");
    Serial.println("Tilt NOSE DOWN (forward) -> P should decrease (-)");
    Serial.println("Tilt NOSE UP (backward)  -> P should increase (+)");
    Serial.println("If direction is wrong, update SWAP_XY / INVERT_X/Y/Z.");
  }

  prevTime = micros();
}

// ══════════════════════════════════════════
//  MAIN LOOP
// ══════════════════════════════════════════
void loop() {

  // ── 1. Read raw sensor data (14 bytes) ──
  //  Layout: Accel XYZ (6) + Temp (2) + Gyro XYZ (6)
  uint8_t rawData[14];
  readBytes(MPU9250_ADDR, ACCEL_XOUT_H, 14, rawData);

  float ax = (float)toInt16(rawData[0],  rawData[1]);
  float ay = (float)toInt16(rawData[2],  rawData[3]);
  float az = (float)toInt16(rawData[4],  rawData[5]);
  // rawData[6..7] = temperature (unused in flight loop)
  float gx = (float)toInt16(rawData[8],  rawData[9]);
  float gy = (float)toInt16(rawData[10], rawData[11]);
  float gz = (float)toInt16(rawData[12], rawData[13]);
  applyAxisMapping(ax, ay, az, gx, gy, gz);

  // ── 2. Convert gyro to °/s ──────────────
  //  Gyro ±500°/s → divisor = 65.5
  //  Module is RIGHT-SIDE UP, Y-axis = front:
  //    gyroRoll  → rotation sensed by gy (tilting left/right)
  //    gyroPitch → rotation sensed by gx (tilting front/back)
  //    gyroYaw   → gz (heading rotation, unaffected)
  float gyroRoll  = (gy / 65.5f);
  float gyroPitch = (gx / 65.5f);
  float gyroYaw   = (gz / 65.5f);

  // ── 3. Accel angle estimates ─────────────
  //  Right-side-up, Y-axis = front:
  //    Roll  = tilt around Y-axis → atan2(ax, az)
  //    Pitch = tilt around X-axis → atan2(-ay, sqrt(ax²+az²))
  //    (negative ay because forward tilt lowers ay reading)
  float accRoll  = atan2(ax, az)
                   * 180.0f / PI - rollOffset;

  float accPitch = atan2(-ay,
                      sqrt(ax * ax + az * az))
                    * 180.0f / PI - pitchOffset;

  // ── 4. Delta time ────────────────────────
  unsigned long now = micros();
  float dt = (now - prevTime) / 1000000.0f;
  prevTime = now;

  // Guard: skip bad dt (first loop / overflow)
  if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

  // ── 5. Complementary filter ──────────────
  //  98% gyro integration + 2% accelerometer correction
  roll  = 0.98f * (roll  + gyroRoll  * dt) + 0.02f * accRoll;
  pitch = 0.98f * (pitch + gyroPitch * dt) + 0.02f * accPitch;
  yaw  += gyroYaw * dt;   // gyro only — no magnetometer

  throttle = readThrottleUs();

  if (ORIENTATION_TEST_MODE) {
    stopMotors();
    static unsigned long lastPrintMs = 0;
    unsigned long nowMs = millis();
    if (nowMs - lastPrintMs >= 120) {
      lastPrintMs = nowMs;
      Serial.print("R:"); Serial.print(roll, 1);
      Serial.print(" P:"); Serial.print(pitch, 1);
      Serial.print(" Gx:"); Serial.print(gyroPitch, 1);
      Serial.print(" Gy:"); Serial.print(gyroRoll, 1);
      Serial.print(" Thr:"); Serial.println(throttle);
    }
    return;
  }

  // ── 6. Disarm if throttle too low ────────
  if (throttle < ARMED_THRESHOLD) {
    stopMotors();
    rollIntegral  = 0;
    pitchIntegral = 0;
    yawIntegral   = 0;
    Serial.println("DISARMED");
    return;
  }

  // ── 7. PID — Roll ────────────────────────
  rollError    = 0 - roll;
  rollIntegral = constrain(rollIntegral + rollError * dt,
                           -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float rollDeriv = (rollError - rollPrevError) / dt;
  float rollPID   = kp_roll  * rollError
                  + ki_roll  * rollIntegral
                  + kd_roll  * rollDeriv;
  rollPrevError = rollError;

  // ── 8. PID — Pitch ───────────────────────
  pitchError    = 0 - pitch;
  pitchIntegral = constrain(pitchIntegral + pitchError * dt,
                            -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float pitchDeriv = (pitchError - pitchPrevError) / dt;
  float pitchPID   = kp_pitch  * pitchError
                   + ki_pitch  * pitchIntegral
                   + kd_pitch  * pitchDeriv;
  pitchPrevError = pitchError;

  // ── 9. PID — Yaw ─────────────────────────
  //  Targets zero yaw rate (heading hold)
  yawError    = 0 - gyroYaw;
  yawIntegral = constrain(yawIntegral + yawError * dt,
                          -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float yawDeriv = (yawError - yawPrevError) / dt;
  float yawPID   = kp_yaw  * yawError
                 + ki_yaw  * yawIntegral
                 + kd_yaw  * yawDeriv;
  yawPrevError = yawError;

  if (THRUST_ONLY_MODE) {
    rollPID = 0.0f;
    pitchPID = 0.0f;
    yawPID = 0.0f;
  }

  // ── 10. Motor Mixing — X configuration ───
  //
  //   FL = +pitchPID +rollPID -yawPID
  //   FR = +pitchPID -rollPID +yawPID
  //   BL = -pitchPID +rollPID +yawPID
  //   BR = -pitchPID -rollPID -yawPID
  //
  int mFL = constrain((int)(throttle + pitchPID + rollPID - yawPID),
                      MIN_THROTTLE, MAX_THROTTLE);
  int mFR = constrain((int)(throttle + pitchPID - rollPID + yawPID),
                      MIN_THROTTLE, MAX_THROTTLE);
  int mBL = constrain((int)(throttle - pitchPID + rollPID + yawPID),
                      MIN_THROTTLE, MAX_THROTTLE);
  int mBR = constrain((int)(throttle - pitchPID - rollPID - yawPID),
                      MIN_THROTTLE, MAX_THROTTLE);

  motorFL.writeMicroseconds(mFL);
  motorFR.writeMicroseconds(mFR);
  motorBL.writeMicroseconds(mBL);
  motorBR.writeMicroseconds(mBR);

  // ── 11. Serial debug ─────────────────────
  Serial.print("R:");  Serial.print(roll,  1);
  Serial.print(" P:"); Serial.print(pitch, 1);
  Serial.print(" Y:"); Serial.print(yaw,   1);
  Serial.print(" dt:"); Serial.print(dt * 1000, 2);
  Serial.print("ms");
  Serial.print(" FL:"); Serial.print(mFL);
  Serial.print(" FR:"); Serial.print(mFR);
  Serial.print(" BL:"); Serial.print(mBL);
  Serial.print(" BR:"); Serial.println(mBR);
}
