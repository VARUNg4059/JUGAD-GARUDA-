// ============================================================
//  DRONE FLIGHT CONTROLLER — Arduino Uno + MPU-6050
//  Board   : Arduino Uno (ATmega328P)
//  IMU     : MPU-6050  (I2C addr 0x68, AD0 floating = GND default)
//  RC      : FlySky FS-CT6B → FS-R6B receiver (6-ch, 2.4 GHz)
//  ESCs    : Standard PWM 1000–2000 µs
//
//  MOTOR LAYOUT — X Configuration:
//
//         FRONT
//    FL(M1) ↖   ↗ FR(M2)
//     CW         CCW
//            ✚
//     CCW        CW
//    BL(M4) ↙   ↘ BR(M3)
//         BACK
//
//  PIN ASSIGNMENTS:
//    MPU-6050  : SDA → A4,  SCL → A5
//    RC CH1    : Pin 8   (Roll    — right stick L/R)
//    RC CH2    : Pin 9   (Pitch   — right stick U/D)
//    RC CH3    : Pin 10  (Throttle— left  stick U/D)
//    RC CH4    : Pin 11  (Yaw     — left  stick L/R)
//    STATUS LED: Pin 13  (HIGH when armed/running)
//    M1 FL ESC : Pin 4
//    M2 FR ESC : Pin 5
//    M3 BR ESC : Pin 6
//    M4 BL ESC : Pin 7
//
//  MPU MOUNTING (viewed from above):
//    Y-axis arrow → pointing toward FRONT of drone
//    X-axis arrow → pointing toward FR (right side)
//    Module face  → UP
//
//  AXIS CONVENTION (right-hand rule, standard):
//    Roll  = tilt left/right  → rotation around Y-axis (X accel dominant)
//    Pitch = tilt front/back  → rotation around X-axis (Y accel dominant)
//    Yaw   = heading rotation → rotation around Z-axis (gz)
//
//  OPERATING MODES (set at compile time):
//    THRUST_ONLY_MODE    true  → all motors same throttle, no PID
//    THRUST_ONLY_MODE    false → full PID stabilisation
//    ORIENTATION_TEST_MODE true  → motors stay at 1000µs, prints IMU data
//    ORIENTATION_TEST_MODE false → normal flight
// ============================================================

#include <Wire.h>
#include <Servo.h>   // Standard Arduino Servo (not ESP32Servo)

// ──────────────────────────────────────────────────────────
//  COMPILE-TIME MODE SWITCHES
//  Change these before uploading to switch behaviour.
// ──────────────────────────────────────────────────────────
#define THRUST_ONLY_MODE      false   // true = raw throttle, no PID
#define ORIENTATION_TEST_MODE true   // true = IMU print only, no motors

// ──────────────────────────────────────────────────────────
//  MPU-6050 I2C ADDRESS & REGISTER MAP
//  AD0 floating (treated as GND) → 0x68
// ──────────────────────────────────────────────────────────
#define MPU6050_ADDR    0x68
#define WHO_AM_I        0x75
#define PWR_MGMT_1      0x6B
#define SMPLRT_DIV      0x19
#define CONFIG_REG      0x1A   // DLPF config  (renamed to avoid collision)
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_XOUT_H    0x3B   // first of 14 consecutive registers

// ──────────────────────────────────────────────────────────
//  RC INPUT PINS  (hardware interrupt-capable: 2,3; others via PCINT)
//  We use Pin Change Interrupts (PCINT) for all 4 channels so
//  pulseIn() blocking is completely avoided.
// ──────────────────────────────────────────────────────────
#define RC_CH1_PIN  8    // Roll     (PCINT0)
#define RC_CH2_PIN  9    // Pitch    (PCINT1)
#define RC_CH3_PIN  10   // Throttle (PCINT2)
#define RC_CH4_PIN  11   // Yaw      (PCINT3)
#define STATUS_LED  13   // HIGH = armed / running

// ──────────────────────────────────────────────────────────
//  MOTOR / ESC PINS
//  M1=FL(CW)  M2=FR(CCW)  M3=BR(CW)  M4=BL(CCW)
// ──────────────────────────────────────────────────────────
#define MOTOR_FL_PIN  4   // M1 — Front Left  — CW
#define MOTOR_FR_PIN  5   // M2 — Front Right — CCW
#define MOTOR_BR_PIN  6   // M3 — Back Right  — CW
#define MOTOR_BL_PIN  7   // M4 — Back Left   — CCW

Servo motorFL, motorFR, motorBR, motorBL;

// ──────────────────────────────────────────────────────────
//  SAFETY CONSTANTS
// ──────────────────────────────────────────────────────────
#define MIN_THROTTLE      1000
#define MAX_THROTTLE      2000
#define ARMED_THRESHOLD   1050   // below this → motors stop, integrals reset
#define INTEGRAL_LIMIT     200   // anti-windup clamp (µs equivalent)
#define RC_TIMEOUT_US   50000UL  // 50 ms — if no pulse seen, treat as 1000

// ──────────────────────────────────────────────────────────
//  PID CONSTANTS
//  ⚠ ALWAYS start with props OFF.
//  kp_roll/pitch start at 1.5 — tune up slowly.
//  ki = 0 until kp/kd are tuned.
// ──────────────────────────────────────────────────────────
float kp_roll  = 1.5f,  ki_roll  = 0.0f,  kd_roll  = 0.04f;
float kp_pitch = 1.5f,  ki_pitch = 0.0f,  kd_pitch = 0.04f;
float kp_yaw   = 2.0f,  ki_yaw   = 0.0f,  kd_yaw   = 0.0f;

// ──────────────────────────────────────────────────────────
//  PID STATE
// ──────────────────────────────────────────────────────────
float rollError,  rollPrevError  = 0.0f, rollIntegral  = 0.0f;
float pitchError, pitchPrevError = 0.0f, pitchIntegral = 0.0f;
float yawError,   yawPrevError   = 0.0f, yawIntegral   = 0.0f;

// ──────────────────────────────────────────────────────────
//  ATTITUDE & CALIBRATION STATE
// ──────────────────────────────────────────────────────────
float roll  = 0.0f, pitch = 0.0f, yaw = 0.0f;
float rollOffset = 0.0f, pitchOffset = 0.0f;
bool  calibrationComplete = false;

// ──────────────────────────────────────────────────────────
//  TIMING
// ──────────────────────────────────────────────────────────
unsigned long prevTimeMicros = 0;

// ──────────────────────────────────────────────────────────
//  RC CHANNEL VALUES — written by ISR, read by main loop
//  volatile because modified in interrupt context.
// ──────────────────────────────────────────────────────────
volatile uint16_t rcRaw[4]    = {1000, 1000, 1000, 1000};
// [0]=CH1 Roll  [1]=CH2 Pitch  [2]=CH3 Throttle  [3]=CH4 Yaw
volatile unsigned long rcRiseTime[4] = {0, 0, 0, 0};
volatile uint8_t  prevPCINT0 = 0;   // previous state of PINB

// ──────────────────────────────────────────────────────────
//  PIN CHANGE INTERRUPT — PORTB (handles pins 8–13)
//  CH1=pin8=PB0  CH2=pin9=PB1  CH3=pin10=PB2  CH4=pin11=PB3
// ──────────────────────────────────────────────────────────
ISR(PCINT0_vect) {
  unsigned long now = micros();
  uint8_t curr = PINB;
  uint8_t changed = curr ^ prevPCINT0;

  // CH1 — PB0 (pin 8)
  if (changed & 0x01) {
    if (curr & 0x01) { rcRiseTime[0] = now; }          // rising edge
    else {
      uint16_t w = (uint16_t)(now - rcRiseTime[0]);
      if (w >= 900 && w <= 2100) rcRaw[0] = w;         // falling edge → valid
    }
  }
  // CH2 — PB1 (pin 9)
  if (changed & 0x02) {
    if (curr & 0x02) { rcRiseTime[1] = now; }
    else {
      uint16_t w = (uint16_t)(now - rcRiseTime[1]);
      if (w >= 900 && w <= 2100) rcRaw[1] = w;
    }
  }
  // CH3 — PB2 (pin 10)
  if (changed & 0x04) {
    if (curr & 0x04) { rcRiseTime[2] = now; }
    else {
      uint16_t w = (uint16_t)(now - rcRiseTime[2]);
      if (w >= 900 && w <= 2100) rcRaw[2] = w;
    }
  }
  // CH4 — PB3 (pin 11)
  if (changed & 0x08) {
    if (curr & 0x08) { rcRiseTime[3] = now; }
    else {
      uint16_t w = (uint16_t)(now - rcRiseTime[3]);
      if (w >= 900 && w <= 2100) rcRaw[3] = w;
    }
  }

  prevPCINT0 = curr;
}

// ──────────────────────────────────────────────────────────
//  SAFE RC READ — disables interrupts briefly for atomic copy
// ──────────────────────────────────────────────────────────
void readRC(uint16_t &ch1, uint16_t &ch2,
            uint16_t &ch3, uint16_t &ch4) {
  noInterrupts();
  ch1 = rcRaw[0];
  ch2 = rcRaw[1];
  ch3 = rcRaw[2];
  ch4 = rcRaw[3];
  interrupts();
  ch1 = constrain(ch1, MIN_THROTTLE, MAX_THROTTLE);
  ch2 = constrain(ch2, MIN_THROTTLE, MAX_THROTTLE);
  ch3 = constrain(ch3, MIN_THROTTLE, MAX_THROTTLE);
  ch4 = constrain(ch4, MIN_THROTTLE, MAX_THROTTLE);
}

// ══════════════════════════════════════════════════════════
//  LOW-LEVEL I2C HELPERS
// ══════════════════════════════════════════════════════════
void writeByte(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  uint8_t err = Wire.endTransmission();
  if (err) {
    Serial.print(F("[I2C] Write error reg=0x"));
    Serial.print(reg, HEX);
    Serial.print(F(" err="));
    Serial.println(err);
  }
}

uint8_t readByte(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  Wire.requestFrom(addr, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

void readBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    Serial.println(F("[I2C] readBytes transmission failed"));
    return;
  }
  Wire.requestFrom(addr, count);
  for (uint8_t i = 0; i < count; i++) {
    dest[i] = Wire.available() ? Wire.read() : 0;
  }
}

int16_t toInt16(uint8_t high, uint8_t low) {
  return (int16_t)((uint16_t)(high << 8) | low);
}

// ══════════════════════════════════════════════════════════
//  MPU-6050 INITIALISATION
//  Settings chosen for drone use:
//    Gyro  : ±500 °/s  → 65.5 LSB/(°/s)
//    Accel : ±2 g      → 16384 LSB/g
//    DLPF  : ~42 Hz    → attenuates motor vibration
//    Rate  : 1 kHz     (SMPLRT_DIV = 0)
// ══════════════════════════════════════════════════════════
void initMPU() {
  Serial.println(F("[MPU] Initialising MPU-6050..."));

  // 1. Wake from sleep; use internal 8 MHz oscillator
  writeByte(MPU6050_ADDR, PWR_MGMT_1, 0x00);
  delay(100);

  // 2. Select PLL w/ X-gyro reference (better stability)
  writeByte(MPU6050_ADDR, PWR_MGMT_1, 0x01);
  delay(50);

  // 3. DLPF = 3 → 42 Hz LPF on accel & gyro
  writeByte(MPU6050_ADDR, CONFIG_REG, 0x03);

  // 4. Sample rate divider = 0 → 1 kHz
  writeByte(MPU6050_ADDR, SMPLRT_DIV, 0x00);

  // 5. Gyro full scale ±500 °/s (bits [4:3] = 01)
  writeByte(MPU6050_ADDR, GYRO_CONFIG, 0x08);

  // 6. Accel full scale ±2 g (bits [4:3] = 00)
  writeByte(MPU6050_ADDR, ACCEL_CONFIG, 0x00);

  delay(50);
  Serial.println(F("[MPU] Initialisation complete."));
  Serial.println(F("[MPU]   Gyro  : +/-500 deg/s  (65.5 LSB/deg/s)"));
  Serial.println(F("[MPU]   Accel : +/-2g          (16384 LSB/g)"));
  Serial.println(F("[MPU]   DLPF  : 42 Hz"));
  Serial.println(F("[MPU]   Rate  : 1 kHz"));
}

// ══════════════════════════════════════════════════════════
//  MPU-6050 CALIBRATION
//
//  Samples 2000 readings (~4 s) with the drone flat & still.
//  The averaged roll and pitch angles become the ZERO reference.
//  This means wherever the drone is placed during calibration,
//  THAT surface is treated as perfectly level.
//
//  Why 2000 samples?
//    - Averages out gyro/accel noise
//    - Motor vibration is absent (motors not yet spinning)
//
//  Axis convention with this MPU mounting:
//    Y arrow → FRONT   X arrow → RIGHT (FR side)
//
//    Roll  angle = atan2(ax, az)          — tilt left/right
//    Pitch angle = atan2(-ay, sqrt(ax²+az²)) — tilt front/back
//      (negative ay: when nose dips, ay increases, so we negate)
// ══════════════════════════════════════════════════════════
void calibrateMPU() {
  Serial.println(F(""));
  Serial.println(F("========================================"));
  Serial.println(F("[CAL] MPU-6050 CALIBRATION STARTING"));
  Serial.println(F("[CAL] Place drone on flat surface & DO NOT move it!"));
  Serial.println(F("[CAL] Sampling 2000 readings... (~4 seconds)"));
  Serial.println(F("========================================"));

  uint8_t rawData[14];
  float rollSum  = 0.0f;
  float pitchSum = 0.0f;

  for (int i = 0; i < 2000; i++) {
    readBytes(MPU6050_ADDR, ACCEL_XOUT_H, 14, rawData);

    float ax = (float)toInt16(rawData[0], rawData[1]);
    float ay = (float)toInt16(rawData[2], rawData[3]);
    float az = (float)toInt16(rawData[4], rawData[5]);

    // MPU mounting: Y→front, X→right, face UP — standard orientation
    // No axis remapping needed; X and Y are correct for roll/pitch.
    rollSum  += atan2(ax, az) * 180.0f / PI;
    pitchSum += atan2(-ay, sqrt(ax * ax + az * az)) * 180.0f / PI;

    // Progress every 200 samples
    if (i % 200 == 0) {
      Serial.print(F("[CAL] Progress: "));
      Serial.print(i / 20);
      Serial.println(F("%"));
    }
    delay(2);
  }

  rollOffset  = rollSum  / 2000.0f;
  pitchOffset = pitchSum / 2000.0f;
  calibrationComplete = true;

  Serial.println(F(""));
  Serial.println(F("[CAL] === CALIBRATION COMPLETE ==="));
  Serial.print(F("[CAL] Roll  offset : "));
  Serial.print(rollOffset, 4);
  Serial.println(F(" deg"));
  Serial.print(F("[CAL] Pitch offset : "));
  Serial.print(pitchOffset, 4);
  Serial.println(F(" deg"));
  Serial.println(F("[CAL] Current surface = LEVEL REFERENCE (zero)"));
  Serial.println(F("========================================"));
  Serial.println(F(""));
}

// ══════════════════════════════════════════════════════════
//  ESC ARMING SEQUENCE
//  Sends 1000 µs for 3 seconds.
//  Required by BLHeli / SimonK ESCs before accepting throttle.
//  Motors are attached (Servo.attach) BEFORE this is called
//  so the signal line goes LOW immediately — no spurious pulses.
// ══════════════════════════════════════════════════════════
void armESCs() {
  Serial.println(F("[ARM] Sending 1000us to all ESCs for 3 seconds..."));
  Serial.println(F("[ARM] Do NOT move the drone!"));
  motorFL.writeMicroseconds(1000);
  motorFR.writeMicroseconds(1000);
  motorBR.writeMicroseconds(1000);
  motorBL.writeMicroseconds(1000);
  delay(3000);
  Serial.println(F("[ARM] ESCs ARMED — ready to accept throttle."));
}

// ══════════════════════════════════════════════════════════
//  SAFETY STOP — all motors to minimum
// ══════════════════════════════════════════════════════════
void stopMotors() {
  motorFL.writeMicroseconds(1000);
  motorFR.writeMicroseconds(1000);
  motorBR.writeMicroseconds(1000);
  motorBL.writeMicroseconds(1000);
}

// ══════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }   // wait for Serial on Uno (instant, just safe)

  Serial.println(F(""));
  Serial.println(F("============================================================"));
  Serial.println(F("  DRONE FLIGHT CONTROLLER — Arduino Uno + MPU-6050"));
  Serial.println(F("============================================================"));
  Serial.print(F("  THRUST_ONLY_MODE    : "));
  Serial.println(THRUST_ONLY_MODE    ? F("TRUE  (raw throttle, no PID)") : F("FALSE (full PID stabilisation)"));
  Serial.print(F("  ORIENTATION_TEST    : "));
  Serial.println(ORIENTATION_TEST_MODE ? F("TRUE  (IMU print only, motors off)") : F("FALSE (normal flight)"));
  Serial.println(F("============================================================"));
  Serial.println(F(""));

  // Status LED — will go HIGH after arming
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  // ── RC Pin Change Interrupts ─────────────────────────────
  // Enable PCINT for PB0..PB3 (pins 8,9,10,11)
  PCICR  |= (1 << PCIE0);          // enable PORTB PCINTs
  PCMSK0 |= (1 << PCINT0)          // pin 8  CH1 Roll
           |(1 << PCINT1)          // pin 9  CH2 Pitch
           |(1 << PCINT2)          // pin 10 CH3 Throttle
           |(1 << PCINT3);         // pin 11 CH4 Yaw
  prevPCINT0 = PINB;               // capture initial state
  Serial.println(F("[RC]  Pin Change Interrupts enabled on pins 8,9,10,11."));
  Serial.println(F("[RC]  CH1=Roll CH2=Pitch CH3=Throttle CH4=Yaw"));

  // ── I2C — 400 kHz Fast Mode ──────────────────────────────
  // Arduino Uno: SDA=A4, SCL=A5 (hardware fixed)
  Wire.begin();
  Wire.setClock(400000UL);
  Serial.println(F("[I2C] Started at 400 kHz. SDA=A4, SCL=A5."));

  delay(200);   // MPU-6050 power-up time

  // ── Verify MPU-6050 WHO_AM_I ─────────────────────────────
  // MPU-6050 WHO_AM_I = 0x68 (bits [6:1] of register 0x75)
  uint8_t whoAmI = readByte(MPU6050_ADDR, WHO_AM_I);
  Serial.print(F("[MPU] WHO_AM_I register = 0x"));
  Serial.println(whoAmI, HEX);

  if (whoAmI == 0x68) {
    Serial.println(F("[MPU] MPU-6050 detected OK!"));
  } else {
    Serial.println(F("[MPU] *** ERROR: MPU-6050 NOT FOUND! ***"));
    Serial.println(F("[MPU] Expected WHO_AM_I = 0x68"));
    Serial.println(F("[MPU] Check wiring: SDA->A4, SCL->A5, VCC->5V, GND->GND"));
    Serial.println(F("[MPU] AD0 should be floating or connected to GND."));
    Serial.println(F("[MPU] If AD0 is pulled HIGH, change MPU6050_ADDR to 0x69."));
    Serial.println(F("[MPU] HALTING — fix the connection and reset."));
    digitalWrite(STATUS_LED, LOW);
    while (1) {
      // Blink LED fast to indicate hardware fault
      digitalWrite(STATUS_LED, HIGH); delay(100);
      digitalWrite(STATUS_LED, LOW);  delay(100);
    }
  }

  // ── Initialise MPU-6050 ──────────────────────────────────
  initMPU();

  // ── Calibrate (MUST happen before motors attach) ─────────
  // Safety gate: ESCs not attached yet so no accidental spin
  if (!ORIENTATION_TEST_MODE) {
    calibrateMPU();
  } else {
    // In orientation test mode we still need calibration for
    // offset-corrected attitude display to be meaningful.
    Serial.println(F("[TEST] ORIENTATION_TEST_MODE: calibrating for display..."));
    calibrateMPU();
    Serial.println(F("[TEST] Motors will remain at 1000us throughout."));
    Serial.println(F("[TEST] Tilt RIGHT        -> Roll  should INCREASE (+)"));
    Serial.println(F("[TEST] Tilt LEFT         -> Roll  should DECREASE (-)"));
    Serial.println(F("[TEST] Tilt NOSE DOWN    -> Pitch should DECREASE (-)"));
    Serial.println(F("[TEST] Tilt NOSE UP      -> Pitch should INCREASE (+)"));
    Serial.println(F("[TEST] If reversed, swap INVERT_X / INVERT_Y defines."));
  }

  // ── Attach motors AFTER calibration ─────────────────────
  // Attach immediately writes 1000µs — no runaway on plug-in.
  motorFL.attach(MOTOR_FL_PIN, 1000, 2000);
  motorFR.attach(MOTOR_FR_PIN, 1000, 2000);
  motorBR.attach(MOTOR_BR_PIN, 1000, 2000);
  motorBL.attach(MOTOR_BL_PIN, 1000, 2000);
  stopMotors();   // explicit 1000µs to all
  Serial.println(F("[ESC] Motors attached. All at 1000us."));

  if (!ORIENTATION_TEST_MODE) {
    armESCs();
    digitalWrite(STATUS_LED, HIGH);   // LED ON = armed and running
    Serial.println(F("[SYS] System ARMED. Status LED ON."));
  } else {
    Serial.println(F("[TEST] Orientation test mode — motors will stay off."));
    digitalWrite(STATUS_LED, HIGH);   // LED ON = system running
  }

  prevTimeMicros = micros();

  Serial.println(F(""));
  Serial.println(F("[SYS] Entering main loop."));
  Serial.println(F("[SYS] Serial columns:"));
  if (ORIENTATION_TEST_MODE) {
    Serial.println(F("[SYS]   [TEST] R:<roll> P:<pitch> Yaw:<yaw> Gx:<gyroPitch> Gy:<gyroRoll> Gz:<gyroYaw> CH1:<roll_in> CH2:<pitch_in> CH3:<thr> CH4:<yaw_in>"));
  } else if (THRUST_ONLY_MODE) {
    Serial.println(F("[SYS]   [THST] Thr:<throttle> FL:<us> FR:<us> BR:<us> BL:<us> R:<roll> P:<pitch>"));
  } else {
    Serial.println(F("[SYS]   [STAB] R:<roll> P:<pitch> Y:<yaw> dt:<ms> rPID:<val> pPID:<val> yPID:<val> FL:<us> FR:<us> BR:<us> BL:<us> Thr:<throttle>"));
  }
  Serial.println(F(""));
}

// ══════════════════════════════════════════════════════════
//  MAIN LOOP
// ══════════════════════════════════════════════════════════
void loop() {

  // ── 1. Read raw IMU data (14 bytes burst) ────────────────
  //  Layout:  [0-5] Accel XYZ   [6-7] Temp   [8-13] Gyro XYZ
  uint8_t rawData[14];
  readBytes(MPU6050_ADDR, ACCEL_XOUT_H, 14, rawData);

  float ax = (float)toInt16(rawData[0],  rawData[1]);   // raw accel X
  float ay = (float)toInt16(rawData[2],  rawData[3]);   // raw accel Y
  float az = (float)toInt16(rawData[4],  rawData[5]);   // raw accel Z
  // rawData[6..7] = temperature — not used
  float gx = (float)toInt16(rawData[8],  rawData[9]);   // raw gyro X
  float gy = (float)toInt16(rawData[10], rawData[11]);  // raw gyro Y
  float gz = (float)toInt16(rawData[12], rawData[13]);  // raw gyro Z

  // ── 2. Axis mapping for this MPU mounting ────────────────
  //  Physical mounting: Y→front, X→right, face UP
  //  Standard orientation: no swap, no invert needed.
  //
  //  Gyro rates in °/s  (±500°/s → divide by 65.5)
  //  gyroRoll  : rate of roll  = gy  (rotation around Y-axis)
  //  gyroPitch : rate of pitch = gx  (rotation around X-axis)
  //  gyroYaw   : rate of yaw   = gz  (rotation around Z-axis)
  float gyroRoll  =  gy / 65.5f;
  float gyroPitch =  gx / 65.5f;
  float gyroYaw   =  gz / 65.5f;

  // ── 3. Accelerometer angle estimates ─────────────────────
  //  atan2 outputs in radians → convert to degrees → subtract offset
  //  Roll  = atan2(ax, az)            [left/right tilt]
  //  Pitch = atan2(-ay, sqrt(ax²+az²)) [front/back tilt]
  //    Negate ay: when nose dips forward, ay increases, we want pitch < 0
  float accRoll  = atan2(ax, az) * 180.0f / PI - rollOffset;
  float accPitch = atan2(-ay, sqrt(ax * ax + az * az)) * 180.0f / PI - pitchOffset;

  // ── 4. Delta time ─────────────────────────────────────────
  unsigned long nowMicros = micros();
  float dt = (float)(nowMicros - prevTimeMicros) / 1000000.0f;
  prevTimeMicros = nowMicros;
  // Guard: skip first loop overflow / absurd dt
  if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

  // ── 5. Complementary filter ───────────────────────────────
  //  98% gyro integration (fast, low-noise) +
  //   2% accelerometer correction (drift correction, slow)
  roll  = 0.98f * (roll  + gyroRoll  * dt) + 0.02f * accRoll;
  pitch = 0.98f * (pitch + gyroPitch * dt) + 0.02f * accPitch;
  yaw  += gyroYaw * dt;   // gyro-only (no magnetometer on MPU-6050)

  // ── 6. Read RC channels ───────────────────────────────────
  uint16_t rcCH1, rcCH2, rcCH3, rcCH4;
  readRC(rcCH1, rcCH2, rcCH3, rcCH4);
  // CH1 = Roll input, CH2 = Pitch input, CH3 = Throttle, CH4 = Yaw
  int throttle = (int)rcCH3;

  // ── 7. ORIENTATION TEST MODE ──────────────────────────────
  if (ORIENTATION_TEST_MODE) {
    stopMotors();
    static unsigned long lastPrintMs = 0;
    unsigned long nowMs = millis();
    if (nowMs - lastPrintMs >= 100) {   // 10 Hz print rate
      lastPrintMs = nowMs;
      Serial.print(F("[TEST] R:"));    Serial.print(roll, 2);
      Serial.print(F(" P:"));          Serial.print(pitch, 2);
      Serial.print(F(" Yaw:"));        Serial.print(yaw, 2);
      Serial.print(F(" Gx:"));         Serial.print(gyroPitch, 2);
      Serial.print(F(" Gy:"));         Serial.print(gyroRoll, 2);
      Serial.print(F(" Gz:"));         Serial.print(gyroYaw, 2);
      Serial.print(F(" CH1:"));        Serial.print(rcCH1);
      Serial.print(F(" CH2:"));        Serial.print(rcCH2);
      Serial.print(F(" CH3(Thr):"));   Serial.print(rcCH3);
      Serial.print(F(" CH4:"));        Serial.println(rcCH4);
    }
    return;
  }

  // ── 8. DISARM if throttle too low ────────────────────────
  //  ARMED_THRESHOLD = 1050 µs.
  //  If throttle stick is at bottom (1000 µs), everything stops.
  //  This is the primary safety gate in flight.
  if (throttle < ARMED_THRESHOLD) {
    stopMotors();
    rollIntegral  = 0.0f;
    pitchIntegral = 0.0f;
    yawIntegral   = 0.0f;
    static unsigned long lastDisarmPrint = 0;
    if (millis() - lastDisarmPrint >= 500) {
      lastDisarmPrint = millis();
      Serial.print(F("[DISARMED] Throttle="));
      Serial.print(throttle);
      Serial.println(F("us (below 1050us) — motors stopped, integrals reset."));
    }
    return;
  }

  // ── 9. THRUST-ONLY MODE ───────────────────────────────────
  //  All motors receive same throttle — PID bypassed.
  //  Used for safe bench testing of motor/ESC response.
  if (THRUST_ONLY_MODE) {
    int thr = constrain(throttle, MIN_THROTTLE, MAX_THROTTLE);
    motorFL.writeMicroseconds(thr);
    motorFR.writeMicroseconds(thr);
    motorBR.writeMicroseconds(thr);
    motorBL.writeMicroseconds(thr);

    static unsigned long lastPrintMs = 0;
    if (millis() - lastPrintMs >= 100) {
      lastPrintMs = millis();
      Serial.print(F("[THST] Thr:"));  Serial.print(thr);
      Serial.print(F(" FL:"));         Serial.print(thr);
      Serial.print(F(" FR:"));         Serial.print(thr);
      Serial.print(F(" BR:"));         Serial.print(thr);
      Serial.print(F(" BL:"));         Serial.print(thr);
      Serial.print(F(" R:"));          Serial.print(roll, 1);
      Serial.print(F(" P:"));          Serial.println(pitch, 1);
    }
    return;
  }

  // ── 10. PID — ROLL ───────────────────────────────────────
  //  Target: roll = 0 (level)
  //  Error positive → drone tilted right → FL/BL need more power
  rollError    = 0.0f - roll;
  rollIntegral = constrain(rollIntegral + rollError * dt,
                           -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float rollDeriv = (rollError - rollPrevError) / dt;
  float rollPID   = kp_roll * rollError
                  + ki_roll * rollIntegral
                  + kd_roll * rollDeriv;
  rollPrevError = rollError;

  // ── 11. PID — PITCH ──────────────────────────────────────
  //  Target: pitch = 0 (level)
  //  Error positive → drone pitched back → FL/FR need more power
  pitchError    = 0.0f - pitch;
  pitchIntegral = constrain(pitchIntegral + pitchError * dt,
                            -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float pitchDeriv = (pitchError - pitchPrevError) / dt;
  float pitchPID   = kp_pitch * pitchError
                   + ki_pitch * pitchIntegral
                   + kd_pitch * pitchDeriv;
  pitchPrevError = pitchError;

  // ── 12. PID — YAW ────────────────────────────────────────
  //  Target: zero yaw rate (heading hold / no rotation)
  //  Uses gyroYaw directly (rate control, not angle control)
  yawError    = 0.0f - gyroYaw;
  yawIntegral = constrain(yawIntegral + yawError * dt,
                          -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float yawDeriv = (yawError - yawPrevError) / dt;
  float yawPID   = kp_yaw * yawError
                 + ki_yaw * yawIntegral
                 + kd_yaw * yawDeriv;
  yawPrevError = yawError;

  // ── 13. MOTOR MIXING — X Configuration ───────────────────
  //
  //  Motor spin directions:
  //    FL (M1) CW    FR (M2) CCW
  //    BL (M4) CCW   BR (M3) CW
  //
  //  Mixing table (standard X-config):
  //    FL = throttle + pitchPID + rollPID - yawPID
  //    FR = throttle + pitchPID - rollPID + yawPID
  //    BR = throttle - pitchPID - rollPID - yawPID
  //    BL = throttle - pitchPID + rollPID + yawPID
  //
  //  Logic check:
  //    Roll right (drone tilts R): rollError < 0 → rollPID < 0
  //      FL gets less (+roll), FR gets more (-roll)
  //      BL gets less (+roll), BR gets more (-roll)
  //      → left side spins up, drone levels → correct ✓
  //
  //    Pitch nose down: pitchError > 0 → pitchPID > 0
  //      FL/FR get more, BL/BR get less
  //      → front motors push nose up → correct ✓
  //
  //    Yaw CW (from above): gyroYaw > 0 → yawError < 0 → yawPID < 0
  //      FL(-yaw) gets more, FR(+yaw) gets less
  //      BR(-yaw) gets more, BL(+yaw) gets less
  //      → CW motors (FL,BR) spin up to counter CW rotation → correct ✓

  int mFL = constrain((int)(throttle + pitchPID + rollPID - yawPID),
                      MIN_THROTTLE, MAX_THROTTLE);
  int mFR = constrain((int)(throttle + pitchPID - rollPID + yawPID),
                      MIN_THROTTLE, MAX_THROTTLE);
  int mBR = constrain((int)(throttle - pitchPID - rollPID - yawPID),
                      MIN_THROTTLE, MAX_THROTTLE);
  int mBL = constrain((int)(throttle - pitchPID + rollPID + yawPID),
                      MIN_THROTTLE, MAX_THROTTLE);

  motorFL.writeMicroseconds(mFL);
  motorFR.writeMicroseconds(mFR);
  motorBR.writeMicroseconds(mBR);
  motorBL.writeMicroseconds(mBL);

  // ── 14. SERIAL TELEMETRY ─────────────────────────────────
  //  Print at ~25 Hz to avoid flooding Serial and adding latency.
  static unsigned long lastPrintMs = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastPrintMs >= 40) {
    lastPrintMs = nowMs;
    Serial.print(F("[STAB] R:"));     Serial.print(roll, 2);
    Serial.print(F(" P:"));           Serial.print(pitch, 2);
    Serial.print(F(" Y:"));           Serial.print(yaw, 2);
    Serial.print(F(" dt:"));          Serial.print(dt * 1000.0f, 2);
    Serial.print(F("ms rPID:"));      Serial.print(rollPID, 2);
    Serial.print(F(" pPID:"));        Serial.print(pitchPID, 2);
    Serial.print(F(" yPID:"));        Serial.print(yawPID, 2);
    Serial.print(F(" FL:"));          Serial.print(mFL);
    Serial.print(F(" FR:"));          Serial.print(mFR);
    Serial.print(F(" BR:"));          Serial.print(mBR);
    Serial.print(F(" BL:"));          Serial.print(mBL);
    Serial.print(F(" Thr:"));         Serial.println(throttle);
  }
}

// ============================================================
//  END OF FILE
// ============================================================
