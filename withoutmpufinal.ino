/*
 * DRONE FLIGHT CONTROLLER
 * Features:
 * - 4-channel receiver input (Roll, Pitch, Throttle, Yaw)
 * - Direct BLDC motor control via ESCs
 * - PID pass-through from flight controller commands
 * NOTE: MPU6050 removed. Angle feedback = 0. 
 *       PID acts purely on receiver-commanded desired angles.
 */

#include <ESP32Servo.h>

// ============== PIN DEFINITIONS ==============
// Receiver Channels
#define CH1_PIN 34  // Roll
#define CH2_PIN 35  // Pitch
#define CH3_PIN 32  // Throttle
#define CH4_PIN 33  // Yaw

// Motor ESC Pins (PWM Output)
#define MOTOR_FL 25  // Front Left
#define MOTOR_FR 26  // Front Right
#define MOTOR_BL 27  // Back Left
#define MOTOR_BR 14  // Back Right

// ============== OBJECTS ==============
Servo motorFL, motorFR, motorBL, motorBR;

// ============== RECEIVER VARIABLES ==============
int ch1_value = 1500;  // Roll
int ch2_value = 1500;  // Pitch
int ch3_value = 1000;  // Throttle
int ch4_value = 1500;  // Yaw

// ============== PID VARIABLES ==============
// PID gains - TUNE THESE for your drone!
float kp_roll = 1.5, ki_roll = 0.05, kd_roll = 0.8;
float kp_pitch = 1.5, ki_pitch = 0.05, kd_pitch = 0.8;
float kp_yaw = 2.0, ki_yaw = 0.02, kd_yaw = 1.0;

// PID errors
float error_roll, prev_error_roll = 0, integral_roll = 0;
float error_pitch, prev_error_pitch = 0, integral_pitch = 0;
float error_yaw, prev_error_yaw = 0, integral_yaw = 0;

// PID outputs
float pid_roll, pid_pitch, pid_yaw;

// Desired angles/rates from receiver
float desired_roll = 0, desired_pitch = 0, desired_yaw_rate = 0;

// ============== MOTOR OUTPUTS ==============
int throttle = 1000;
int motor_fl, motor_fr, motor_bl, motor_br;

// ============== TIMING ==============
unsigned long prevTime = 0;
float dt = 0.01;  // 10ms loop time

// ============== SAFETY ==============
bool motorsArmed = false;
#define MIN_THROTTLE     1000
#define MAX_THROTTLE     2000
// THROTTLE_CUTOFF: motors send 1000µs (stop) when throttle is below this.
// Check serial monitor → "CH3 (Throttle)" at stick bottom, then set ~50 above that value.
// Example: if stick bottom reads 1040µs, set this to 1090.
#define THROTTLE_CUTOFF  1150

// ============== DEBUG ==============
bool verboseDebug = true;  // Set to false to reduce output
unsigned long lastDebugPrint = 0;

// ============================================
//               SETUP
// ============================================
void setup() {
  Serial.begin(115200);

  Serial.println("\n=== DRONE FLIGHT CONTROLLER ===");

  // Initialize receiver pins
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);
  pinMode(CH4_PIN, INPUT);

  // Initialize ESCs
  motorFL.attach(MOTOR_FL, 1000, 2000);
  motorFR.attach(MOTOR_FR, 1000, 2000);
  motorBL.attach(MOTOR_BL, 1000, 2000);
  motorBR.attach(MOTOR_BR, 1000, 2000);

  // Send minimum signal to ESCs for initialization
  Serial.println("\nInitializing ESCs...");
  Serial.println("Sending 1000µs to all motors...");
  motorFL.writeMicroseconds(1000);
  motorFR.writeMicroseconds(1000);
  motorBL.writeMicroseconds(1000);
  motorBR.writeMicroseconds(1000);

  Serial.println("Waiting 3 seconds for ESC beeps...");
  for (int i = 3; i > 0; i--) {
    Serial.print(i);
    Serial.println("...");
    delay(1000);
  }
  Serial.println("ESC initialization complete!");

  Serial.println("\n=== SYSTEM READY ===");
  Serial.println("ARMING INSTRUCTIONS:");
  Serial.println("1. Move throttle stick to MINIMUM (bottom)");
  Serial.println("2. Move throttle stick to MAXIMUM (top)");
  Serial.println("3. Move throttle stick back to MINIMUM");
  Serial.println("4. Motors will ARM and be ready to spin");
  Serial.println("====================\n");

  prevTime = millis();
  lastDebugPrint = millis();
}

// ============================================
//               MAIN LOOP
// ============================================
void loop() {
  // Calculate loop time
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // Read receiver inputs
  readReceiver();

  // Check arming sequence
  checkArming();

  // Calculate PID corrections
  calculatePID();

  // Mix and output to motors
  mixMotors();

  // Continuous debug output
  debugPrint();

  // Maintain loop time ~10ms
  delay(10);
}

// ============================================
//        READ RECEIVER
// ============================================
void readReceiver() {
  int raw_ch1 = pulseIn(CH1_PIN, HIGH, 30000);  // Roll
  int raw_ch2 = pulseIn(CH2_PIN, HIGH, 30000);  // Pitch
  int raw_ch3 = pulseIn(CH3_PIN, HIGH, 30000);  // Throttle
  int raw_ch4 = pulseIn(CH4_PIN, HIGH, 30000);  // Yaw

  // Debug raw values
  static unsigned long lastRxDebug = 0;
  if (millis() - lastRxDebug > 2000) {  // Every 2 seconds
    lastRxDebug = millis();
    Serial.println("\n--- RECEIVER RAW VALUES ---");
    Serial.print("CH1 (Roll):     "); Serial.print(raw_ch1);
    Serial.println(raw_ch1 == 0 ? " [NO SIGNAL]" : " [OK]");
    Serial.print("CH2 (Pitch):    "); Serial.print(raw_ch2);
    Serial.println(raw_ch2 == 0 ? " [NO SIGNAL]" : " [OK]");
    Serial.print("CH3 (Throttle): "); Serial.print(raw_ch3);
    Serial.println(raw_ch3 == 0 ? " [NO SIGNAL]" : " [OK]");
    Serial.print("CH4 (Yaw):      "); Serial.print(raw_ch4);
    Serial.println(raw_ch4 == 0 ? " [NO SIGNAL]" : " [OK]");
  }

  // Validate and constrain
  ch1_value = (raw_ch1 == 0) ? 1500 : constrain(raw_ch1, 1000, 2000);
  ch2_value = (raw_ch2 == 0) ? 1500 : constrain(raw_ch2, 1000, 2000);
  ch3_value = (raw_ch3 == 0) ? 1000 : constrain(raw_ch3, 1000, 2000);
  ch4_value = (raw_ch4 == 0) ? 1500 : constrain(raw_ch4, 1000, 2000);

  // Map receiver values to desired angles/rates
  desired_roll = map(ch1_value, 1000, 2000, -30, 30);       // ±30° max roll
  desired_pitch = map(ch2_value, 1000, 2000, -30, 30);      // ±30° max pitch
  throttle = ch3_value;                                       // Direct throttle
  desired_yaw_rate = map(ch4_value, 1000, 2000, -180, 180); // ±180°/s yaw rate
}

// ============================================
//        CALCULATE PID
// ============================================
void calculatePID() {
  if (!motorsArmed) {
    // Reset PID when disarmed
    integral_roll = 0;
    integral_pitch = 0;
    integral_yaw = 0;
    prev_error_roll = 0;
    prev_error_pitch = 0;
    prev_error_yaw = 0;
    return;
  }

  // ROLL PID (no IMU feedback — angle assumed 0)
  error_roll = desired_roll - 0.0;
  integral_roll += error_roll * dt;
  integral_roll = constrain(integral_roll, -50, 50);  // Anti-windup
  float derivative_roll = (error_roll - prev_error_roll) / dt;
  pid_roll = (kp_roll * error_roll) + (ki_roll * integral_roll) + (kd_roll * derivative_roll);
  prev_error_roll = error_roll;

  // PITCH PID (no IMU feedback — angle assumed 0)
  error_pitch = desired_pitch - 0.0;
  integral_pitch += error_pitch * dt;
  integral_pitch = constrain(integral_pitch, -50, 50);
  float derivative_pitch = (error_pitch - prev_error_pitch) / dt;
  pid_pitch = (kp_pitch * error_pitch) + (ki_pitch * integral_pitch) + (kd_pitch * derivative_pitch);
  prev_error_pitch = error_pitch;

  // YAW PID (rate control — yaw rate assumed 0)
  error_yaw = desired_yaw_rate - 0.0;
  integral_yaw += error_yaw * dt;
  integral_yaw = constrain(integral_yaw, -50, 50);
  float derivative_yaw = (error_yaw - prev_error_yaw) / dt;
  pid_yaw = (kp_yaw * error_yaw) + (ki_yaw * integral_yaw) + (kd_yaw * derivative_yaw);
  prev_error_yaw = error_yaw;
}

// ============================================
//        MOTOR MIXING
// ============================================
void mixMotors() {
  if (!motorsArmed || throttle < THROTTLE_CUTOFF) {
    // Motors off
    motor_fl = 1000;
    motor_fr = 1000;
    motor_bl = 1000;
    motor_br = 1000;
  } else {
    // Quadcopter X configuration motor mixing
    // FL: Throttle - Pitch + Roll - Yaw
    // FR: Throttle - Pitch - Roll + Yaw
    // BL: Throttle + Pitch + Roll + Yaw
    // BR: Throttle + Pitch - Roll - Yaw

    motor_fl = throttle - pid_pitch + pid_roll - pid_yaw;
    motor_fr = throttle - pid_pitch - pid_roll + pid_yaw;
    motor_bl = throttle + pid_pitch + pid_roll + pid_yaw;
    motor_br = throttle + pid_pitch - pid_roll - pid_yaw;

    // Constrain to valid PWM range
    motor_fl = constrain(motor_fl, MIN_THROTTLE, MAX_THROTTLE);
    motor_fr = constrain(motor_fr, MIN_THROTTLE, MAX_THROTTLE);
    motor_bl = constrain(motor_bl, MIN_THROTTLE, MAX_THROTTLE);
    motor_br = constrain(motor_br, MIN_THROTTLE, MAX_THROTTLE);
  }

  // Write to ESCs
  motorFL.writeMicroseconds(motor_fl);
  motorFR.writeMicroseconds(motor_fr);
  motorBL.writeMicroseconds(motor_bl);
  motorBR.writeMicroseconds(motor_br);
}

// ============================================
//        ARMING CHECK
// ============================================
// Arming sequence: throttle LOW → HIGH → LOW
// Each step must be held for at least STEP_HOLD_MS ms
// to avoid false triggers from signal noise.
// Entire sequence must complete within SEQUENCE_TIMEOUT_MS.
//
// Disarm: hold throttle below DISARM_THRESHOLD for DISARM_HOLD_MS.

#define ARM_LOW_THRESHOLD    THROTTLE_CUTOFF  // µs — "throttle low"
#define ARM_HIGH_THRESHOLD   1850   // µs — "throttle high"
#define DISARM_THRESHOLD     THROTTLE_CUTOFF  // µs — hold low to disarm (matches motor cutoff)
#define STEP_HOLD_MS          300   // ms each step must be stable
#define SEQUENCE_TIMEOUT_MS 12000   // ms to complete full sequence
#define DISARM_HOLD_MS       4000   // ms to hold low before disarm

void checkArming() {
  // ---- Arming state machine ----
  // States: 0=waiting for LOW, 1=got LOW, 2=got HIGH, 3=armed
  static uint8_t armState = 0;
  static unsigned long stepStartTime = 0;
  static unsigned long seqStartTime  = 0;

  if (!motorsArmed) {
    unsigned long now = millis();

    switch (armState) {

      case 0:  // Waiting for throttle LOW
        if (ch3_value < ARM_LOW_THRESHOLD) {
          armState     = 1;
          stepStartTime = now;
          seqStartTime  = now;
          Serial.println("[ARMING] Step 1: Throttle LOW — hold...");
        }
        break;

      case 1:  // Throttle LOW — wait for it to be stable, then wait for HIGH
        if (ch3_value >= ARM_LOW_THRESHOLD) {
          // Throttle moved away before step settled — restart
          armState = 0;
          Serial.println("[ARMING] Cancelled — throttle rose before step settled");
          break;
        }
        if (now - stepStartTime >= STEP_HOLD_MS) {
          // LOW confirmed — now look for HIGH
          Serial.println("[ARMING] Step 1 confirmed. Move throttle to MAXIMUM.");
          armState = 2;
        }
        // Sequence timeout
        if (now - seqStartTime > SEQUENCE_TIMEOUT_MS) {
          armState = 0;
          Serial.println("[ARMING] Timeout — restarting sequence");
        }
        break;

      case 2:  // Waiting for throttle HIGH
        if (ch3_value > ARM_HIGH_THRESHOLD) {
          Serial.println("[ARMING] Step 2: Throttle HIGH detected. Move back to MINIMUM.");
          armState     = 3;
          stepStartTime = now;
        }
        if (now - seqStartTime > SEQUENCE_TIMEOUT_MS) {
          armState = 0;
          Serial.println("[ARMING] Timeout — restarting sequence");
        }
        break;

      case 3:  // Got HIGH — now wait for LOW again to confirm arm
        if (ch3_value >= ARM_LOW_THRESHOLD) {
          // Still waiting — check timeout
          if (now - seqStartTime > SEQUENCE_TIMEOUT_MS) {
            armState = 0;
            Serial.println("[ARMING] Timeout — restarting sequence");
          }
          break;
        }
        // Throttle is LOW again — arm!
        motorsArmed = true;
        armState    = 0;
        Serial.println("\n*** MOTORS ARMED ***");
        Serial.println("*** CAUTION: MOTORS WILL SPIN ***\n");
        break;
    }

  } else {
    // ---- Disarm logic ----
    static unsigned long lowThrottleStart = 0;
    static bool          disarmWarned     = false;

    if (ch3_value < DISARM_THRESHOLD) {
      if (lowThrottleStart == 0) {
        lowThrottleStart = millis();
        disarmWarned     = false;
      }
      unsigned long heldMs = millis() - lowThrottleStart;
      if (!disarmWarned && heldMs > 1000) {
        Serial.print("[DISARM] Throttle low — disarming in ");
        Serial.print((DISARM_HOLD_MS - heldMs) / 1000 + 1);
        Serial.println("s...");
        disarmWarned = true;
      }
      if (heldMs >= DISARM_HOLD_MS) {
        motorsArmed      = false;
        lowThrottleStart = 0;
        disarmWarned     = false;
        Serial.println("\n*** MOTORS DISARMED ***\n");
      }
    } else {
      // Throttle came back up — cancel pending disarm
      if (lowThrottleStart != 0) {
        lowThrottleStart = 0;
        disarmWarned     = false;
      }
    }
  }

  // ---- Periodic status print ----
  static unsigned long lastArmStatus = 0;
  if (millis() - lastArmStatus > 3000) {
    lastArmStatus = millis();
    Serial.print("[STATUS] Armed: ");
    Serial.print(motorsArmed ? "YES" : "NO");
    if (!motorsArmed) {
      Serial.print(" | Throttle: ");
      Serial.print(ch3_value);
      Serial.print(" | ArmState: ");
      Serial.print(armState);
    }
    Serial.println();
  }
}

// ============================================
//        DEBUG OUTPUT
// ============================================
void debugPrint() {
  // Print detailed status every 500ms
  if (millis() - lastDebugPrint > 500) {
    lastDebugPrint = millis();

    Serial.println("\n========================================");
    Serial.println("        FLIGHT CONTROLLER STATUS");
    Serial.println("========================================");

    // System Status
    Serial.println("--- SYSTEM ---");
    Serial.print("Armed: ");
    Serial.println(motorsArmed ? "YES ✓" : "NO ✗");
    Serial.print("Loop Time: ");
    Serial.print(dt * 1000, 1);
    Serial.println(" ms");

    // Receiver Inputs
    Serial.println("\n--- RECEIVER INPUTS ---");
    Serial.print("CH1 (Roll):     ");
    Serial.print(ch1_value);
    Serial.print(" µs → Desired: ");
    Serial.print(desired_roll, 1);
    Serial.println("°");

    Serial.print("CH2 (Pitch):    ");
    Serial.print(ch2_value);
    Serial.print(" µs → Desired: ");
    Serial.print(desired_pitch, 1);
    Serial.println("°");

    Serial.print("CH3 (Throttle): ");
    Serial.print(ch3_value);
    Serial.print(" µs → Power: ");
    Serial.print(throttle);
    Serial.println(" µs");

    Serial.print("CH4 (Yaw):      ");
    Serial.print(ch4_value);
    Serial.print(" µs → Rate: ");
    Serial.print(desired_yaw_rate, 1);
    Serial.println("°/s");

    // PID Outputs
    Serial.println("\n--- PID CORRECTIONS ---");
    Serial.print("Roll:  ");
    Serial.print(pid_roll, 1);
    Serial.print(" | Error: ");
    Serial.println(error_roll, 2);

    Serial.print("Pitch: ");
    Serial.print(pid_pitch, 1);
    Serial.print(" | Error: ");
    Serial.println(error_pitch, 2);

    Serial.print("Yaw:   ");
    Serial.print(pid_yaw, 1);
    Serial.print(" | Error: ");
    Serial.println(error_yaw, 2);

    // Motor Outputs
    Serial.println("\n--- MOTOR OUTPUTS (µs) ---");
    Serial.print("Front Left:  ");
    Serial.print(motor_fl);
    if (motor_fl > 1000) Serial.print(" ✓ ACTIVE");
    Serial.println();

    Serial.print("Front Right: ");
    Serial.print(motor_fr);
    if (motor_fr > 1000) Serial.print(" ✓ ACTIVE");
    Serial.println();

    Serial.print("Back Left:   ");
    Serial.print(motor_bl);
    if (motor_bl > 1000) Serial.print(" ✓ ACTIVE");
    Serial.println();

    Serial.print("Back Right:  ");
    Serial.print(motor_br);
    if (motor_br > 1000) Serial.print(" ✓ ACTIVE");
    Serial.println();

    // Warnings
    Serial.println("\n--- DIAGNOSTICS ---");
    if (ch1_value == 0 || ch2_value == 0 || ch3_value == 0 || ch4_value == 0) {
      Serial.println("⚠ WARNING: NO RECEIVER SIGNAL DETECTED!");
      Serial.println("  Check receiver connections and power");
    }

    if (!motorsArmed && throttle > 1100) {
      Serial.println("⚠ Motors NOT armed. Arming sequence:");
      Serial.println("  1. Move throttle to MINIMUM");
      Serial.println("  2. Move throttle to MAXIMUM");
      Serial.println("  3. Move throttle back to MINIMUM");
    }

    if (motorsArmed && throttle < 1100) {
      Serial.println("⚠ Motors armed but throttle too low");
      Serial.println("  Increase throttle to spin motors");
    }

    if (motorsArmed && throttle > 1100 && motor_fl == 1000) {
      Serial.println("⚠ ERROR: Motors should be spinning but aren't!");
      Serial.println("  Check ESC connections and calibration");
    }

    Serial.println("========================================\n");
  }
}
