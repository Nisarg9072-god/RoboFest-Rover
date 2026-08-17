/*
 * rover_firmware.ino
 * ==================
 * Arduino Mega 2560 — ROBOFEST Autonomous Rover
 * Low-level real-time motor control firmware
 *
 * Responsibilities:
 *   - Receive velocity commands from Raspberry Pi via USB Serial
 *   - Convert (linear_x, angular_z) to left/right wheel target velocities
 *   - Run 6x closed-loop PID controllers for each motor
 *   - Read 6x quadrature wheel encoders via hardware and pin-change interrupts
 *   - Send encoder odometry and status feedback to Raspberry Pi
 *   - Implement watchdog timer (zero PWM if no command received)
 *   - Monitor hardware E-stop status
 *   - Report faults via serial
 *
 * Status: PLANNED
 *
 * IMPORTANT BEFORE FLASHING:
 *   1. Measure motor stall current — verify motor driver ratings FIRST.
 *   2. Verify all TBD pin assignments on the physical wiring diagram.
 *   3. Verify encoder counts per revolution (TBD) from motor datasheet.
 *   4. Tune PID gains (all TBD) through motor testing (Level 6 test plan).
 *   5. Verify wheel_base and wheel_radius from physical measurements.
 *
 * Serial protocol:
 *   RPi → Arduino: <CMD_VEL,linear_x,angular_z,seq,checksum\n>
 *                  <ARM\n>  <DISARM\n>  <STOP\n>  <RESET\n>  <PING\n>
 *   Arduino → RPi: <ENC,vl1,vl2,vl3,vr1,vr2,vr3,seq,checksum\n>
 *                  <STATUS,batt_mv,estop,armed,wd_ok,seq\n>
 *                  <FAULT,code,description\n>
 *                  <PONG\n>
 */

#include <Arduino.h>

// =============================================================================
// CONFIGURATION — ALL VALUES MARKED TBD MUST BE MEASURED/VERIFIED
// =============================================================================

// Serial communication
#define SERIAL_BAUD       115200
#define SERIAL_TIMEOUT_MS 500      // TBD: watchdog — ms before zeroing motors

// Robot geometry — TBD: measure from physical rover
const float WHEEL_BASE_M   = 0.30f;  // TBD: distance between left/right wheel centrelines (m)
const float WHEEL_RADIUS_M = 0.05f;  // TBD: wheel radius (m)

// Encoder resolution — TBD: get from motor datasheet
const float TICKS_PER_REV = 1440.0f; // TBD: quadrature encoder counts per wheel revolution
const float RAD_PER_TICK  = (2.0f * PI) / TICKS_PER_REV;

// PID gains — TBD: tune experimentally using Level 6 test (motor PID test)
// Start with Kp only. Increase until oscillation. Add Kd to dampen. Add Ki for offset.
struct PIDGains {
  float kp;  // TBD
  float ki;  // TBD
  float kd;  // TBD
};

// One set of gains per side (left / right). Tune independently if motors differ.
const PIDGains LEFT_GAINS  = {1.0f, 0.1f, 0.01f};  // TBD
const PIDGains RIGHT_GAINS = {1.0f, 0.1f, 0.01f};  // TBD

// PWM limits
const int PWM_MAX   = 255;
const int PWM_MIN   = 0;
const int PWM_DEAD  = 10;   // TBD: minimum PWM that causes motor motion (deadband)

// Battery monitoring — TBD: calibrate ADC reading against actual voltage
const int   BATT_ADC_PIN         = A0;         // TBD: pin for battery voltage divider
const float BATT_VOLTAGE_DIVIDER = 10.0f;      // TBD: divider ratio (verify with multimeter)
const float BATT_ADC_REF         = 5.0f;       // Arduino Mega AREF (5V)
const float BATT_LOW_MV          = 21000.0f;   // TBD
const float BATT_CRITICAL_MV     = 19200.0f;   // TBD

// E-Stop input pin (normally HIGH = safe; LOW = E-Stop pressed)
const int ESTOP_PIN = 2;  // TBD: wire to hardware E-Stop signal (if relay has a status output)

// =============================================================================
// PIN ASSIGNMENTS — ALL TBD: assign after physical wiring
// =============================================================================

// Motor Left Front (Motor 0)
const int M0_PWM = 4;   // TBD
const int M0_DIR = 5;   // TBD
const int M0_ENC_A = 18; // Hardware interrupt pin — TBD
const int M0_ENC_B = 19; // Hardware interrupt pin — TBD

// Motor Left Mid (Motor 1)
const int M1_PWM = 6;   // TBD
const int M1_DIR = 7;   // TBD
const int M1_ENC_A = 20; // Hardware interrupt pin — TBD
const int M1_ENC_B = 21; // Hardware interrupt pin — TBD

// Motor Left Rear (Motor 2)
const int M2_PWM = 8;   // TBD
const int M2_DIR = 9;   // TBD
const int M2_ENC_A = 3; // TBD (pin-change interrupt)
const int M2_ENC_B = 11; // TBD

// Motor Right Front (Motor 3)
const int M3_PWM = 22;  // TBD
const int M3_DIR = 23;  // TBD
const int M3_ENC_A = 24; // TBD (pin-change interrupt)
const int M3_ENC_B = 25; // TBD

// Motor Right Mid (Motor 4)
const int M4_PWM = 26;  // TBD
const int M4_DIR = 27;  // TBD
const int M4_ENC_A = 28; // TBD (pin-change interrupt)
const int M4_ENC_B = 29; // TBD

// Motor Right Rear (Motor 5)
const int M5_PWM = 30;  // TBD
const int M5_DIR = 31;  // TBD
const int M5_ENC_A = 32; // TBD (pin-change interrupt)
const int M5_ENC_B = 33; // TBD

// =============================================================================
// GLOBAL STATE
// =============================================================================

// Encoder tick counters (updated in ISR — must be volatile)
volatile long enc_ticks[6] = {0, 0, 0, 0, 0, 0};
long prev_ticks[6]         = {0, 0, 0, 0, 0, 0};

// Wheel angular velocities (rad/s), calculated from encoder deltas
float wheel_omega[6] = {0.0f};

// Target wheel angular velocities (rad/s) — set by velocity calculator
float target_omega[6] = {0.0f};

// PID state per motor
float pid_integral[6]  = {0.0f};
float pid_prev_err[6]  = {0.0f};

// Motor arm state
bool motors_armed = false;

// Watchdog
unsigned long last_cmd_ms = 0;
bool watchdog_ok = true;

// Sequence number for feedback packets
uint16_t fb_seq = 0;

// PID loop timing
unsigned long last_pid_ms = 0;
const unsigned long PID_PERIOD_MS = 10;  // 100 Hz PID loop

// Feedback timing
unsigned long last_fb_ms = 0;
const unsigned long FB_PERIOD_MS = 20;   // 50 Hz feedback to Raspberry Pi

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD);

  // Motor driver pins
  int pwm_pins[] = {M0_PWM, M1_PWM, M2_PWM, M3_PWM, M4_PWM, M5_PWM};
  int dir_pins[] = {M0_DIR, M1_DIR, M2_DIR, M3_DIR, M4_DIR, M5_DIR};
  for (int i = 0; i < 6; i++) {
    pinMode(pwm_pins[i], OUTPUT);
    pinMode(dir_pins[i], OUTPUT);
    analogWrite(pwm_pins[i], 0);
    digitalWrite(dir_pins[i], LOW);
  }

  // Encoder pins
  // Hardware interrupts (2, 3, 18, 19, 20, 21 on Mega) — TBD
  attachInterrupt(digitalPinToInterrupt(M0_ENC_A), isr_enc0_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), isr_enc1_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), isr_enc2_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M3_ENC_A), isr_enc3_a, CHANGE);

  // TBD: configure pin-change interrupts for M4, M5 if hardware interrupts exhausted
  // PCICR |= (1 << PCIE2);
  // PCMSK2 |= (1 << PCINT20) | (1 << PCINT21);

  // E-Stop input
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  // Battery ADC
  pinMode(BATT_ADC_PIN, INPUT);

  // Watchdog
  last_cmd_ms = millis();

  Serial.println(F("<STATUS,0,0,0,1,0>"));  // Initial status
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
  unsigned long now = millis();

  // ── Parse incoming serial ──────────────────────────────────────────────────
  if (Serial.available()) {
    parseSerial();
  }

  // ── Watchdog check ─────────────────────────────────────────────────────────
  if ((now - last_cmd_ms) > SERIAL_TIMEOUT_MS) {
    if (motors_armed) {
      stopAllMotors();
      watchdog_ok = false;
      sendFault("WATCHDOG", "CMD timeout — motors zeroed");
    }
  } else {
    watchdog_ok = true;
  }

  // ── PID loop (100 Hz) ─────────────────────────────────────────────────────
  if ((now - last_pid_ms) >= PID_PERIOD_MS) {
    float dt = (now - last_pid_ms) / 1000.0f;
    last_pid_ms = now;
    updateWheelVelocities(dt);
    if (motors_armed) {
      runPIDAll(dt);
    }
  }

  // ── Send feedback (50 Hz) ────────────────────────────────────────────────
  if ((now - last_fb_ms) >= FB_PERIOD_MS) {
    last_fb_ms = now;
    sendEncoderFeedback();
    sendStatus();
  }
}

// =============================================================================
// SERIAL PARSER
// =============================================================================

char serial_buf[128];
int  serial_idx = 0;

void parseSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      serial_buf[serial_idx] = '\0';
      processCommand(String(serial_buf));
      serial_idx = 0;
    } else if (c != '<' && c != '>') {
      if (serial_idx < 127) {
        serial_buf[serial_idx++] = c;
      }
    }
  }
}

void processCommand(const String& cmd) {
  last_cmd_ms = millis();

  if (cmd.startsWith("CMD_VEL,")) {
    parseCmdVel(cmd);
  } else if (cmd == "ARM") {
    motors_armed = true;
    Serial.println(F("<STATUS,0,0,1,1,0>"));
  } else if (cmd == "DISARM") {
    motors_armed = false;
    stopAllMotors();
  } else if (cmd == "STOP") {
    stopAllMotors();
    for (int i = 0; i < 6; i++) target_omega[i] = 0.0f;
  } else if (cmd == "RESET") {
    motors_armed = false;
    stopAllMotors();
    for (int i = 0; i < 6; i++) {
      pid_integral[i] = 0.0f;
      pid_prev_err[i] = 0.0f;
    }
    watchdog_ok = true;
  } else if (cmd == "PING") {
    Serial.println(F("<PONG>"));
  }
}

// =============================================================================
// VELOCITY COMMAND
// =============================================================================

void parseCmdVel(const String& cmd) {
  /*
   * Parse: CMD_VEL,linear_x,angular_z,seq,checksum
   * Convert to left/right wheel group velocities.
   */
  int idx1 = cmd.indexOf(',');
  int idx2 = cmd.indexOf(',', idx1 + 1);
  int idx3 = cmd.indexOf(',', idx2 + 1);

  if (idx1 < 0 || idx2 < 0 || idx3 < 0) {
    sendFault("PARSE", "Malformed CMD_VEL");
    return;
  }

  float linear_x  = cmd.substring(idx1 + 1, idx2).toFloat();
  float angular_z = cmd.substring(idx2 + 1, idx3).toFloat();

  // Differential drive kinematics:
  //   V_left  = linear_x - angular_z * (wheel_base / 2)
  //   V_right = linear_x + angular_z * (wheel_base / 2)
  float v_left  = linear_x - angular_z * (WHEEL_BASE_M / 2.0f);
  float v_right = linear_x + angular_z * (WHEEL_BASE_M / 2.0f);

  // Convert linear velocity (m/s) → angular velocity (rad/s)
  float omega_left  = v_left  / WHEEL_RADIUS_M;
  float omega_right = v_right / WHEEL_RADIUS_M;

  // Assign to all wheels on each side
  target_omega[0] = omega_left;   // Left Front
  target_omega[1] = omega_left;   // Left Mid
  target_omega[2] = omega_left;   // Left Rear
  target_omega[3] = omega_right;  // Right Front
  target_omega[4] = omega_right;  // Right Mid
  target_omega[5] = omega_right;  // Right Rear
}

// =============================================================================
// ENCODER VELOCITY
// =============================================================================

void updateWheelVelocities(float dt) {
  /*
   * Calculate wheel angular velocity (rad/s) from encoder tick delta.
   * Called at PID rate (100 Hz).
   */
  for (int i = 0; i < 6; i++) {
    long ticks;
    noInterrupts();
    ticks = enc_ticks[i];
    interrupts();

    long delta = ticks - prev_ticks[i];
    prev_ticks[i] = ticks;

    // rad/s = (ticks/tick_per_rev) * 2pi / dt = ticks * rad_per_tick / dt
    if (dt > 0.0f) {
      wheel_omega[i] = (float)delta * RAD_PER_TICK / dt;
    }
  }
}

// =============================================================================
// PID CONTROLLER
// =============================================================================

float pidCompute(int motor_idx, float target, float measured, float dt, const PIDGains& gains) {
  float error      = target - measured;
  pid_integral[motor_idx] += error * dt;

  // Anti-windup: clamp integral
  pid_integral[motor_idx] = constrain(pid_integral[motor_idx], -100.0f, 100.0f);

  float derivative = (dt > 0.0f) ? ((error - pid_prev_err[motor_idx]) / dt) : 0.0f;
  pid_prev_err[motor_idx] = error;

  return gains.kp * error + gains.ki * pid_integral[motor_idx] + gains.kd * derivative;
}

void runPIDAll(float dt) {
  int pwm_pins[] = {M0_PWM, M1_PWM, M2_PWM, M3_PWM, M4_PWM, M5_PWM};
  int dir_pins[] = {M0_DIR, M1_DIR, M2_DIR, M3_DIR, M4_DIR, M5_DIR};

  for (int i = 0; i < 6; i++) {
    const PIDGains& gains = (i < 3) ? LEFT_GAINS : RIGHT_GAINS;
    float output = pidCompute(i, target_omega[i], wheel_omega[i], dt, gains);

    // Direction
    if (output >= 0) {
      digitalWrite(dir_pins[i], HIGH);
    } else {
      digitalWrite(dir_pins[i], LOW);
      output = -output;
    }

    // Apply deadband and clamp
    int pwm = (int)output;
    if (pwm < PWM_DEAD && target_omega[i] != 0.0f) {
      pwm = PWM_DEAD;
    }
    pwm = constrain(pwm, PWM_MIN, PWM_MAX);
    analogWrite(pwm_pins[i], pwm);
  }
}

// =============================================================================
// MOTOR STOP
// =============================================================================

void stopAllMotors() {
  int pwm_pins[] = {M0_PWM, M1_PWM, M2_PWM, M3_PWM, M4_PWM, M5_PWM};
  for (int i = 0; i < 6; i++) {
    analogWrite(pwm_pins[i], 0);
    pid_integral[i] = 0.0f;
  }
  for (int i = 0; i < 6; i++) {
    target_omega[i] = 0.0f;
  }
}

// =============================================================================
// FEEDBACK
// =============================================================================

void sendEncoderFeedback() {
  /*
   * Send wheel velocities (rad/s) to Raspberry Pi.
   * Format: <ENC,vl1,vl2,vl3,vr1,vr2,vr3,seq,chk>
   */
  uint8_t chk = 0;
  char buf[128];
  snprintf(buf, sizeof(buf),
    "ENC,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u",
    wheel_omega[0], wheel_omega[1], wheel_omega[2],
    wheel_omega[3], wheel_omega[4], wheel_omega[5],
    fb_seq
  );
  // Simple checksum: XOR of all chars
  for (int i = 0; buf[i] != '\0'; i++) chk ^= (uint8_t)buf[i];
  Serial.print('<');
  Serial.print(buf);
  Serial.print(',');
  Serial.print(chk);
  Serial.println('>');
  fb_seq++;
}

void sendStatus() {
  /*
   * Format: <STATUS,batt_mv,estop,armed,wd_ok,seq>
   */
  float batt_mv = readBatteryMv();
  bool  estop   = (digitalRead(ESTOP_PIN) == LOW);

  char buf[64];
  snprintf(buf, sizeof(buf),
    "STATUS,%.0f,%d,%d,%d,%u",
    batt_mv, (int)estop, (int)motors_armed, (int)watchdog_ok, fb_seq
  );
  Serial.print('<');
  Serial.print(buf);
  Serial.println('>');
}

void sendFault(const char* code, const char* description) {
  Serial.print(F("<FAULT,"));
  Serial.print(code);
  Serial.print(',');
  Serial.print(description);
  Serial.println(F(">"));
}

// =============================================================================
// BATTERY MONITORING
// =============================================================================

float readBatteryMv() {
  /*
   * Read battery voltage via voltage divider on ADC pin.
   * Formula: V_batt = (ADC_raw / 1023.0) * AREF * DIVIDER_RATIO
   * TBD: calibrate BATT_VOLTAGE_DIVIDER against actual battery voltage.
   */
  int raw = analogRead(BATT_ADC_PIN);
  float v_adc = (raw / 1023.0f) * BATT_ADC_REF;
  return v_adc * BATT_VOLTAGE_DIVIDER * 1000.0f; // returns mV
}

// =============================================================================
// ENCODER ISRs
// TBD: Implement proper quadrature decoding (A+B state machine)
//       Simple version: count on A edge only (single-channel mode).
//       Full quadrature: track A and B state for direction + 4x resolution.
// =============================================================================

void isr_enc0_a() {
  // TBD: add direction detection using B channel
  // Example full quadrature:
  // bool a = digitalRead(M0_ENC_A); bool b = digitalRead(M0_ENC_B);
  // enc_ticks[0] += (a == b) ? +1 : -1;
  enc_ticks[0]++;  // Placeholder — replace with full quadrature
}

void isr_enc1_a() { enc_ticks[1]++; }
void isr_enc2_a() { enc_ticks[2]++; }
void isr_enc3_a() { enc_ticks[3]++; }
// TBD: enc4 and enc5 via pin-change ISR (PCINT)
// ISR(PCINT2_vect) { ... }
