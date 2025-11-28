/*
 * Activity recognition + TENS feedback
 * Sensors: MPU6050 (gyro Y) + dual foot-pressure sensors
 * States: WALK / OBSTACLE / STAIRS / DRIVE
 * TENS stimulation is modulated according to the detected state
 * 
 * ========== State determination (priority order) ==========
 * 1. DRIVE:
 *    - Gyro standard deviation < 30 (ankle nearly static)
 *    - Front-foot pressure > 1200 g (pedal press)
 *    - Rear-foot pressure < 2000 g (light support)
 * 2. OBSTACLE:
 *    - Gyro peak > 350 (fast leg swing)
 *    - |Front - Rear| pressure > 4000 g (sudden shift)
 * 3. STAIRS:
 *    - Gyro standard deviation > 60 (continuous motion)
 *    - Up: gyro mean > 0 & front > 1500 g
 *    - Down: gyro mean < 0 & rear > 1500 g
 * 4. WALK:
 *    - Default (none of the above), then gait phase is detected
 * 
 * ========== Output meaning ==========
 * State: final stable state
 * Gait: gait phase (WALK only)
 * GyY / GyY_Std: current gyro Y and std
 * Front/Rear: average pressure (g)
 * f: TENS frequency (Hz)
 * I1/I2: TENS intensity for front/rear channels (0-20)
 */

#include <Wire.h>
#include <MPU6050.h>

#define DEBUG Serial

// ======== Hardware definitions ========
const uint8_t I2C_ADDR = 0x01;
const uint8_t FRONT_SENSOR_PIN = A2;
const uint8_t REAR_SENSOR_PIN  = A3;
// ======== IMU configuration ========
MPU6050 mpu(0x68);
int32_t gy_offset = 0;
const uint16_t CALIB_SAMPLES = 500;

// ======== Pressure sensor parameters ========
#define VOLTAGE_MIN 150
#define VOLTAGE_MAX 3300
#define PRESS_MIN   500
#define PRESS_MAX   20000

// ======== State definitions ========
enum ActivityState { WALK, OBSTACLE, STAIRS, DRIVE };              // High-level activities
enum GaitPhase { SWING, INITIAL_CONTACT, MID_STANCE, TERMINAL_STANCE };  // Walk sub-phases

// ======== State thresholds ========
const float GYRO_Y_DRIVE_MAX = 30.0f;    // DRIVE: low gyro variance (almost static)
const float GYRO_Y_OBS_PEAK = 350.0f;    // OBSTACLE: peak angular velocity
const float GYRO_Y_STAIRS_MIN = 60.0f;   // STAIRS: sustained motion
const long FRONT_ACTIVE_MIN = 1200;      // Front-foot activation threshold
const long FRONT_DRIVE_MIN = 1200;       // DRIVE: front > 1200g (pedal press)
const long REAR_DRIVE_MAX = 2000;        // DRIVE: rear < 2000g (light support)
const long PRESS_DIFF_OBS = 4000;        // OBSTACLE: |front - rear| > 4000g
const long PRESS_STAIRS_FRONT = 1500;    // STAIRS up: front > 1500g
const long PRESS_STAIRS_REAR = 1500;     // STAIRS down: rear > 1500g

// ======== Gait thresholds ========
const long PRESS_SWING_MAX = 800;        // SWING: total pressure < 800g
const long PRESS_CONTACT_MIN = 1500;     // Heel strike: rear > 1500g
const long PRESS_TERMINAL_FRONT = 1800;  // Push-off: front > 1800g
const float ANGLE_INITIAL_MAX = -2.0f;   // Heel strike angle ≤ -2°
const float ANGLE_MID_MIN = 3.0f;        // Mid stance angle ≥ 3°
const float ANGLE_TERMINAL_MIN = 12.0f;  // Push-off angle ≥ 12°

// ======== TENS parameters ========
const float P0_g = 200.0f;
const float P1_g = 3000.0f;
const float GAMMA = 0.5f;

const float FREQ_MIN = 5.0f;
const float FREQ_MAX = 25.0f;
const uint16_t PULSE_US = 200;
const uint8_t INTENS_MIN = 0;
const uint8_t INTENS_MAX = 20;
const uint8_t INTENS_STEP_MAX = 2;
const float FREQ_STEP_MAX = 2.0f;

const float DRIVE_ACCEL_THRESHOLD = 50.0f;
const float DRIVE_PRESS_RATE_THRESHOLD = 100.0f;

// ======== Timing parameters ========
const uint32_t SEND_HZ = 100;
const uint32_t SEND_DT_US = 1000000UL / SEND_HZ;
const uint32_t SENSE_DT_MS = 100;
const uint16_t WINDOW_MS = 1000;
const uint8_t SAMPLE_INTERVAL_MS = 50;
const uint8_t MAX_SAMPLES = WINDOW_MS / SAMPLE_INTERVAL_MS;
const float HYST_g = 50.0f;
const uint16_t ARM_PRESS_MS = 200;
const uint16_t NO_PRESS_OFF_MS = 150;

// ======== Data buffers ========
int16_t gyBuf[MAX_SAMPLES];
long frontBuf[MAX_SAMPLES];
long rearBuf[MAX_SAMPLES];
uint8_t bufIdx = 0;
bool bufferFilled = false;

float ankleAngle = 0.0f;
GaitPhase currentGaitPhase = SWING;
ActivityState lastState = WALK;
uint8_t stateStableCount = 0;
const uint8_t STATE_STABLE_THRESHOLD = 2;

// TENS control variables
float freq_curr = 10.0f, freq_target = 10.0f;
uint8_t inten1_curr = 0, inten2_curr = 0;
uint8_t inten1_target = 0, inten2_target = 0;

long lastFrontPress = 0;
long lastRearPress = 0;
int16_t lastGyroY = 0;
unsigned long lastPressTime = 0;

uint8_t frame[15] = {
  0xFA, 0x01,
  0x05, 0xC8, 0x00, 0x14,
  0x05, 0xC8, 0x00, 0x14,
  0x00, 0x00,
  0x00, 0x00,
  0xAF
};

// ======== Utility helpers ========
int mvFromADC(int adc) { return map(adc, 0, 1023, 0, 5000); }

long gFromMilliVolt(int mv) {
  if (mv <= VOLTAGE_MIN) return 0;
  if (mv >= VOLTAGE_MAX) return PRESS_MAX;
  return map(mv, VOLTAGE_MIN, VOLTAGE_MAX, PRESS_MIN, PRESS_MAX);
}

float compressNorm(long g) {
  float n = (g - P0_g) / (P1_g - P0_g);
  if (n < 0) n = 0;
  if (n > 1) n = 1;
  n = pow(n, GAMMA);
  if (n < 0.02f) n = 0;
  return n;
}

float slewFloat(float current, float target, float step) {
  if (target > current + step) return current + step;
  if (target < current - step) return current - step;
  return target;
}

uint8_t slewByte(uint8_t current, uint8_t target, uint8_t step) {
  if (target > current + step) return current + step;
  if (target < current - step) return current - step;
  return target;
}

void setWaveformByFreq(float freq_hz) {
  uint32_t T_us = (uint32_t)(1000000.0f / freq_hz);
  uint32_t B_us = PULSE_US, D_us = PULSE_US;
  uint32_t remain = (T_us > (B_us + D_us)) ? (T_us - (B_us + D_us)) : 0;
  uint32_t A_us = remain / 2, C_us = remain / 2;

  uint16_t A_step = A_us / 10;
  uint16_t B_step = B_us / 10;
  uint16_t C_step = C_us / 10;
  uint16_t D_step = D_us / 10;

  frame[2]=(A_step>>8); frame[3]=(A_step&0xFF);
  frame[4]=(B_step>>8); frame[5]=(B_step&0xFF);
  frame[6]=(C_step>>8); frame[7]=(C_step&0xFF);
  frame[8]=(D_step>>8); frame[9]=(D_step&0xFF);
}

int readSmooth(int pin) {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100);
  }
  return sum / 10;
}

// ======== IMU helpers ========
void calibrateYAxis() {
  gy_offset = 0;
  int16_t gx, gy, gz;
  DEBUG.println(F("Calibrating... keep the ankle still and parallel to the ground"));
  for (uint16_t i = 0; i < CALIB_SAMPLES; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    gy_offset += gy;
    delay(2);
  }
  gy_offset /= CALIB_SAMPLES;
  DEBUG.print(F("Gyro Y offset: "));
  DEBUG.println(gy_offset);
}

void updateAnkleAngle(int16_t gyroY) {
  float gyroY_degPerSec = gyroY / 131.0f;
  ankleAngle += gyroY_degPerSec * (SAMPLE_INTERVAL_MS / 1000.0f);
  if (abs(gyroY) < 20) {
    if (ankleAngle > 0.5f) ankleAngle -= 0.5f;
    else if (ankleAngle < -0.5f) ankleAngle += 0.5f;
    else ankleAngle = 0.0f;
  }
  if (ankleAngle > 30.0f) ankleAngle = 30.0f;
  if (ankleAngle < -30.0f) ankleAngle = -30.0f;
}

float computeStd(int16_t *buffer) {
  uint8_t samples = bufferFilled ? MAX_SAMPLES : bufIdx;
  if (samples == 0) return 0;
  float mean = 0;
  for (uint8_t i = 0; i < samples; i++) mean += buffer[i];
  mean /= samples;
  float var = 0;
  for (uint8_t i = 0; i < samples; i++) {
    float diff = buffer[i] - mean;
    var += diff * diff;
  }
  return sqrt(var / samples);
}

float computeMean(int16_t *buffer) {
  uint8_t samples = bufferFilled ? MAX_SAMPLES : bufIdx;
  if (samples == 0) return 0;
  float sum = 0;
  for (uint8_t i = 0; i < samples; i++) sum += buffer[i];
  return sum / samples;
}

// ======== State recognition ========
GaitPhase detectGaitPhase(long front, long rear, float angle) {
  long totalPress = front + rear;
  if (totalPress < PRESS_SWING_MAX) return SWING;                                // Foot lifted
  if (rear > PRESS_CONTACT_MIN && angle <= ANGLE_INITIAL_MAX) return INITIAL_CONTACT;   // Heel strike
  if (front > PRESS_TERMINAL_FRONT && angle >= ANGLE_TERMINAL_MIN) return TERMINAL_STANCE; // Push-off
  if (angle >= ANGLE_MID_MIN && angle < ANGLE_TERMINAL_MIN) return MID_STANCE;   // Mid stance
  return currentGaitPhase;
}

ActivityState classify() {
  uint8_t samples = bufferFilled ? MAX_SAMPLES : bufIdx;
  if (samples == 0) return WALK;

  float gyroYStd = computeStd(gyBuf);                 // variance (activity level)
  float gyroYMean = computeMean(gyBuf);               // direction (+up / -down)
  int16_t latestGy = gyBuf[(bufIdx + MAX_SAMPLES - 1) % MAX_SAMPLES];
  float gyroYPeak = abs(latestGy);                    // instantaneous peak

  long frontMean = 0, rearMean = 0;
  long maxPressDiff = 0;
  long frontMax = 0, rearMax = 0;
  for (uint8_t i = 0; i < samples; i++) {
    frontMean += frontBuf[i];
    rearMean += rearBuf[i];
    long diff = labs(frontBuf[i] - rearBuf[i]);
    if (diff > maxPressDiff) maxPressDiff = diff;
    if (frontBuf[i] > frontMax) frontMax = frontBuf[i];
    if (rearBuf[i] > rearMax) rearMax = rearBuf[i];
  }
  frontMean /= samples;
  rearMean /= samples;

  bool frontDrive = frontMean > FRONT_DRIVE_MIN;     // pedal press
  bool rearLight = rearMean < REAR_DRIVE_MAX;        // light rear support
  bool frontStairs = frontMean > PRESS_STAIRS_FRONT; // ascending
  bool rearStairs = rearMean > PRESS_STAIRS_REAR;    // descending

  // Priority: DRIVE > OBSTACLE > STAIRS > WALK
  if (gyroYStd < GYRO_Y_DRIVE_MAX && frontDrive && rearLight) return DRIVE;
  if (gyroYPeak > GYRO_Y_OBS_PEAK && maxPressDiff > PRESS_DIFF_OBS) return OBSTACLE;
  if (gyroYStd > GYRO_Y_STAIRS_MIN) {
    if ((gyroYMean > 0 && frontStairs) || (gyroYMean < 0 && rearStairs)) return STAIRS;
  }
  return WALK;
}

const char* gaitPhaseToText(GaitPhase phase) {
  switch (phase) {
    case SWING: return "SWING";
    case INITIAL_CONTACT: return "INITIAL_CONTACT";
    case MID_STANCE: return "MID_STANCE";
    case TERMINAL_STANCE: return "TERMINAL_STANCE";
    default: return "UNKNOWN";
  }
}

// ======== TENS control logic ========
void controlTENS(ActivityState state, GaitPhase phase, long front, long rear, int16_t gyroY) {
  float n1 = compressNorm(front);
  float n2 = compressNorm(rear);

  switch (state) {
    case DRIVE: {
      unsigned long now = millis();
      long frontRate = 0, rearRate = 0;
      float gyroRate = 0;
      if (lastPressTime > 0 && (now - lastPressTime) >= SENSE_DT_MS) {
        frontRate = labs(front - lastFrontPress) * 1000 / (now - lastPressTime);
        rearRate = labs(rear - lastRearPress) * 1000 / (now - lastPressTime);
        gyroRate = abs(gyroY - lastGyroY) * 1000.0f / (now - lastPressTime);
      }
      bool accelerating = (frontRate > DRIVE_PRESS_RATE_THRESHOLD) || (gyroRate > DRIVE_ACCEL_THRESHOLD);  // detect acceleration vs constant speed
      if (accelerating) {
        float rateNorm = min(frontRate / 500.0f, 1.0f);
        n1 = max(n1, rateNorm * 0.8f);
        freq_target = FREQ_MIN + n1 * (FREQ_MAX - FREQ_MIN) * 1.2f;  // +20% frequency boost
        if (freq_target > FREQ_MAX) freq_target = FREQ_MAX;
      } else {
        freq_target = FREQ_MIN + n1 * (FREQ_MAX - FREQ_MIN);
      }
      inten1_target = (uint8_t)(INTENS_MIN + n1 * (INTENS_MAX - INTENS_MIN));        // front channel dominant
      inten2_target = (uint8_t)(INTENS_MIN + n2 * 0.5f * (INTENS_MAX - INTENS_MIN)); // rear at 50%
      lastFrontPress = front;
      lastRearPress = rear;
      lastGyroY = gyroY;
      lastPressTime = now;
      break;
    }

    case WALK:  // adjust stimulation according to gait phase
      switch (phase) {
        case INITIAL_CONTACT:  // heel strike
          inten1_target = (uint8_t)(INTENS_MIN + n1 * 0.6f * (INTENS_MAX - INTENS_MIN));
          inten2_target = (uint8_t)(INTENS_MIN + n2 * (INTENS_MAX - INTENS_MIN));
          freq_target = FREQ_MIN + max(n1, n2) * (FREQ_MAX - FREQ_MIN);
          break;
        case MID_STANCE:       // mid stance
          inten1_target = (uint8_t)(INTENS_MIN + n1 * 0.8f * (INTENS_MAX - INTENS_MIN));
          inten2_target = (uint8_t)(INTENS_MIN + n2 * 0.8f * (INTENS_MAX - INTENS_MIN));
          freq_target = FREQ_MIN + max(n1, n2) * 0.9f * (FREQ_MAX - FREQ_MIN);
          break;
        case TERMINAL_STANCE:  // push-off
          inten1_target = (uint8_t)(INTENS_MIN + n1 * (INTENS_MAX - INTENS_MIN));
          inten2_target = (uint8_t)(INTENS_MIN + n2 * 0.5f * (INTENS_MAX - INTENS_MIN));
          freq_target = FREQ_MIN + n1 * (FREQ_MAX - FREQ_MIN);
          break;
        case SWING:            // leg off the ground → no stimulation
          inten1_target = 0;
          inten2_target = 0;
          freq_target = FREQ_MIN;
          break;
      }
      break;

    case OBSTACLE: {
      float gyroNorm = min(abs(gyroY) / 500.0f, 1.0f);   // IMU contribution
      float pressNorm = max(n1, n2);                     // pressure contribution
      float combined = max(gyroNorm, pressNorm);         // take larger value
      inten1_target = (uint8_t)(INTENS_MIN + combined * (INTENS_MAX - INTENS_MIN));
      inten2_target = (uint8_t)(INTENS_MIN + combined * 0.7f * (INTENS_MAX - INTENS_MIN));
      freq_target = FREQ_MIN + combined * (FREQ_MAX - FREQ_MIN) * 1.3f;  // +30% frequency
      if (freq_target > FREQ_MAX) freq_target = FREQ_MAX;
      break;
    }

    case STAIRS: {
      float gyroNorm = min(abs(gyroY) / 300.0f, 1.0f);   // IMU weighting 60%
      float pressNorm = max(n1, n2);                     // pressure weighting 40%
      float combined = gyroNorm * 0.6f + pressNorm * 0.4f;
      if (n1 > n2) {  // ascending: front foot dominant
        inten1_target = (uint8_t)(INTENS_MIN + combined * (INTENS_MAX - INTENS_MIN));
        inten2_target = (uint8_t)(INTENS_MIN + n2 * 0.5f * (INTENS_MAX - INTENS_MIN));
      } else {        // descending: rear foot dominant
        inten1_target = (uint8_t)(INTENS_MIN + n1 * 0.5f * (INTENS_MAX - INTENS_MIN));
        inten2_target = (uint8_t)(INTENS_MIN + combined * (INTENS_MAX - INTENS_MIN));
      }
      freq_target = FREQ_MIN + combined * (FREQ_MAX - FREQ_MIN);
      break;
    }
  }

  // Clamp outputs to safe boundaries
  if (freq_target > FREQ_MAX) freq_target = FREQ_MAX;
  if (freq_target < FREQ_MIN) freq_target = FREQ_MIN;
  if (inten1_target > INTENS_MAX) inten1_target = INTENS_MAX;
  if (inten2_target > INTENS_MAX) inten2_target = INTENS_MAX;
}

// ======== Main program ========
void setup() {
  DEBUG.begin(115200);
  Wire.begin();
  delay(100);

  DEBUG.println(F("=== Activity recognition + TENS system ==="));
  DEBUG.println(F("Initializing MPU6050..."));
  mpu.initialize();
  if (!mpu.testConnection()) {
    DEBUG.println(F("Error: MPU6050 connection failed!"));
    while (1);
  }
  DEBUG.println(F("MPU6050 connection OK"));
  calibrateYAxis();
  DEBUG.println(F("Calibration done. Commands: 'a'=arm, '0'=stop"));
}

void loop() {
  static unsigned long lastSample = 0;
  static unsigned long lastSense = 0;
  static unsigned long lastSend = 0;
  static bool armed = false;
  static ActivityState detectedState = WALK;
  static ActivityState prevDetectedState = WALK;

  unsigned long now = millis();
  unsigned long nowUs = micros();

  if (DEBUG.available()) {
    String cmd = DEBUG.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("a")) {
      armed = true;
      DEBUG.println(F("✔ Armed"));
    } else if (cmd == "0") {
      armed = false;
      inten1_curr = inten2_curr = 0;
      inten1_target = inten2_target = 0;
      freq_target = FREQ_MIN;
      frame[10] = frame[11] = 0;
      frame[12] = frame[13] = 0;
      Wire.beginTransmission(I2C_ADDR);
      Wire.write(frame, 15);
      Wire.endTransmission();
      DEBUG.println(F("⛔ Emergency stop: stimulation off"));
    }
  }

  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;

    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    gy -= gy_offset;
    gyBuf[bufIdx] = gy;
    updateAnkleAngle(gy);

    int mv1 = mvFromADC(readSmooth(FRONT_SENSOR_PIN));
    int mv2 = mvFromADC(readSmooth(REAR_SENSOR_PIN));
    long front = gFromMilliVolt(mv1);
    long rear = gFromMilliVolt(mv2);
    frontBuf[bufIdx] = front;
    rearBuf[bufIdx] = rear;

    bufIdx = (bufIdx + 1) % MAX_SAMPLES;
    if (bufIdx == 0) bufferFilled = true;

    prevDetectedState = detectedState;
    detectedState = classify();
    if (detectedState == prevDetectedState) {
      stateStableCount++;
    } else {
      stateStableCount = 0;
    }
    ActivityState state = lastState;
    if (stateStableCount >= STATE_STABLE_THRESHOLD) {
      state = detectedState;
      lastState = state;
    }

    GaitPhase gaitPhase = currentGaitPhase;
    if (state == WALK) {
      gaitPhase = detectGaitPhase(front, rear, ankleAngle);
      currentGaitPhase = gaitPhase;
    } else {
      currentGaitPhase = SWING;
    }

    if (armed) {
      controlTENS(state, gaitPhase, front, rear, gy);
    } else {
      inten1_target = inten2_target = 0;
      freq_target = FREQ_MIN;
    }

    inten1_curr = slewByte(inten1_curr, inten1_target, INTENS_STEP_MAX);
    inten2_curr = slewByte(inten2_curr, inten2_target, INTENS_STEP_MAX);
    freq_curr = slewFloat(freq_curr, freq_target, FREQ_STEP_MAX);
    setWaveformByFreq(freq_curr);

    frame[10] = (inten1_curr > 0) ? 1 : 0;
    frame[11] = (inten2_curr > 0) ? 1 : 0;
    frame[12] = inten1_curr;
    frame[13] = inten2_curr;

    if (now - lastSense >= SENSE_DT_MS) {
      lastSense = now;
      uint8_t samples = bufferFilled ? MAX_SAMPLES : bufIdx;
      float gyroYStd = (samples > 0) ? computeStd(gyBuf) : 0;
      long frontMean = 0, rearMean = 0;
      for (uint8_t i = 0; i < samples && i < MAX_SAMPLES; i++) {
        frontMean += frontBuf[i];
        rearMean += rearBuf[i];
      }
      if (samples > 0) {
        frontMean /= samples;
        rearMean /= samples;
      }

      const char* stateNames[] = {"WALK", "OBSTACLE", "STAIRS", "DRIVE"};
      DEBUG.print(stateNames[state]);
      if (state == WALK) {
        const char* gaitNames[] = {"SWING", "INIT_CONTACT", "MID_STANCE", "TERM_STANCE"};
        DEBUG.print(' ');
        DEBUG.print(gaitNames[gaitPhase]);
      }
      DEBUG.print(F(" | GyY:")); DEBUG.print(gy);
      DEBUG.print(F(" Std:")); DEBUG.print(gyroYStd, 1);
      DEBUG.print(F(" | F:")); DEBUG.print(frontMean);
      DEBUG.print(F(" R:")); DEBUG.print(rearMean);
      DEBUG.print(F(" | f:")); DEBUG.print(freq_curr, 1);
      DEBUG.print(F(" I:")); DEBUG.print(inten1_curr);
      DEBUG.print('/'); DEBUG.println(inten2_curr);
    }
  }

  if (nowUs - lastSend >= SEND_DT_US) {
    lastSend = nowUs;
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(frame, 15);
    Wire.endTransmission();
  }
}