#include <Arduino.h>
#include <Wire.h>

#define DEBUG Serial

// ======== 硬件定义 ========
const uint8_t I2C_ADDR = 0x01;
const int SENSOR1_PIN = A2;  // 前
const int SENSOR2_PIN = A3;  //后脚

// ======== 传感器参数 ========
#define VOLTAGE_MIN 150
#define VOLTAGE_MAX 3300
#define PRESS_MIN   500
#define PRESS_MAX   20000

// ======== 感知-刺激映射参数 ========
const float P0_g = 200.0f;   // 触发起点
const float P1_g = 3000.0f;  // 满踩上限
const float GAMMA = 0.5f;    // 幂律压缩：弱刺激更敏感

// ======== 电刺激安全参数 ========
const float FREQ_MIN = 5.0f;
const float FREQ_MAX = 25.0f;
const uint16_t PULSE_US = 200;
const uint8_t INTENS_MIN = 0;
const uint8_t INTENS_MAX = 20;
const uint8_t INTENS_STEP_MAX = 2;
const float   FREQ_STEP_MAX   = 2.0f;

// ======== 时序参数 ========
const uint32_t SEND_HZ = 100;
const uint32_t SEND_DT_US = 1000000UL / SEND_HZ;
const uint32_t SENSE_DT_MS = 100;
const float HYST_g = 50.0f;
const uint16_t ARM_PRESS_MS   = 200;
const uint16_t NO_PRESS_OFF_MS= 150;

// ======== 数据帧 ========
uint8_t frame[15] = {
  0xFA, 0x01,
  0x05, 0xC8, 0x00, 0x14,
  0x05, 0xC8, 0x00, 0x14,
  0x00, 0x00,
  0x00, 0x00,
  0xAF
};

// ======== 辅助函数 ========
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
  if (n < 0.02) n = 0;
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

// === 平滑读取（去抖、均值滤波） ===
int readSmooth(int pin) {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100);
  }
  return sum / 10;
}

// ======== 状态定义 ========
enum State { DISARMED, ARMED_IDLE, ARMED_OUTPUT };
State state = DISARMED;

float freq_curr = 10.0f, freq_target = 10.0f;
uint8_t inten1_curr = 0, inten2_curr = 0;
uint8_t inten1_target = 0, inten2_target = 0;

// ======== 初始化 ========
void setup() {
  DEBUG.begin(115200);
  Wire.begin();
  delay(800);
  DEBUG.println(F("✅ 系统启动：输入 a 解锁，输入 0 急停。"));
}

// ======== 主循环 ========
void loop() {
  static uint32_t lastSenseMs = 0, lastSendUs = 0;
  static uint32_t pressTimer = 0, noPressTimer = 0;

  uint32_t nowMs = millis();

  // --- 串口命令识别 ---
  if (DEBUG.available()) {
    String cmd = DEBUG.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("a")) {
      if (state == DISARMED) {
        state = ARMED_IDLE;
        DEBUG.println("✔ 已解锁(ARMED_IDLE)");
      } else DEBUG.println("⚠ 已经解锁");
    } else if (cmd == "0") {
      state = DISARMED;
      inten1_curr = inten2_curr = 0;
      frame[10] = frame[11] = 0;
      frame[12] = frame[13] = 0;
      Wire.beginTransmission(I2C_ADDR);
      Wire.write(frame, 15);
      Wire.endTransmission();
      DEBUG.println("⛔ 急停：输出关闭。");
    } else if (cmd.length() > 0) {
      DEBUG.print("❓ 未识别命令：");
      DEBUG.println(cmd);
    }
  }

  // --- 周期采样 ---
  if (nowMs - lastSenseMs >= SENSE_DT_MS) {
    lastSenseMs = nowMs;

    int mv1 = mvFromADC(readSmooth(SENSOR1_PIN));
    int mv2 = mvFromADC(readSmooth(SENSOR2_PIN));
    long g1 = gFromMilliVolt(mv1);
    long g2 = gFromMilliVolt(mv2);
    float n1 = compressNorm(g1);
    float n2 = compressNorm(g2);

    // === 状态机 ===
    switch (state) {
      case DISARMED:
        break;

      case ARMED_IDLE:
        if (g1 > P0_g + HYST_g || g2 > P0_g + HYST_g) {
          pressTimer += SENSE_DT_MS;
          if (pressTimer >= ARM_PRESS_MS) {
            state = ARMED_OUTPUT;
            DEBUG.println("▶ 检测到足底压力，开始输出(ARMED_OUTPUT)");
          }
        } else pressTimer = 0;
        break;

      case ARMED_OUTPUT:
        if (g1 < P0_g - HYST_g && g2 < P0_g - HYST_g) {
          noPressTimer += SENSE_DT_MS;
          if (noPressTimer >= NO_PRESS_OFF_MS) {
            state = ARMED_IDLE;
            inten1_target = inten2_target = 0;
            DEBUG.println("⛔ 松脚，停止输出。");
          }
        } else noPressTimer = 0;
        break;
    }

    // === 映射刺激参数 ===
    if (state == ARMED_OUTPUT) {
      inten1_target = (uint8_t)(INTENS_MIN + n1 * (INTENS_MAX - INTENS_MIN));
      inten2_target = (uint8_t)(INTENS_MIN + n2 * (INTENS_MAX - INTENS_MIN));
      float maxN = max(n1, n2);
      freq_target = FREQ_MIN + maxN * (FREQ_MAX - FREQ_MIN);
    } else {
      inten1_target = inten2_target = 0;
      freq_target = 10.0f;
    }

    inten1_curr = slewByte(inten1_curr, inten1_target, INTENS_STEP_MAX);
    inten2_curr = slewByte(inten2_curr, inten2_target, INTENS_STEP_MAX);
    freq_curr = slewFloat(freq_curr, freq_target, FREQ_STEP_MAX);
    setWaveformByFreq(freq_curr);

    frame[10] = (inten1_curr > 0) ? 1 : 0;
    frame[11] = (inten2_curr > 0) ? 1 : 0;
    frame[12] = inten1_curr;
    frame[13] = inten2_curr;

    DEBUG.print("state=");
    DEBUG.print(state == DISARMED ? "DISARMED" :
                state == ARMED_IDLE ? "ARMED_IDLE" : "ARMED_OUTPUT");
    DEBUG.print(" | g1="); DEBUG.print
    (g1);
    DEBUG.print(" g, g2="); DEBUG.print(g2);
    DEBUG.print(" g | f="); DEBUG.print(freq_curr, 1);
    DEBUG.print(" Hz | I1="); DEBUG.print(inten1_curr);
    DEBUG.print(" I2="); DEBUG.println(inten2_curr);
  }

  // --- 周期发送帧 ---
  uint32_t nowUs = micros();
  if (nowUs - lastSendUs >= SEND_DT_US) {
    lastSendUs = nowUs;
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(frame, 15);
    Wire.endTransmission();
  }
}