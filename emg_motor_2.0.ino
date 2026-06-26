/*
  A0/A1 双通道 EMG + 编码器闭环电机安全控制版

  控制逻辑：
  A0 超过阈值并稳定确认 → 正向转动指定角度
  A1 超过阈值并稳定确认 → 反向转动指定角度

  肌电强度分级：
  轻度收缩 → 小角度
  中度收缩 → 中角度
  强度收缩 → 大角度

  手动命令：
  1 = 手动正向小角度
  2 = 手动反向小角度
  3 = 手动正向中角度
  4 = 手动反向中角度
  5 = 手动正向大角度
  6 = 手动反向大角度
  0 = 立即停止 / 急停
  e = 开启 / 关闭 EMG 自动触发
  c = 重新校准 EMG 基线
  p = 打印当前编码器位置

  电机接线：
  Arduino D6 → 电机驱动 IN1
  Arduino D5 → 电机驱动 IN2
  Arduino D9 → 电机驱动 ENA / PWM
  Arduino GND → 电机驱动 GND

  EMG接线：
  正转通道 OUT / RAW → A0
  反转通道 OUT / RAW → A1
  EMG VDD → 5V
  EMG GND → GND

  编码器接线：
  Encoder A 黄线 → D2
  Encoder B 白线 → D3
  Encoder VCC 蓝线 → 5V
  Encoder GND 绿线 → GND
*/

#define EMG_FORWARD_PIN A0
#define EMG_REVERSE_PIN A1

#define MOTOR_PIN1 6
#define MOTOR_PIN2 5
#define MOTOR_ENABLE_PIN 9

#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 3

// ================= 手动命令 =================

#define CMD_STOP '0'
#define CMD_MANUAL_FORWARD_SMALL '1'
#define CMD_MANUAL_REVERSE_SMALL '2'
#define CMD_MANUAL_FORWARD_MEDIUM '3'
#define CMD_MANUAL_REVERSE_MEDIUM '4'
#define CMD_MANUAL_FORWARD_LARGE '5'
#define CMD_MANUAL_REVERSE_LARGE '6'
#define CMD_TOGGLE_EMG 'e'
#define CMD_RECALIBRATE 'c'
#define CMD_PRINT_POSITION 'p'

// ================= 电机安全参数 =================

#define MOTOR_SPEED 225

// 编码器失效保护：即使没到目标位置，超过这个时间也必须停
#define MAX_RUN_TIME_MS 1200

// 每次动作后冷却
#define COOLDOWN_MS 5000

// 到达目标附近多少计数以内就停止，防止来回抖动
#define POSITION_TOLERANCE 3

// ================= 角度/行程参数 =================
// 注意：这里先用“编码器计数”代表动作幅度。
// 如果你知道每度对应多少编码器计数，可以换算成真实角度。
// 例如：目标角度15°，每度4个计数，则目标计数=60。

#define SMALL_STEP_COUNTS 60
#define MEDIUM_STEP_COUNTS 120
#define LARGE_STEP_COUNTS 180

// 总行程保护，防止累计转太多。
// 你需要根据实际机械结构调小或调大。
// 第一次建议保守一点，比如 ±300。
#define MIN_POSITION_COUNT -300
#define MAX_POSITION_COUNT 300

// ================= EMG 参数 =================

#define CALIBRATION_TIME_MS 3000

// A0 正向阈值
#define FORWARD_ON_THRESHOLD 170
#define FORWARD_OFF_THRESHOLD 80

// A1 反向阈值
#define REVERSE_ON_THRESHOLD 150
#define REVERSE_OFF_THRESHOLD 80

// 意图确认：必须持续超过阈值这么久才认为是有效动作
#define ACTIVE_STABLE_MS 150

// 放松确认：必须两路都低于释放阈值这么久，才允许下一次触发
#define RELEASE_STABLE_MS 500

// EMG平滑系数
#define ALPHA 0.25

#define PRINT_INTERVAL_MS 200

// ================= 可选硬件急停 / 限位 =================

#define USE_ESTOP 0
#define ESTOP_PIN 4

#define USE_LIMIT_SWITCHES 0
#define FORWARD_LIMIT_PIN 10
#define REVERSE_LIMIT_PIN 11

// ================= 状态机 =================

enum MotorState {
  IDLE,
  MOVING_FORWARD,
  MOVING_REVERSE,
  COOLDOWN,
  CONFLICT_LOCK,
  EMERGENCY_STOP
};

MotorState motorState = IDLE;

// ================= EMG变量 =================

int forwardBaseline = 0;
int reverseBaseline = 0;

float forwardFeature = 0;
float reverseFeature = 0;

unsigned long forwardActiveStartTime = 0;
unsigned long reverseActiveStartTime = 0;
unsigned long releaseStartTime = 0;
unsigned long cooldownStartTime = 0;
unsigned long motorStartTime = 0;
unsigned long lastPrintTime = 0;

bool armed = false;

// 默认关闭EMG自动触发，先手动测试方向和行程
bool emgEnabled = false;

// ================= 编码器变量 =================

volatile long encoderCount = 0;
long targetPosition = 0;

// ================= 函数声明 =================

void motorForward();
void motorReverse();
void motorStop();
void hardStop(const char* reason);

void startMoveToTarget(long deltaCounts, const char* message);
void startForwardStep(long stepCounts, const char* message);
void startReverseStep(long stepCounts, const char* message);

void calibrateEMG();
void handleSerialCommand(char input);

bool estopPressed();
bool forwardLimitHit();
bool reverseLimitHit();

long getEncoderCount();
void resetEncoderCount();
void encoderISR();

int getForwardStepFromFeature(float featureValue);
int getReverseStepFromFeature(float featureValue);

const char* stateName(MotorState state);

// ================= 初始化 =================

void setup() {
  // 上电第一时间先拉低，减少误动作风险
  digitalWrite(MOTOR_PIN1, LOW);
  digitalWrite(MOTOR_PIN2, LOW);
  digitalWrite(MOTOR_ENABLE_PIN, LOW);

  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

#if USE_ESTOP
  pinMode(ESTOP_PIN, INPUT_PULLUP);
#endif

#if USE_LIMIT_SWITCHES
  pinMode(FORWARD_LIMIT_PIN, INPUT_PULLUP);
  pinMode(REVERSE_LIMIT_PIN, INPUT_PULLUP);
#endif

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, CHANGE);

  motorStop();

  Serial.begin(115200);

  Serial.println("=================================");
  Serial.println("A0/A1 EMG + 编码器闭环电机安全控制版");
  Serial.println("请保持肌肉完全放松 3 秒，开始校准...");
  Serial.println("=================================");

  calibrateEMG();
  resetEncoderCount();
  motorStop();

  Serial.println("校准完成");
  Serial.print("A0 forwardBaseline = ");
  Serial.println(forwardBaseline);
  Serial.print("A1 reverseBaseline = ");
  Serial.println(reverseBaseline);
  Serial.println("---------------------------------");
  Serial.println("默认：EMG自动触发关闭");
  Serial.println("输入 1：手动正向小角度");
  Serial.println("输入 2：手动反向小角度");
  Serial.println("输入 3：手动正向中角度");
  Serial.println("输入 4：手动反向中角度");
  Serial.println("输入 5：手动正向大角度");
  Serial.println("输入 6：手动反向大角度");
  Serial.println("输入 0：立即停止/急停");
  Serial.println("输入 e：开启/关闭EMG自动触发");
  Serial.println("输入 c：重新校准EMG基线");
  Serial.println("输入 p：打印当前位置");
  Serial.println("---------------------------------");
}

// ================= 主循环 =================

void loop() {
  unsigned long now = millis();

  if (estopPressed()) {
    hardStop("硬件急停触发");
    return;
  }

  long currentPosition = getEncoderCount();

  // 读取EMG
  int forwardRaw = analogRead(EMG_FORWARD_PIN);
  int reverseRaw = analogRead(EMG_REVERSE_PIN);

  int forwardDiff = abs(forwardRaw - forwardBaseline);
  int reverseDiff = abs(reverseRaw - reverseBaseline);

  forwardFeature = ALPHA * forwardDiff + (1.0 - ALPHA) * forwardFeature;
  reverseFeature = ALPHA * reverseDiff + (1.0 - ALPHA) * reverseFeature;

  bool forwardActive = forwardFeature > FORWARD_ON_THRESHOLD;
  bool reverseActive = reverseFeature > REVERSE_ON_THRESHOLD;

  bool forwardReleased = forwardFeature < FORWARD_OFF_THRESHOLD;
  bool reverseReleased = reverseFeature < REVERSE_OFF_THRESHOLD;
  bool bothReleased = forwardReleased && reverseReleased;

  bool conflict = forwardActive && reverseActive;

  // 串口命令
  if (Serial.available()) {
    char input = Serial.read();
    if (input != '\n' && input != '\r') {
      handleSerialCommand(input);
    }
  }

  // 状态打印
  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;

    Serial.print("A0 raw=");
    Serial.print(forwardRaw);
    Serial.print(" F=");
    Serial.print(forwardFeature, 1);

    Serial.print(" | A1 raw=");
    Serial.print(reverseRaw);
    Serial.print(" R=");
    Serial.print(reverseFeature, 1);

    Serial.print(" | pos=");
    Serial.print(currentPosition);
    Serial.print(" target=");
    Serial.print(targetPosition);

    Serial.print(" | state=");
    Serial.print(stateName(motorState));

    Serial.print(" | armed=");
    Serial.print(armed ? "YES" : "NO");

    Serial.print(" | EMG=");
    Serial.println(emgEnabled ? "ON" : "OFF");
  }

  // 冲突保护
  if (conflict && motorState == IDLE) {
    motorStop();
    motorState = CONFLICT_LOCK;
    armed = false;
    forwardActiveStartTime = 0;
    reverseActiveStartTime = 0;
    Serial.println("保护：A0和A1同时触发，进入冲突锁定");
  }

  // 放松解锁
  if (bothReleased && motorState == IDLE) {
    if (releaseStartTime == 0) {
      releaseStartTime = now;
    }

    if (now - releaseStartTime >= RELEASE_STABLE_MS) {
      armed = true;
    }
  } else {
    releaseStartTime = 0;
  }

  // 意图确认：A0稳定触发
  if (forwardActive && !reverseActive && motorState == IDLE) {
    if (forwardActiveStartTime == 0) {
      forwardActiveStartTime = now;
    }
  } else {
    forwardActiveStartTime = 0;
  }

  // 意图确认：A1稳定触发
  if (reverseActive && !forwardActive && motorState == IDLE) {
    if (reverseActiveStartTime == 0) {
      reverseActiveStartTime = now;
    }
  } else {
    reverseActiveStartTime = 0;
  }

  bool forwardStableActive =
    forwardActiveStartTime != 0 &&
    now - forwardActiveStartTime >= ACTIVE_STABLE_MS;

  bool reverseStableActive =
    reverseActiveStartTime != 0 &&
    now - reverseActiveStartTime >= ACTIVE_STABLE_MS;

  // 状态机
  switch (motorState) {
    case IDLE:
      motorStop();

      if (emgEnabled && armed && forwardStableActive) {
        int step = getForwardStepFromFeature(forwardFeature);

        if (!forwardLimitHit()) {
          startForwardStep(step, "EMG触发：正向闭环动作");
        } else {
          hardStop("正向限位触发，禁止正向动作");
        }
      }

      else if (emgEnabled && armed && reverseStableActive) {
        int step = getReverseStepFromFeature(reverseFeature);

        if (!reverseLimitHit()) {
          startReverseStep(step, "EMG触发：反向闭环动作");
        } else {
          hardStop("反向限位触发，禁止反向动作");
        }
      }
      break;

    case MOVING_FORWARD:
      currentPosition = getEncoderCount();

      if (estopPressed() || forwardLimitHit()) {
        hardStop("正向运动中急停/限位触发");
        break;
      }

      if (currentPosition >= targetPosition - POSITION_TOLERANCE) {
        motorStop();
        cooldownStartTime = now;
        motorState = COOLDOWN;
        Serial.println("正向到达目标位置，停止，进入冷却");
      }

      else if (now - motorStartTime >= MAX_RUN_TIME_MS) {
        hardStop("正向运动超时，疑似编码器/机械异常");
      }
      break;

    case MOVING_REVERSE:
      currentPosition = getEncoderCount();

      if (estopPressed() || reverseLimitHit()) {
        hardStop("反向运动中急停/限位触发");
        break;
      }

      if (currentPosition <= targetPosition + POSITION_TOLERANCE) {
        motorStop();
        cooldownStartTime = now;
        motorState = COOLDOWN;
        Serial.println("反向到达目标位置，停止，进入冷却");
      }

      else if (now - motorStartTime >= MAX_RUN_TIME_MS) {
        hardStop("反向运动超时，疑似编码器/机械异常");
      }
      break;

    case COOLDOWN:
      motorStop();

      if (now - cooldownStartTime >= COOLDOWN_MS) {
        if (bothReleased) {
          motorState = IDLE;
          armed = false;
          releaseStartTime = now;
          Serial.println("冷却结束，等待放松解锁");
        } else {
          cooldownStartTime = now;
          Serial.println("冷却结束但EMG未完全放松，继续保护等待");
        }
      }
      break;

    case CONFLICT_LOCK:
      motorStop();

      if (bothReleased) {
        cooldownStartTime = now;
        motorState = COOLDOWN;
        Serial.println("冲突解除，进入冷却");
      }
      break;

    case EMERGENCY_STOP:
      motorStop();

      if (bothReleased) {
        motorState = IDLE;
        armed = false;
        releaseStartTime = now;
        Serial.println("急停解除，回到待机状态");
      }
      break;
  }
}

// ================= 串口命令处理 =================

void handleSerialCommand(char input) {
  Serial.print("收到指令：");
  Serial.println(input);

  if (input == CMD_STOP) {
    hardStop("串口急停");
  }

  else if (input == CMD_MANUAL_FORWARD_SMALL) {
    if (motorState == IDLE) {
      startForwardStep(SMALL_STEP_COUNTS, "手动：正向小角度");
    } else {
      Serial.println("当前不允许动作，请等待停止/冷却/解除保护");
    }
  }

  else if (input == CMD_MANUAL_REVERSE_SMALL) {
    if (motorState == IDLE) {
      startReverseStep(SMALL_STEP_COUNTS, "手动：反向小角度");
    } else {
      Serial.println("当前不允许动作，请等待停止/冷却/解除保护");
    }
  }

  else if (input == CMD_MANUAL_FORWARD_MEDIUM) {
    if (motorState == IDLE) {
      startForwardStep(MEDIUM_STEP_COUNTS, "手动：正向中角度");
    } else {
      Serial.println("当前不允许动作，请等待停止/冷却/解除保护");
    }
  }

  else if (input == CMD_MANUAL_REVERSE_MEDIUM) {
    if (motorState == IDLE) {
      startReverseStep(MEDIUM_STEP_COUNTS, "手动：反向中角度");
    } else {
      Serial.println("当前不允许动作，请等待停止/冷却/解除保护");
    }
  }

  else if (input == CMD_MANUAL_FORWARD_LARGE) {
    if (motorState == IDLE) {
      startForwardStep(LARGE_STEP_COUNTS, "手动：正向大角度");
    } else {
      Serial.println("当前不允许动作，请等待停止/冷却/解除保护");
    }
  }

  else if (input == CMD_MANUAL_REVERSE_LARGE) {
    if (motorState == IDLE) {
      startReverseStep(LARGE_STEP_COUNTS, "手动：反向大角度");
    } else {
      Serial.println("当前不允许动作，请等待停止/冷却/解除保护");
    }
  }

  else if (input == CMD_TOGGLE_EMG || input == 'E') {
    emgEnabled = !emgEnabled;
    armed = false;
    forwardActiveStartTime = 0;
    reverseActiveStartTime = 0;

    Serial.print("EMG自动触发：");
    Serial.println(emgEnabled ? "ON" : "OFF");
  }

  else if (input == CMD_RECALIBRATE || input == 'C') {
    motorStop();
    motorState = EMERGENCY_STOP;
    Serial.println("准备重新校准：请保持完全放松3秒");

    calibrateEMG();

    forwardFeature = 0;
    reverseFeature = 0;
    armed = false;
    motorState = IDLE;

    Serial.println("重新校准完成");
    Serial.print("A0 forwardBaseline = ");
    Serial.println(forwardBaseline);
    Serial.print("A1 reverseBaseline = ");
    Serial.println(reverseBaseline);
  }

  else if (input == CMD_PRINT_POSITION || input == 'P') {
    Serial.print("当前编码器位置 = ");
    Serial.println(getEncoderCount());
  }

  else {
    Serial.println("无效指令。可用：1正小，2反小，3正中，4反中，5正大，6反大，0急停，e开关EMG，c校准，p位置");
  }
}

// ================= EMG强度分级 =================

int getForwardStepFromFeature(float featureValue) {
  if (featureValue >= 300) {
    return LARGE_STEP_COUNTS;
  } else if (featureValue >= 230) {
    return MEDIUM_STEP_COUNTS;
  } else {
    return SMALL_STEP_COUNTS;
  }
}

int getReverseStepFromFeature(float featureValue) {
  if (featureValue >= 280) {
    return LARGE_STEP_COUNTS;
  } else if (featureValue >= 220) {
    return MEDIUM_STEP_COUNTS;
  } else {
    return SMALL_STEP_COUNTS;
  }
}

// ================= 启动闭环动作 =================

void startForwardStep(long stepCounts, const char* message) {
  long current = getEncoderCount();
  long proposedTarget = current + stepCounts;

  if (proposedTarget > MAX_POSITION_COUNT) {
    Serial.println("保护：正向目标超过最大行程，动作取消");
    motorStop();
    motorState = EMERGENCY_STOP;
    return;
  }

  targetPosition = proposedTarget;

  motorStop();
  delay(30);

  motorForward();
  motorStartTime = millis();
  motorState = MOVING_FORWARD;

  armed = false;
  forwardActiveStartTime = 0;
  reverseActiveStartTime = 0;
  releaseStartTime = 0;

  Serial.print(message);
  Serial.print("，目标位置=");
  Serial.println(targetPosition);
}

void startReverseStep(long stepCounts, const char* message) {
  long current = getEncoderCount();
  long proposedTarget = current - stepCounts;

  if (proposedTarget < MIN_POSITION_COUNT) {
    Serial.println("保护：反向目标超过最小行程，动作取消");
    motorStop();
    motorState = EMERGENCY_STOP;
    return;
  }

  targetPosition = proposedTarget;

  motorStop();
  delay(30);

  motorReverse();
  motorStartTime = millis();
  motorState = MOVING_REVERSE;

  armed = false;
  forwardActiveStartTime = 0;
  reverseActiveStartTime = 0;
  releaseStartTime = 0;

  Serial.print(message);
  Serial.print("，目标位置=");
  Serial.println(targetPosition);
}

// ================= 电机控制 =================

void motorForward() {
  digitalWrite(MOTOR_PIN1, HIGH);
  digitalWrite(MOTOR_PIN2, LOW);
  analogWrite(MOTOR_ENABLE_PIN, MOTOR_SPEED);
}

void motorReverse() {
  digitalWrite(MOTOR_PIN1, LOW);
  digitalWrite(MOTOR_PIN2, HIGH);
  analogWrite(MOTOR_ENABLE_PIN, MOTOR_SPEED);
}

void motorStop() {
  analogWrite(MOTOR_ENABLE_PIN, 0);
  digitalWrite(MOTOR_PIN1, LOW);
  digitalWrite(MOTOR_PIN2, LOW);
}

void hardStop(const char* reason) {
  motorStop();
  motorState = EMERGENCY_STOP;
  armed = false;
  forwardActiveStartTime = 0;
  reverseActiveStartTime = 0;
  releaseStartTime = 0;

  Serial.print("保护停机：");
  Serial.println(reason);
}

// ================= EMG 校准 =================

void calibrateEMG() {
  long forwardSum = 0;
  long reverseSum = 0;
  int count = 0;

  unsigned long startTime = millis();

  while (millis() - startTime < CALIBRATION_TIME_MS) {
    forwardSum += analogRead(EMG_FORWARD_PIN);
    reverseSum += analogRead(EMG_REVERSE_PIN);
    count++;
    delay(5);
  }

  if (count == 0) {
    count = 1;
  }

  forwardBaseline = forwardSum / count;
  reverseBaseline = reverseSum / count;

  forwardFeature = 0;
  reverseFeature = 0;
}

// ================= 编码器 =================

void encoderISR() {
  int a = digitalRead(ENCODER_A_PIN);
  int b = digitalRead(ENCODER_B_PIN);

  if (a == b) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

long getEncoderCount() {
  noInterrupts();
  long value = encoderCount;
  interrupts();
  return value;
}

void resetEncoderCount() {
  noInterrupts();
  encoderCount = 0;
  interrupts();
}

// ================= 急停 / 限位 =================

bool estopPressed() {
#if USE_ESTOP
  return digitalRead(ESTOP_PIN) == LOW;
#else
  return false;
#endif
}

bool forwardLimitHit() {
#if USE_LIMIT_SWITCHES
  return digitalRead(FORWARD_LIMIT_PIN) == LOW;
#else
  return false;
#endif
}

bool reverseLimitHit() {
#if USE_LIMIT_SWITCHES
  return digitalRead(REVERSE_LIMIT_PIN) == LOW;
#else
  return false;
#endif
}

// ================= 状态名 =================

const char* stateName(MotorState state) {
  switch (state) {
    case IDLE:
      return "IDLE";
    case MOVING_FORWARD:
      return "FORWARD";
    case MOVING_REVERSE:
      return "REVERSE";
    case COOLDOWN:
      return "COOLDOWN";
    case CONFLICT_LOCK:
      return "CONFLICT";
    case EMERGENCY_STOP:
      return "E_STOP";
    default:
      return "UNKNOWN";
  }
}