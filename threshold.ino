#define EMG_PIN A1

#define CALIBRATION_TIME_MS 3000
#define NOISE_TEST_TIME_MS 2000
#define PRINT_INTERVAL_MS 100

// 平滑系数：越大越灵敏，越小越稳定
#define ALPHA 0.25

int baseline = 0;       // 放松基线
int relaxNoise = 0;     // 放松状态下最大噪声
float feature = 0;      // 平滑后的肌电特征值
int peak = 0;           // 用力后的峰值

unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(115200);

  Serial.println("=================================");
  Serial.println("A0 单通道 EMG 阈值测试程序启动");
  Serial.println("请保持肌肉完全放松 3 秒，开始自动校准...");
  Serial.println("=================================");

  calibrateEMG();

  Serial.println();
  Serial.println("校准完成！");
  Serial.print("A0 baseline = ");
  Serial.println(baseline);

  Serial.print("A0 放松噪声峰值 relaxNoise = ");
  Serial.println(relaxNoise);

  Serial.println("---------------------------------");
  Serial.println("现在开始测试：");
  Serial.println("1. 保持放松，观察 feature 范围");
  Serial.println("2. 用力收缩，观察 peak 能到多少");
  Serial.println("3. 串口输入 r 可以重置 peak");
  Serial.println("---------------------------------");
}

void loop() {
  unsigned long now = millis();

  int raw = analogRead(EMG_PIN);
  int diff = abs(raw - baseline);

  // 平滑后的特征值
  feature = ALPHA * diff + (1.0 - ALPHA) * feature;

  if (feature > peak) {
    peak = feature;
  }

  // 串口输入 r，重置峰值
  if (Serial.available()) {
    char input = Serial.read();

    if (input == 'r' || input == 'R') {
      peak = 0;
      Serial.println("peak 已重置，请重新用力测试");
    }
  }

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;

    int suggestedOn = calculateOnThreshold(relaxNoise, peak);
    int suggestedOff = calculateOffThreshold(relaxNoise, suggestedOn);

    Serial.print("A0 raw=");
    Serial.print(raw);

    Serial.print(" | baseline=");
    Serial.print(baseline);

    Serial.print(" | feature=");
    Serial.print(feature, 1);

    Serial.print(" | peak=");
    Serial.print(peak);

    Serial.print(" | 建议ON=");
    Serial.print(suggestedOn);

    Serial.print(" | 建议OFF=");
    Serial.println(suggestedOff);
  }
}

void calibrateEMG() {
  long sum = 0;
  int count = 0;

  unsigned long startTime = millis();

  // 第一轮：计算放松基线
  while (millis() - startTime < CALIBRATION_TIME_MS) {
    sum += analogRead(EMG_PIN);
    count++;
    delay(5);
  }

  if (count == 0) {
    count = 1;
  }

  baseline = sum / count;

  Serial.println("基线已记录，请继续保持放松 2 秒，测量噪声范围...");

  // 第二轮：计算放松噪声峰值
  startTime = millis();

  while (millis() - startTime < NOISE_TEST_TIME_MS) {
    int raw = analogRead(EMG_PIN);
    int diff = abs(raw - baseline);

    if (diff > relaxNoise) {
      relaxNoise = diff;
    }

    delay(5);
  }

  feature = 0;
  peak = 0;
}

int calculateOnThreshold(int noiseValue, int peakValue) {
  // 如果还没有明显用力，先给一个默认建议
  if (peakValue < noiseValue + 20) {
    return noiseValue + 30;
  }

  // 推荐触发阈值：放松噪声和用力峰值之间
  int threshold = noiseValue + (peakValue - noiseValue) * 0.45;

  // 防止太敏感
  if (threshold < noiseValue + 15) {
    threshold = noiseValue + 15;
  }

  return threshold;
}

int calculateOffThreshold(int noiseValue, int onThreshold) {
  // 释放阈值低于触发阈值，用来防抖
  int threshold = noiseValue + (onThreshold - noiseValue) * 0.45;

  if (threshold < noiseValue + 5) {
    threshold = noiseValue + 5;
  }

  return threshold;
}