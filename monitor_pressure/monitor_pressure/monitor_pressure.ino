#include <Arduino.h>
#include <Wire.h>

// ======== 刺激参数（安全起步） ========
const uint8_t I2C_ADDR = 0x01;  // TENS 从机地址
float  FREQ_HZ = 10.0f;          // 默认频率：10 Hz
float  PW_US   = 400.0f;         // 单边脉宽 400 us（已加宽）
uint8_t I_LEVEL = 1;             // 默认强度等级 1
const bool CH1_ON = true;        // 通道 1 开启
const bool CH2_ON = false;       // 通道 2 关闭

// ======== 采样配置 ========
const uint8_t PIN_P = A0;        // N+ 节点（经分压/限流）→ A0
const uint8_t PIN_N = A3;        // N- 节点（经分压/限流）→ A3
const uint16_t PRINT_HZ = 1000;  // 串口输出速率（点/秒）


// ======== 数据帧发送函数（发送刺激参数） ========
static void sendParams(uint8_t I) {
  uint32_t P = (uint32_t)(100000.0f / FREQ_HZ + 0.5f);  // 周期总步数(10us)
  uint32_t B = (uint32_t)(PW_US / 10.0f + 0.5f);      // 正脉宽步数
  if (B < 1) B = 1;
  uint32_t D = B;                                       // 负脉宽=正脉宽
  if (P <= (B + D + 2)) P = (B + D + 2);                // 预留最小间隔
  uint32_t A = (P - B - D) / 2;
  uint32_t C = P - B - D - A;

  auto H=[](uint32_t x){ return (uint8_t)(x >> 8); };
  auto L=[](uint32_t x){ return (uint8_t)(x & 0xFF); };

  uint8_t tx[15] = {
    0xFA, 0x01,
    H(A), L(A),
    H(B), L(B),
    H(C), L(C),
    H(D), L(D),
    CH1_ON?1:0, CH2_ON?1:0,
    I, CH2_ON?I:0,
    0xAF
  };

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(tx, 15);
  Wire.endTransmission();
}

// ======== 读取模拟电压函数（用于压力传感器） ========
static float readMilliVoltOnce(uint8_t pin) {
  (void)analogRead(pin);                 // 丢首读
  int r = analogRead(pin);               // 0..1023
  return r * (5000.0f / 1023.0f);        // 默认基准 5V
}

void setup() {
  Serial.begin(115200);
  analogReference(DEFAULT);              // 0..5V
  pinMode(PIN_P, INPUT);
  pinMode(PIN_N, INPUT);

  Wire.begin();                          // A4=SDA, A5=SCL
  Wire.setClock(100000);

  delay(200);
  sendParams(I_LEVEL);                   // 先下发一次
  Serial.println("mV_A0,mV_A3,mV_diff");
}

void loop() {
  // 1) 周期性重发参数，防止从机掉线/超时
  static unsigned long lastTx = 0;
  if (millis() - lastTx >= 300) {
    lastTx = millis();
    sendParams(I_LEVEL);
  }

  // 2) 固定速率输出一行 CSV（不平均）
  static unsigned long lastPrint = 0;
  const unsigned long periodMs = 1000UL / PRINT_HZ;
  if (millis() - lastPrint >= periodMs) {
    lastPrint += periodMs;

    float vP = readMilliVoltOnce(PIN_P);   // A0 毫伏
    float vN = readMilliVoltOnce(PIN_N);   // A3 毫伏
    float vD = vP - vN;                    // 差分（缩放后的双向方波）

    Serial.print(vP,1); Serial.print(',');
    Serial.print(vN,1); Serial.print(',');
    Serial.println(vD,1);
  }
}

