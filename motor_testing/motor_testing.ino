// 电机控制引脚
int motorPin1 = 4;   // IN1 (电机A方向)
int motorPin2 = 5;   // IN2 (电机A方向)
int speedPinA = 6;   // ENA (PWM控制电机A的速度)

// 编码器信号引脚
int pinA = 2;  // HALL SENSOR A Vout 接 D2
int pinB = 3;  // HALL SENSOR B Vout 接 D3

// 控制电机转动的变量
int motorSpeed = 255;  // 电机速度，范围从0到255（0为停转，255为最大速度）

void setup() {
  Serial.begin(9600);     // 初始化串口通信
  pinMode(motorPin1, OUTPUT);  // 设置电机方向控制为输出
  pinMode(motorPin2, OUTPUT);  // 设置电机方向控制为输出
  pinMode(speedPinA, OUTPUT);  // 设置PWM控制电机速度为输出
  
  Serial.println("电机控制程序启动，请输入 'r' 正转 或 'f' 反转 控制电机。");
}

void loop() {
  if (Serial.available()) {
    char input = Serial.read();  // 获取串口输入

    if (input == 'r') {
      // 正转
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
      analogWrite(speedPinA, motorSpeed);  // 设置速度
      Serial.println("电机正转");
    }
    else if (input == 'f') {
      // 反转
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
      analogWrite(speedPinA, motorSpeed);  // 设置速度
      Serial.println("电机反转");
    }
    else {
      Serial.println("无效输入，请输入 'r' 正转 或 'f' 反转。");
    }
  }
}

