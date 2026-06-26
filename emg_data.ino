/*
  Four-channel Driving Pedal EMG Acquisition

  A0: anterior lower leg main channel      -> Release / dorsiflexion
  A1: anterior lower leg auxiliary channel -> Release auxiliary
  A2: posterior lower leg main channel     -> Press / plantarflexion
  A3: posterior lower leg auxiliary channel-> Press auxiliary

  Experimental cycle:
  NEUTRAL -> PRESS -> HOLD -> RELEASE -> REST

  Output CSV:
  time_ms,label,
  A0_raw,A1_raw,A2_raw,A3_raw,
  A0_feature,A1_feature,A2_feature,A3_feature,
  A0_baseline,A1_baseline,A2_baseline,A3_baseline

  Commands:
  n = NEUTRAL
  p = PRESS
  h = HOLD
  u = RELEASE
  r = REST
  c = RECALIBRATE
*/

#define EMG_A0_PIN A0   // Release main
#define EMG_A1_PIN A1   // Release auxiliary
#define EMG_A2_PIN A2   // Press main
#define EMG_A3_PIN A3   // Press auxiliary

// Motor pins are forced OFF during data acquisition
#define MOTOR_PIN1 6
#define MOTOR_PIN2 5
#define MOTOR_ENABLE_PIN 9

#define CALIBRATION_TIME_MS 3000
#define SAMPLE_INTERVAL_MS 20   // 50 Hz
#define ALPHA 0.25              // smoothing factor

int baseA0 = 0;
int baseA1 = 0;
int baseA2 = 0;
int baseA3 = 0;

float featA0 = 0;
float featA1 = 0;
float featA2 = 0;
float featA3 = 0;

unsigned long lastSampleTime = 0;

const char* currentLabel = "REST";

void setup() {
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  motorStop();

  Serial.begin(115200);
  delay(800);

  Serial.println("# Four-channel driving pedal EMG acquisition started");
  Serial.println("# Keep leg relaxed for 3 seconds. Calibrating baseline...");
  calibrateBaseline();

  Serial.println("# Calibration finished");
  Serial.print("# A0_baseline=");
  Serial.println(baseA0);
  Serial.print("# A1_baseline=");
  Serial.println(baseA1);
  Serial.print("# A2_baseline=");
  Serial.println(baseA2);
  Serial.print("# A3_baseline=");
  Serial.println(baseA3);

  Serial.println("# Commands: n=NEUTRAL, p=PRESS, h=HOLD, u=RELEASE, r=REST, c=CALIBRATE");
  Serial.println("time_ms,label,A0_raw,A1_raw,A2_raw,A3_raw,A0_feature,A1_feature,A2_feature,A3_feature,A0_baseline,A1_baseline,A2_baseline,A3_baseline");
}

void loop() {
  motorStop();      // Always keep motor disabled during acquisition
  handleCommand();

  unsigned long now = millis();

  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;

    int rawA0 = analogRead(EMG_A0_PIN);
    int rawA1 = analogRead(EMG_A1_PIN);
    int rawA2 = analogRead(EMG_A2_PIN);
    int rawA3 = analogRead(EMG_A3_PIN);

    int diffA0 = abs(rawA0 - baseA0);
    int diffA1 = abs(rawA1 - baseA1);
    int diffA2 = abs(rawA2 - baseA2);
    int diffA3 = abs(rawA3 - baseA3);

    featA0 = ALPHA * diffA0 + (1.0 - ALPHA) * featA0;
    featA1 = ALPHA * diffA1 + (1.0 - ALPHA) * featA1;
    featA2 = ALPHA * diffA2 + (1.0 - ALPHA) * featA2;
    featA3 = ALPHA * diffA3 + (1.0 - ALPHA) * featA3;

    Serial.print(now);
    Serial.print(",");
    Serial.print(currentLabel);
    Serial.print(",");

    Serial.print(rawA0);
    Serial.print(",");
    Serial.print(rawA1);
    Serial.print(",");
    Serial.print(rawA2);
    Serial.print(",");
    Serial.print(rawA3);
    Serial.print(",");

    Serial.print(featA0, 1);
    Serial.print(",");
    Serial.print(featA1, 1);
    Serial.print(",");
    Serial.print(featA2, 1);
    Serial.print(",");
    Serial.print(featA3, 1);
    Serial.print(",");

    Serial.print(baseA0);
    Serial.print(",");
    Serial.print(baseA1);
    Serial.print(",");
    Serial.print(baseA2);
    Serial.print(",");
    Serial.println(baseA3);
  }
}

void handleCommand() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'n' || cmd == 'N') {
      currentLabel = "NEUTRAL";
      Serial.println("# LABEL=NEUTRAL");
    }
    else if (cmd == 'p' || cmd == 'P') {
      currentLabel = "PRESS";
      Serial.println("# LABEL=PRESS");
    }
    else if (cmd == 'h' || cmd == 'H') {
      currentLabel = "HOLD";
      Serial.println("# LABEL=HOLD");
    }
    else if (cmd == 'u' || cmd == 'U') {
      currentLabel = "RELEASE";
      Serial.println("# LABEL=RELEASE");
    }
    else if (cmd == 'r' || cmd == 'R') {
      currentLabel = "REST";
      Serial.println("# LABEL=REST");
    }
    else if (cmd == 'c' || cmd == 'C') {
      motorStop();
      Serial.println("# Recalibrating: keep leg relaxed for 3 seconds...");
      calibrateBaseline();

      featA0 = 0;
      featA1 = 0;
      featA2 = 0;
      featA3 = 0;

      currentLabel = "REST";

      Serial.println("# Recalibration finished");
      Serial.print("# A0_baseline=");
      Serial.println(baseA0);
      Serial.print("# A1_baseline=");
      Serial.println(baseA1);
      Serial.print("# A2_baseline=");
      Serial.println(baseA2);
      Serial.print("# A3_baseline=");
      Serial.println(baseA3);

      Serial.println("time_ms,label,A0_raw,A1_raw,A2_raw,A3_raw,A0_feature,A1_feature,A2_feature,A3_feature,A0_baseline,A1_baseline,A2_baseline,A3_baseline");
    }
  }
}

void calibrateBaseline() {
  long sumA0 = 0;
  long sumA1 = 0;
  long sumA2 = 0;
  long sumA3 = 0;

  int count = 0;

  unsigned long startTime = millis();

  while (millis() - startTime < CALIBRATION_TIME_MS) {
    motorStop();

    sumA0 += analogRead(EMG_A0_PIN);
    sumA1 += analogRead(EMG_A1_PIN);
    sumA2 += analogRead(EMG_A2_PIN);
    sumA3 += analogRead(EMG_A3_PIN);

    count++;
    delay(5);
  }

  if (count == 0) {
    count = 1;
  }

  baseA0 = sumA0 / count;
  baseA1 = sumA1 / count;
  baseA2 = sumA2 / count;
  baseA3 = sumA3 / count;

  featA0 = 0;
  featA1 = 0;
  featA2 = 0;
  featA3 = 0;
}

void motorStop() {
  analogWrite(MOTOR_ENABLE_PIN, 0);
  digitalWrite(MOTOR_PIN1, LOW);
  digitalWrite(MOTOR_PIN2, LOW);
}