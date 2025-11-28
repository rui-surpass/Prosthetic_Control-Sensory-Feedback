
#include <Wire.h>

void setup() {
  Wire.begin();  
  Serial.begin(115200);
  delay(3000);
}
unsigned char  tx1_data[15] = {0xFA,0x01,0x61,0x94,0x00,0x14,0x61,0x94,0x00,0x14,0x01,0x01,0x1E,0x1E,0xAF};

unsigned char Val1_1 = 20;
unsigned char Val1_2 = 40;

void loop() {
      //开启通道1，关闭通道2
      tx1_data[12] = Val1_1;
      tx1_data[13] = Val1_2;
      tx1_data[10] = 1;
      tx1_data[11] = 0;
      Wire.beginTransmission(0x01);     // 向地址为8的从机传送数据
      Wire.write(tx1_data,15); 
      Wire.endTransmission();       // 结束传送
      Serial.print("M1_Ch1 20:");
      delay(5000);
      //关闭通道1，开启通道2
      tx1_data[12] = Val1_1;
      tx1_data[13] = Val1_2;
      tx1_data[10] = 0;
      tx1_data[11] = 1;
      Wire.beginTransmission(0x01);     // 向地址为8的从机传送数据
      Wire.write(tx1_data,15); 
      Wire.endTransmission();       // 结束传送
      Serial.println("M1_Ch2 40:");
      delay(5000);
      //关闭通道1和通道2
      tx1_data[12] = Val1_1;
      tx1_data[13] = Val1_2;
      tx1_data[10] = 0;
      tx1_data[11] = 0;
      Wire.beginTransmission(0x01);     // 向地址为8的从机传送数据
      Wire.write(tx1_data,15); 
      Wire.endTransmission();       // 结束传送
      Serial.println("M1_OFF");
      //开启通道1和通道2
      tx1_data[12] = Val1_1;
      tx1_data[13] = Val1_2;
      tx1_data[10] = 1;
      tx1_data[11] = 1;
      Wire.beginTransmission(0x01);     // 向地址为8的从机传送数据
      Wire.write(tx1_data,15); 
      Wire.endTransmission();       // 结束传送
      Serial.println("M1_ON");
      //关闭通道1和通道2
      tx1_data[12] = Val1_1;
      tx1_data[13] = Val1_2;
      tx1_data[10] = 0;
      tx1_data[11] = 0;
      Wire.beginTransmission(0x01);     // 向地址为8的从机传送数据
      Wire.write(tx1_data,15); 
      Wire.endTransmission();       // 结束传送
      Serial.println("M1_OFF");

      
}