#include <SoftwareSerial.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

const int RXPin = A0;
const int TXPin = A1;

char str;

SoftwareSerial Xbee(RXPin, TXPin);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.begin(9600);
  //Start xbee serial connection
  Xbee.begin(9600);
  // Accelerometer initilization  
  //accel.begin();
}

void loop() {
  mpu6050.update();
  Serial.print("angleX : ");
  Serial.print(mpu6050.getAngleX());
  Serial.print("\tangleY : ");
  Serial.print(mpu6050.getAngleY());
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  Xbee.write(mpu6050.getAngleZ());
  delay(100);
}
