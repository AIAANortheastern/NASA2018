#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "SoftwareSerial.h"
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_ACCELGYRO
#define LED_PIN 13
bool blinkState = false;

const int RXPin = A2;
const int TXPin = A3;

  SoftwareSerial Xbee(RXPin, TXPin);

 
void setup() {

        Wire.begin();
    Xbee.begin(9600);
    accelgyro.initialize();
    pinMode(LED_PIN, OUTPUT);
}
 
void loop() {
 
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
          int start=millis();Serial.print(start);Serial.print("\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
 
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
