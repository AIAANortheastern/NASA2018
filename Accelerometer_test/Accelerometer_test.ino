#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "SoftwareSerial.h"
MPU6050 accelgyro;
int16_t ax, ay, az;
#define OUTPUT_READABLE_ACCELGYRO

const int RXPin = A2;
const int TXPin = A3;

SoftwareSerial Xbee(RXPin, TXPin);

 
void setup() {

    Wire.begin();
    Serial.begin(9600);
    accelgyro.initialize();
}
 
void loop() {
 
        accel_X_Y_Z();

        delay(100);
}

void accel_X_Y_Z(){
        accelgyro.getAcceleration(&ax, &ay, &az);
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.println("\t");
}

