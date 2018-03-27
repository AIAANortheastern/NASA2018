#include <Wire.h>
#include "I2Cdev.h"
#include "Wire.h"
#include "SoftwareSerial.h"

int16_t ax, ay, az;
#define OUTPUT_READABLE_ACCELGYRO

#include <SoftwareSerial.h>

// Starting the accelerometer protocol
//MMA_7455 accel = MMA_7455(i2c_protocol);

// Defining the pins for the xbee radio
const int RXPin = A0;
const int TXPin = A1;

char str;

SoftwareSerial Xbee(RXPin, TXPin);

void setup() {
  //Open Serial connection
  Serial.begin(9600);
  //Start xbee serial connection
  Xbee.begin(9600);
  // Accelerometer initilization  
  //accel.begin();

  // Set Relay pin
  pinMode(10, OUTPUT);

}

void loop() {

if (Serial.available())
  { // If data comes in from serial monitor, send it out to XBee
    Xbee.write(Serial.read());
  }
  if (Xbee.available())
  { // If data comes in from XBee, send it out to serial monitor
    // Serial.write(Xbee.read());
    str = Xbee.read();
    Serial.println(str);
    if (str == 'r'){
      digitalWrite(10, HIGH);
      Serial.println("Relay Activated");
    }
    if (str == 'd'){
      digitalWrite(10, LOW);
      Serial.println("Relay Deactivated");
    }
  }
}
