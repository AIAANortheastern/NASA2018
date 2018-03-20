// Libraries
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

//Starts the acceleratomoter and gyro device
MPU6050 accelgyro;
int16_t ax, ay, az;
#define OUTPUT_READABLE_ACCELGYRO

// Gps and Xbee serial pin definitions
const int gps_rx = A1;
const int gps_tx = A2;
const int xbee_rx = A3;
const int xbee_tx = A4;


const int third_wheel_pin = 1;
const int power_deploy_pin = 2;


Servo third_wheel;
Servo power_deploy;


SoftwareSerial gps_serial(gps_rx, gps_tx);
SoftwareSerial xbee_serial(xbee_rx, xbee_tx);

Adafruit_GPS GPS(&gps_serial);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
#define GPSECHO false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean);











void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
