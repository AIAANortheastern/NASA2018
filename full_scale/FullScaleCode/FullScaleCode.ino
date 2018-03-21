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

//Number of samples for the power flowerometer
#define NUM_SAMPLES 10

// Gps and Xbee serial pin definitions
const int gps_rx = A1;
const int gps_tx = A2;
const int xbee_rx = A3;
const int xbee_tx = A4;

// Pin assignments for the two servos
// Make sure these are PWM capable pins when updated
const int third_wheel_pin = 1;
const int power_deploy_pin = 2;

// Pin assignment for the motor control pin
const int motor_pin_one = 10;

// Create the servo objects
Servo third_wheel;
Servo power_deploy;

// Cerate the serial objects for the gps and the xbee radio
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

    // starts the wire object
    Wire.begin();

    // starts the xbee radio object on the 9600 baud rate
    xbee_serial.begin(9600);

    // intilizes the accelerometer gyro object
    accelgyro.initialize();

    // sets the motor control pin to output mode
    pinMode(motor_pin_one, OUTPUT);

    // attaches the third wheel servo to the correct pin
    third_wheel.attach(third_wheel_pin);

    // attaches the power flower servo to the correct pin
    power_deploy.attach(power_deploy_pin);

    // starts the GPS serial object at the 9600 baud rate
    GPS.begin(9600);
  
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

    // Interrupt goes off every 1 millisecond
    useInterrupt(true);

    delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:

}
