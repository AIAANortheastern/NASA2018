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

uint32_t timer = millis();



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
  
    // Set the update rate of the GPS
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

    // Interrupt goes off every 1 millisecond
    useInterrupt(true);

    // waits after initilization to start running the program
    delay(1000);

}

// Retrieves the sensor data when called
void get_sensor_data() {
    

}

// Operates the driving of the rover
void drive_rover() {
    
  
}

// Deploys the rover from the rocket
void deploy_rover() {


    // After the deployment has finished deploy the thrid wheel
    deploy_third_wheel();
}

// deploys the power flower after the 
void deploy_power_flower() {
  
    power_deploy.attach(power_deploy_pin);

    delay(15);

    power_deploy.write(179);
  
}

void deploy_thrid_wheel {
    
    third_wheel.attach(third_wheel_pin);

    delay(15);

    third_wheel.write(179);
  
}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void get_gps_values()
{
    // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every .5 seconds or so, print out the current stats
  if (millis() - timer > 500) { 
    timer = millis(); // reset the timer
    
    //Serial.print("Fix: "); Serial.print((int)GPS.fix);
    //Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    
    if (GPS.fix) {
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
    }else{
      Serial.print("No Fix");
    }
  }
}


void loop() {


}
