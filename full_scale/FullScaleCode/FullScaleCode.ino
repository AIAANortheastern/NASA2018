// Code complied by Tyler Kosakowski
// Contributers Karl Swanson, Gavin Chardler
// NASA Student Launch 2018 Northeastern University

// Libraries
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

// STR for testing XBEE
char str;

//Starts the acceleratomoter and gyro device
MPU6050 accelgyro;
int16_t ax, ay, az;
#define OUTPUT_READABLE_ACCELGYRO

//Number of samples for the power flowerometer
#define NUM_SAMPLES 10

// Power Flower variables
int sum = 0;                    // sum of samples taken
unsigned char sample_count = 0; // current sample number
float voltage = 0.0;            // calculated voltage

// Gps and Xbee serial pin definitions
const int gps_rx = 2;
const int gps_tx = 3;
const int xbee_rx = A0;
const int xbee_tx = A1;

// Pin assignments for the two servos
// Make sure these are PWM capable pins when updated
const int third_wheel_pin = 1;
const int power_deploy_pin = 2;

// Pin assignment for the motor control pin
const int motor_pin_one = 10;
// If needed a second motor control pin
//const int motor_pin_two = 11;

// PES Deploy Pin
const int pes_pin = 4;

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

boolean launch_mode = true;
boolean rover_mode = false;
boolean standby_mode = false;

void setup() {

    // start Serial Connection
    Serial.begin(4800);

    // starts the wire object
    Wire.begin();

    // starts the xbee radio object on the 9600 baud rate
    xbee_serial.begin(9600);

    // intilizes the accelerometer gyro object
    accelgyro.initialize();

    // sets the motor control pin to output mode
    pinMode(motor_pin_one, OUTPUT);

    // sets the ejection system pin to output mode
    pinMode(pes_pin, OUTPUT);

    // attaches the third wheel servo to the correct pin
    third_wheel.attach(third_wheel_pin);

    // attaches the power flower servo to the correct pin
    power_deploy.attach(power_deploy_pin);

    // starts the GPS serial object at the 9600 baud rate
    //GPS.begin(9600);
  
    // Set the update rate of the GPS
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

    // Interrupt goes off every 1 millisecond
    useInterrupt(true);

    // waits after initilization to start running the program
    delay(1000);

}

void loop() {

    // Sends a single packet
    transmit_packet();

    // Reads the input from the xbee
    str = xbee_serial.read();

    // retransmits this string
    Serial.println(str);

    // Conditionals for different activation states

     // r begins the roving process
    if (str == 'r'){
      // Uncomment these for testing
      // digitalWrite(motor_pin_one, HIGH);
      // Serial.println("Relay Activated");

      // switches into rover mode
      launch_mode = false;
      rover_mode = true;

      // sends a signal to confirm rover deployment begun

      xbee_serial.println("Deploy Start");
      
      // calls the drive rover function
      drive_rover();
    }
    if (str == 'd'){
      digitalWrite(motor_pin_one, LOW);
      Serial.println("Relay Deactivated");
    }
  delay(300);
}

// Retrieves the sensor data when called
void get_sensor_data() {
    // Retrieves the acceleration values
    accelgyro.getAcceleration(&ax, &ay, &az);

    // Finds the voltage across the solar panels
    // only operates when in rover or standby modes

    if (rover_mode || standby_mode){
        voltage = 0;
        // take a number of analog samples and add them up
        while (sample_count < NUM_SAMPLES) {
           sum += analogRead(A2);
           sample_count++;
            delay(10);
        }
        // calculate the voltage
        // use 5.0 for a 5.0V ADC reference voltage
        voltage = ((float)sum / (float)NUM_SAMPLES * 5.015) / 1024.0;
        // voltage multiplied by 11 when using voltage divider that
        // divides by 11. 11.132 is the calibrated voltage divide
        // value
        voltage = (voltage * 11);
        sample_count = 0;
        sum = 0;
    }
}

//Transmitts a packet when called
void transmit_packet() {
  // get accel values
  accelgyro.getAcceleration(&ax, &ay, &az);
  
  if (xbee_serial.available()){
    // Acceleration data
    xbee_serial.print("|X: ");
    xbee_serial.print(ax);
    xbee_serial.print("|Y: ");
    xbee_serial.print(ay);
    xbee_serial.print("|Z: ");
    xbee_serial.println(az);

    if(launch_mode || standby_mode){
        // Transmits the current gps values
        get_gps_values();
    }
  }
}
    

// Operates the driving of the rover
void drive_rover() {

    // Calls the deployment function
    deploy_rover();

    // turns on the motors by activating the relay
    digitalWrite(motor_pin_one, HIGH);
    
    xbee_serial.println("Roving begun !");

    // this will need to be experimentaly dervied
    delay(30000);

    // After the roving turn the motors off by setting the pin low
    digitalWrite(motor_pin_one, LOW);

    xbee_serial.println("Roving completed!");

    //get_sensor_data();
    delay(100);
    
    // Deploys the power flower
    deploy_power_flower();
    delay(500);
    
  
}

// Deploys the rover from the rocket
void deploy_rover() {

    xbee_serial.println("Deployment Begun");

    digitalWrite(pes_pin, HIGH);

    // These delay values will need to be dervied from testing
    delay(1000);
    transmit_packet();
    delay(1000);
    transmit_packet();
    
    // After the deployment has finished deploy the thrid wheel
    deploy_third_wheel();

    xbee_serial.println("Deployment Complete !");
}

// deploys the power flower after the rover has moved the correct distance
void deploy_power_flower() {
    // Holds the temporary position of the power flower
    int pos_power = 0;
  
    for (pos_power = 0; pos_power <= 270; pos_power += 1) { // goes from 0 degrees to 270 degrees
      // in steps of ~1 degree
      power_deploy.write(pos_power);              // tell servo to go to position in variable 'pos_power'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  
}

// Third Wheel deployment code
// I don't know why it works but it does so don't touch it
void deploy_third_wheel() {

    int pos_third = 0;
    
    third_wheel.attach(9);

    third_wheel.write(90);
  
    for (pos_third = 0; pos_third <= 30; pos_third += 1) {
      // in steps of 1 degree
      third_wheel.write(pos_third);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }

    delay (2000);
    
  
}

// The stuff below is a bunch of GPS control functions that i'm not sure how they work
// I might try to reduce the size of these later if time allows

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
      xbee_serial.print("GPS: ");
      xbee_serial.print(GPS.latitudeDegrees, 4);
      xbee_serial.print(", ");
      xbee_serial.print(GPS.latitudeDegrees, 4);
    }else{
      Serial.print("No Fix");
      xbee_serial.print("GPS: NO FIX");
    }
  }
}

