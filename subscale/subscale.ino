#if defined(ARDUINO)
/* Mandatory includes for Arduino */
#include <SPI.h>
#include <Wire.h>
#endif

#include <Adafruit_BMP085.h>
#include <SoftwareSerial.h>

// Starting the accelerometer protocol
//MMA_7455 accel = MMA_7455(i2c_protocol);

// Defining the pins for the xbee radio
const int RXPin = A2;
const int TXPin = A3;

bool True = true;

char str;

// Starting the xbee radio as a serial interface
SoftwareSerial Xbee(RXPin, TXPin);

// Variables that hold the informaton coming from the accelerometer
int16_t x10, y10, z10;
float   xg, yg, zg;

// Counts the number of times the loop has run to get an estimate on the total time
int counter;

// Variables to hold information from the barometer
float Pressure;
float Temperature;
float Altitude;

// Starting the barometer
Adafruit_BMP085 bmp;

int Time = 0;


void setup() {
  //Open Serial connection
  Serial.begin(9600);
  //Start xbee serial connection
  Xbee.begin(9600);
  // Accelerometer initilization  
  //accel.begin();

  // Barometer initilization
  if (!bmp.begin()) {
  Xbee.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
  }

  pinMode(LED_BUILTIN, OUTPUT);
  
  //accel.setSensitivity(2);
  /* Verify sensibility - optional */
  //if(accel.getSensitivity() != 2)   Serial.println("Sensitivity failure");
  /* Set accelerometer mode */
  //accel.setMode(measure);
  /* Verify accelerometer mode - optional */
  //if(accel.getMode() != measure)    Serial.println("Set mode failure");
  /* Set axis offsets */
  /* Note: the offset is hardware specific
   * and defined thanks to the auto-calibration example. */
  //accel.setAxisOffset(0, 0, 0);

}

void loop() {
    /* Get 10-bit axis raw values */
  /*x10 = accel.readAxis10('x');
  y10 = accel.readAxis10('y');
  z10 = accel.readAxis10('z');
  /* Get 10-bit axis values in g */
  /*xg  = accel.readAxis10g('x');
  yg  = accel.readAxis10g('y');
  zg  = accel.readAxis10g('z');*/

  // Read the data from the barometer
  Pressure = bmp.readPressure();
  Temperature = bmp.readTemperature();
  Altitude = bmp.readAltitude();
  
  Xbee.print("M:");
  Xbee.print(Time);
  Xbee.println();
  // Send acceleration values to the xbee to send out
  // X acceleration in decimal form
  /*Xbee.print("x:");
  Xbee.print(x10, DEC);
  Xbee.println();
  // Y acceleration in decimal form
  Xbee.print("y:");
  Xbee.print(y10, DEC);
  Xbee.println();
  // Z acceleration in decimal form
  Xbee.print("z:");
  Xbee.print(z10, DEC);
  Xbee.println();*/ 

  // Barometer Data
  Xbee.print("P:");
  Xbee.print(Pressure, DEC);
  Xbee.println();

  Xbee.print("T:");
  Xbee.print(Temperature, DEC);
  Xbee.println();

  Xbee.print("A:");
  Xbee.print(Altitude, DEC);
  Xbee.println();


  str = Xbee.read();
  if(str == '?'){
     while(True == true) {
        digitalWrite(LED_BUILTIN, HIGH);
        Xbee.print("I'm Here");
     }
  }

  Time+=1;
  
  delay(100);
}
