#include <SoftwareSerial.h>
const int xbee_rx = A2;
const int xbee_tx = A3;

SoftwareSerial xbee_serial(xbee_rx, xbee_tx);

void setup() {
  xbee_serial.begin(9600);
  delay(500);
}

void loop() {
  xbee_serial.println("Hello");
  delay(1000);

}
