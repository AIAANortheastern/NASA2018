#include <Servo.h>

Servo myservo;

const int ServoPin = 6;

void setup() {
    myservo.attach(ServoPin);
}

void loop() {

    myservo.attach(ServoPin);

    delay(15);

    myservo.write(1);

    myservo.detach();

    delay(1000);
    
    myservo.attach(ServoPin);

    delay(15);

    myservo.write(179);

    delay(1000);

    myservo.detach();

    delay(1000);
    
}
