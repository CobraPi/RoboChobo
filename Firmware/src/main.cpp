#include <Arduino.h>

const int xstepPin = 2; //X.STEP
const int xdirPin = 5; // X.DIR

const int ystepPin = 4; // Y.STEP
const int ydirPin = 7; // Y.DIR

//char data[4]; 

void drive_motor(int motor, int step, int dir, int pulseDelay);
bool direction = 1;

void setup() {
    // Sets the two pins as Outputs
    pinMode(xstepPin,OUTPUT); 
    pinMode(xdirPin,OUTPUT);
    pinMode(ystepPin,OUTPUT); 
    pinMode(ydirPin,OUTPUT);
    Serial.begin(9600);
    Serial.println("----stepper test ----");


}

void loop() {
    //digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
    // Makes 200 pulses for making one full cycle rotation

    // for(int i = 100; i < 200; i++) {
    //   for(int j = 25; j > 0; j--) {
    //     drive_motor(0, 10, i % 2, i);
    //     drive_motor(1, 10, i % 2, i);
    //   }
    // }
    if(Serial.available() > 0) {
        String data = Serial.readString();
        int step = data.toInt();
        for(int i = step; i > 0; i--) {
            drive_motor(0, 1, 0, i + 100);
            drive_motor(1, 1, 1, i + 100);
        }
        Serial.println(data);
    }

    /*

    for(int i = 800; i > 100; i--) {
        if(i%10 == 0) {
        
        drive_motor(1, 1, direction, i);
        drive_motor(0, 1, direction, i);
        }
    }
    direction = !direction;
    
    for(int i = 100; i < 800; i++) {

        drive_motor(1, 1, direction, i);
        drive_motor(0, 1, direction, i);
    }


    //delay(500); // One second delay


*/
}

/*
drives the steppers a according to parameters provided
*/
void drive_motor(int motor, int step, int dir, int pulseDelay) {
    int spin,dpin;
    if(motor) {
    spin = ystepPin;
    dpin = ydirPin;
    }
    else {
    spin = xstepPin;
    dpin = xdirPin;
    }
    digitalWrite(dpin, dir);54
    for(int i = 0; i < step; i++) {
        digitalWrite(spin, HIGH);
        delayMicroseconds(pulseDelay);
        digitalWrite(spin,LOW);
        delayMicroseconds(pulseDelay);
    }
}
