#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define MOTOR_DATA 0x00
#define IMU_DATE   0x01
#define ACK        0x02


const int xstepPin = 2; //X.STEP
const int xdirPin = 5; // X.DIR

const int ystepPin = 4; // Y.STEP
const int ydirPin = 7; // Y.DIR
int dir = 0;
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
    byte a = Serial.read();
    while(a != ACK) { 
        a = Serial.read();
    }
    Serial.write(ACK);
}

void loop() {
    if(Serial.available() > 0) {
        dir = !dir; 
        char step = Serial.read();
        for(int i = step; i > 0; i--) {
            drive_motor(0, 1, !dir, 400 + 100);
            drive_motor(1, 1, dir, 400 + 100);
        }
        Serial.write(ACK);
    }
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
    digitalWrite(dpin, dir);
    for(int i = 0; i < step; i++) {
        digitalWrite(spin, HIGH);
        delayMicroseconds(pulseDelay);
        digitalWrite(spin,LOW);
        delayMicroseconds(pulseDelay);
    }
}
