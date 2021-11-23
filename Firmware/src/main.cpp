/*

Packet Structure: {start_bit,
                   control_code,
                   data_length,
                   data
                   end_bit}


*/


#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define BAUD_RATE 9600

#define ACK   0x06
#define END   0x00 
#define BEGIN 0x01
#define DEBUG 0x08   

#define MOTOR_DATA 0x32
#define IMU_DATA   0x33

const int xstepPin = 2; //X.STEP
const int xdirPin = 5; // X.DIR

const int ystepPin = 4; // Y.STEP
const int ydirPin = 7; // Y.DIR
int dir = 0;
//char data[4]; 

void debug();
void drive_motor(int motor, int step, int dir, int pulseDelay);
bool direction = 1;

void setup() {
    // Sets the two pins as Outputs
    pinMode(xstepPin,OUTPUT); 
    pinMode(xdirPin,OUTPUT);
    pinMode(ystepPin,OUTPUT); 
    pinMode(ydirPin,OUTPUT);
    
    Serial.begin(BAUD_RATE);
    
    byte ack = Serial.read(); // initialize the handshake variable
    while(ack != ACK) {       // listen for the handchake character  
        ack = Serial.read();
    }
    Serial.write(ACK); // acknowledge successful connection
}

void loop() {
    // check if there is any data in the serial buffer
    if(Serial.available() > 0 && Serial.read() == BEGIN) {
        // packet structure [BEGIN, control_code, data_length, [data],END]
        debug();
   }
}

void debug() {
    char data[3];
    Serial.readBytesUntil(END, data, 3);
    for(int i = 0; i < 3; i++) {
        Serial.print(data[i]);
        //Serial.write(data[i]);
    }
    Serial.write(END);
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
    digitalWrite(dpin, dir); // Set direction of stepper
    
    // pulse the motor for the desired number of steps 
    for(int i = 0; i < step; i++) {
        digitalWrite(spin, HIGH);
        delayMicroseconds(pulseDelay);
        digitalWrite(spin,LOW);
        delayMicroseconds(pulseDelay);
    }
}
