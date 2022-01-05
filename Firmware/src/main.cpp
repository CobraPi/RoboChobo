/*

Packet Structure: {start_bit,
                   control_code,
                   data_length,
                   data
                   end_bit}

Protocol:

Host   ---> Driver 
Driver ---> Host 

*/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <defines.h>

#define BAUD_RATE 9600

#define ACK   0x06
#define END   0x00 
#define START 0x01
#define DEBUG 0x08   

#define MOTOR_DATA 0x32
#define IMU_DATA   0x33
#define RANGE_DATA 0x34

#define STATE_IR_SENSOR_TIMEOUT 0x40

#define IR_SENSOR_TIMEOUT 500

#define DEBUG 

bool direction = 1;
bool moving = 1;
bool targetChange = 0;



void setup() {
    // Sets the two pins as Outputs
    pinMode(X_STEP_PIN, OUTPUT);
    pinMode(X_DIR_PIN, OUTPUT);
    pinMode(Y_STEP_PIN, OUTPUT); 
    pinMode(Y_DIR_PIN, OUTPUT);

    #ifndef SERIAL 
        Serial.begin(BAUD_RATE);
    
        byte ack = Serial.read(); // initialize the handshake variable
        while(ack != ACK) {       // listen for the handchake character  
            ack = Serial.read();
        }
        Serial.write(ACK); // acknowledge successful connection
    #endif

}

void loop() {
    // 1. Computer 

   }

}
