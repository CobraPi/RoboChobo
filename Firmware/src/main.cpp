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
#include <SPI.h>
#include <Wire.h>
#include <defines.h>
//#include "openimu.h"
#include <IMU.h>

void set_pulse_duration(float pulseTime);
float pulse_time(float speed);
void start_timer();
void stop_timer();

bool direction = 1;
bool moving = 1;
bool targetChange = 0;

uint16_t skipRightPulse = 0;
uint16_t skipLeftPulse = 0;
uint16_t numInterrupts = 0;

float currentAngle = 0;
float targetAngle = 0.0f;

int32_t actualPosition = 0;
int32_t targetPosition = 0;

int timer = 0;

float speed = 3;

IMU imu;

void setup() {
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = 0x00FF;                                            // When timer is equal to this value, interrupt is triggered
    TIMSK1 |= (1 << OCIE1A);                                   // Enable interrupt for OCR1A match
    TCCR1B |= (1 << WGM12);                                    // CTC mode - reset timer on match
    DDRD |= (1 << R_STEP_PIN) | (1 << L_STEP_PIN) | (1 << L_DIR_PIN) | (1 << R_DIR_PIN); // Set pins to output
    //start();
    
    Serial.begin(BAUD_RATE);
    Wire.begin();
    Wire.setClock(400000);
    imu.init();
    //InitSensors();
    //OutputForCalibration();
    //GetOffsetsAndInitialQuat();
    //previousTime = micros();
}

void loop() {

    while(true) {
        imu.poll();
        float ax,ay,az, gx, gy, gz, mx, my, mz;
        imu.get_acc(ax, ay, az);
        imu.get_gyo(gx, gy, gz);
        imu.get_mag(mx, my, mz);
        imu.madgwick_filter();
        imu.cal_attitude();
        imu.cal_heading();   
        imu.get_acc(ax, ay, az);   
        Serial.print("---- Filtered Yaw: ");
        Serial.print(imu.get_yaw_deg());
        Serial.print(" Filtered Pitch: ");
        Serial.print(imu.get_pitch_deg());
        Serial.print(" Filtered Roll: ");
        Serial.println(imu.get_roll_deg()); 
    }

    if(millis() - timer > 10) 
    {
        if(speed <= 500 && direction) 
        {
            speed++;
            if(speed >= 500) 
            {
                direction = !direction;
            }
        }
        if(speed >= 0 && !direction) 
        {
            speed--;
            if(speed <= 0) 
            {
                direction = !direction;
            }
        }
        if((int)speed % 2 == 0) 
        {
            skipRightPulse = 0;
            skipLeftPulse = 0;
        }
        else if((int)speed % 100 == 0) {
            skipLeftPulse = 0;
            skipRightPulse = 0;
        } 
        else {
            skipRightPulse = 0;
            skipLeftPulse = 0;
        }
        set_pulse_duration(pulse_time(speed));
        Serial.println(speed);
        timer = millis();
    }

    if (direction)
        {
            PORTD &= ~(1 << L_DIR_PIN); // Normal direction
            PORTD |= (1 << R_DIR_PIN);
        }
        else
        {
            PORTD |= (1 << L_DIR_PIN); // Reversed direction
            PORTD &= ~(1 << R_DIR_PIN);
        }


}

/*
  Set duration for the pulse generated for stepper motors
*/
void set_pulse_duration(float pulseTime)
{
    if (pulseTime < 1)
        return;
    stop_timer();
    OCR1A = MIN_TICKS * pulseTime;
    TCNT1 = (TCNT1 < OCR1A) ? TCNT1 : 0; // Set the counter to 0, if OCR1A is smaller than TCNT1. - Risk of corrupting TCNT1, if timer is not disabled!
    start_timer();
}

float pulse_time(float speed) 
{
    // Do this to avoid overflowing 16-bit register when going slow speed
    // 1000 / 65535 = 0.01525...
    // Keep the limit a bit higher because of Atmega328p's inaccurate floating point calculations
    if (speed < 0.016f && speed > -0.016f) 
    {
        return MAX_SPEED / 0.016f;
    }
    return fabs(MAX_SPEED / speed);
}

/*
  Sets prescaler to 0 to stop the timer
*/
inline void stop_timer()
{
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
}
/*
  Sets prescaler to 64 to start the timer
*/
inline void start_timer()
{
    TCCR1B |= (1 << CS11) | (1 << CS10);
}

#define RMP_HIGH (PIND >> R_STEP_PIN) & 1
#define LMP_HIGH (PIND >> L_STEP_PIN) & 1
#define RMD_HIGH (PIND >> R_DIR_PIN) & 1

ISR(TIMER1_COMPA_vect) 
{
    // Drive right motor 
    if(skipRightPulse == 0) 
    {
        if(RMP_HIGH) {
            // Set pin low
            PORTD &= ~(1 << R_STEP_PIN);
        }
        else {
            // Set pin high
            PORTD |= (1 << R_STEP_PIN);
        }
    }
    // Drive left motor
    if(skipLeftPulse == 0) {
        if(LMP_HIGH) {
            // Set pin low
            PORTD &= ~(1 << L_STEP_PIN);
        }
        else {
            // Set pin high
            PORTD |= (1 << L_STEP_PIN);
        }
    }
    numInterrupts++;
    if(numInterrupts % 2 == 0) {
        actualPosition += (RMD_HIGH) ? -1 : 1;
    }

}