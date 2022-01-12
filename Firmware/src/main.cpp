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
#include <MPU.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <PID.h>

void set_pulse_duration(float pulseTime);
float pulse_time(float speed);
void start_timer();
void stop_timer();
void twiddle_pid();
float get_angle();
void update_pid(int index, float value);

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

float lastYaw, lastRoll, lastPitch;

int timer = 0;

float speed = 1000;
float tilt;
float angle = 0;

// Twiddle params
float p[] = {0, 0, 0};
float dp[] = {1, 1, 1};
float bestError;
bool first = true;;

PID anglePID(ANGLE_P, ANGLE_I, ANGLE_D, -MAX_SPEED, MAX_SPEED);
PID speedPID(SPEED_P, SPEED_I, SPEED_D, -MAX_ANGLE + 10.0f, MAX_ANGLE - 10.0f);

float ax,ay,az, gx, gy, gz, mx, my, mz, ox, oy, oz;
MPU mpu = MPU();

AccelStepper rStepper(AccelStepper::DRIVER, R_STEP_PIN, R_DIR_PIN);
AccelStepper lStepper(AccelStepper::DRIVER, L_STEP_PIN, L_DIR_PIN);



void setup() {
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = 0x000A;                                            // When timer is equal to this value, interrupt is triggered
    TIMSK1 |= (1 << OCIE1A);                                   // Enable interrupt for OCR1A match
    TCCR1B |= (1 << WGM12);                                    // CTC mode - reset timer on match
    DDRD |= (1 << R_STEP_PIN) | (1 << L_STEP_PIN) | (1 << L_DIR_PIN) | (1 << R_DIR_PIN); // Set pins to output
    //start();
    start_timer(); 
    Serial.begin(BAUD_RATE);
    Wire.begin();
    Wire.setClock(400000);
    //imu.init();
    mpu.init_bno();
    //InitSensors();
    //OutputForCalibration();
    //GetOffsetsAndInitialQuat();
    //previousTime = micros();
    //set_pulse_duration(pulse_time(34));
    rStepper.setMaxSpeed(MAX_SPEED);
    lStepper.setMaxSpeed(MAX_SPEED);
    //rStepper.setSpeed(1500);
    
}

void loop() {
    mpu.poll_bno();
    if(first) {
        Serial.println("twiddling");
        twiddle_pid();
        Serial.print("P: ");
        Serial.print(anglePID.getP());
        Serial.print(" I: ");
        Serial.print(anglePID.getI());
        Serial.print(" D: ");
        Serial.println(anglePID.getD());
        first = false;
    }

    //mpu.print_all_sensor_data();
    
    mpu.get_acc(ax, ay, az);
    mpu.get_gyo(gx, gy, gz);
    mpu.get_mag(mx, my, mz);
    mpu.get_orient(ox, oy, oz);
    mpu.cal_attitude();
    Serial.print("Accel: ");
    Serial.print("\tx= ");
    Serial.print(ax);
    Serial.print(" |\ty= ");
    Serial.print(ay);
    Serial.print(" |\tz= ");
    Serial.print(az);

    Serial.print(" Gyro: ");
    Serial.print("\tx= ");
    Serial.print(gx);
    Serial.print(" |\ty= ");
    Serial.print(gy);
    Serial.print(" |\tz= ");
    Serial.print(gz);

    Serial.print(" Mag: ");
    Serial.print("\tx= ");
    Serial.print(mx);
    Serial.print(" |\ty= ");
    Serial.print(my);
    Serial.print(" |\tz= ");
    Serial.print(mz);
   
    Serial.print(" Orient: ");
    Serial.print("\tx= ");
    Serial.print(ox);
    Serial.print(" |\ty= ");
    Serial.print(oy);
    Serial.print(" |\tz= ");
    Serial.print(oz); 
    
    Serial.print(" Pitch: ");
    Serial.print(mpu.get_pitch_deg());
    Serial.print( " Roll: ");
    Serial.print(mpu.get_roll_deg());

    //imu.madgwick_filter();
    ////imu.mahony_filter();
    //imu.cal_attitude();
    //imu.cal_heading();   
    //float pitch = imu.get_pitch_deg();
    //float roll = imu.get_roll_deg();
    //float yaw = imu.get_yaw_deg();
    //float alpha = 1;
    //speed = roll;
    //imu.get_acc(ax, ay, az);   
    //Serial.print("---- Filtered Yaw: ");
    //Serial.print(alphaFilter(yaw, lastYaw, alpha));
    //Serial.print(" Filtered Pitch: ");
    //Serial.print(alphaFilter(pitch, lastPitch, alpha));
    //Serial.print(" Filtered Roll: ");
    //Serial.println(alphaFilter(roll, lastRoll, alpha));
    //float angle = imu.get_pitch_deg();
    
    constexpr float complementGyro = 1.0f - (1.0f / REFRESH_RATE);
    constexpr float complementAcc = 1.0f / REFRESH_RATE;
    //float accelerationY = atan(-1 * (ax / ACC_RATE) / sqrt(pow((ay / ACC_RATE), 2) + pow((az / ACC_RATE), 2))) * RAD_TO_DEG - CG; // Calculate accelerometer angle from y-axis
    //float accelerationY = mpu.get_pitch_deg();
    //angle += (float)(gy) * -1; /// GYRO_RATE / REFRESH_RATE;                                                                                   // Calculate the angular rotation gyro has measured from this loop
    //angle = mpu.get_pitch_deg();
    //angle = complementGyro * angle + complementAcc * accelerationY;
    angle = get_angle(); 
    float Vspeed = anglePID.compute(angle - targetAngle);
    Serial.print("    Angle: "); 
    Serial.print(angle);
    Serial.print("    Speed: ");
    Serial.print(Vspeed);
    tilt = Vspeed;//map(Vspeed, -MAX_ANGLE, MAX_ANGLE, -MAX_SPEED, MAX_SPEED);
    Serial.print("    Tilt: "); 
    Serial.println(tilt);
    delay(REFRESH_RATE);
    //rStepper.move(1000);
    //rStepper.setSpeed(m);
    //rStepper.runSpeed();

}

void update_pid(int index, float value) {
    anglePID.reset(); 
    switch(index) {
        case 0:
            anglePID.setP(value);
            break;
        case 1:
            anglePID.setI(value);
            break;
        case 2:
            anglePID.setD(value);
            break;
    }
}

void twiddle_pid() {
    for(int i = 0; i < 3; i++) {
        p[i] = 0;
        dp[i] = 1;
    }
    angle = get_angle();
    float error = anglePID.compute(angle - targetAngle);
    bestError = error;
    while((dp[0] + dp[1] + dp[2]) > TWID_THRESH) {
        for(int i = 0; i < 3; i++) {
            p[i] += dp[i]; 
            update_pid(i, p[i]);
            angle = get_angle();
            error = anglePID.compute(angle-targetAngle);
            tilt = error;
            if(error < bestError) {
                bestError = error;
                dp[i] *= 1.1;
            } 
            else {
                p[i] -= 2 * dp[i];
                update_pid(i, p[i]); 
                angle = get_angle();
                error = anglePID.compute(angle-targetAngle);
                tilt = error; 
                if(error < bestError) {
                     bestError = error;
                     dp[i] *= 1.05;
                }
                else {
                    p[i] += dp[i];
                    update_pid(i, p[i]);
                    dp[i] *= 0.95;
                }
            }
            //update_pid(i, p[i]);

            Serial.println(p[i]);
        }
    }
    //anglePID.setPID(p[0], p[1], p[2]);    
}
float get_angle() {
    mpu.poll_bno();
    mpu.get_acc(ax, ay, az);
    mpu.get_gyo(gx, gy, gz);
    mpu.get_mag(mx, my, mz);
    mpu.get_orient(ox, oy, oz);
    mpu.cal_attitude();
    float reading = mpu.get_pitch_deg();
    reading += gx * -1;
    return reading;
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
    if(tilt > MAX_SPEED) {
        tilt = MAX_SPEED;
    }
    else if(tilt < -MAX_SPEED) {
        tilt = -MAX_SPEED;
    }
    rStepper.setSpeed(tilt * -1);
    lStepper.setSpeed(tilt); 
    rStepper.runSpeed();
    lStepper.runSpeed();
}