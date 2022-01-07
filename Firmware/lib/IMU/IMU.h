//
// Created by joey on 8/23/16.
//

#ifndef IMU_H
#define IMU_H
#include <avr/io.h>
#include <LSM303.h>
#include <L3G.h>
#include <Wire.h>
#include <Utils.h>

#define GYRO_SCALE_FACTOR 0.0175
#define ACC_SCALE_FACTOR 0.019140625

#define DECLINATION 14.1//12.7 // Declination (degrees) in Philadelphia, PA

enum AXIS {X, Y, Z};

class IMU {

private:
    L3G L3GOBJ;
    LSM303 LSM303OBJ;

    // Raw IMU component values
    float _rawAx, _rawAy, _rawAz;
    float _rawGx, _rawGy, _rawGz;
    float _rawMx, _rawMy, _rawMz;

    // Filtered IMU component values
    float _ax, _ay, _az;
    float _gx, _gy, _gz;
    float _mx, _my, _mz;

    // Gravity in G's
    float _axG, _ayG, _azG;

    // Attitude
    float _pitchRadians;
    float _rollRadians;

    float _pitchDegrees;
    float _rollDegrees;


    // Orientation
    float _heading;


public:
    

    IMU();
    void init();
    void poll();

    void get_acc(float &x, float &y, float &z);
    void get_gyo(float &x, float &y, float &z);
    void get_mag(float &x, float &y, float &z);

    void mahony_filter();
    void madgwick_filter();

    void cal_attitude();
    void cal_heading();

    float get_yaw_rad();
    float get_pitch_rad();
    float get_roll_rad();

    float get_yaw_deg();
    float get_pitch_deg();
    float get_roll_deg();




};


#endif //IMU_H
