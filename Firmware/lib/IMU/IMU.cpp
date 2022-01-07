//
// Created by joey on 8/23/16.
//

#include "IMU.h"



#define sampleFreq	512.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain
#define betaDef		0.1f		// 2 * proportional gain (Madgwick)
//---------------------------------------------------------------------------------------------------
// Variable definitions
volatile float beta = betaDef;								// 2 * proportional gain (Kp) (Madgwick)
volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
float invSqrt(float x);
//------------

IMU::IMU() {
    // Initialize class attributes
    _rawAx = 0;
    _rawAy = 0;
    _rawAz = 0;

    _rawGx = 0;
    _rawGy = 0;
    _rawGz = 0;

    _rawMx = 0;
    _rawMy = 0;
    _rawMz = 0;

    _ax = 0;
    _ay = 0;
    _az = 0;

    _gx = 0;
    _gy = 0;
    _gz = 0;

    _mx = 0;
    _my = 0;
    _mz = 0;

    _axG = 0;
    _ayG = 0;
    _azG = 0;

    _pitchRadians = 0;
    _rollRadians = 0;

    _pitchDegrees = 0;
    _rollDegrees = 0;

    _heading = 0;

};

void IMU::init() {
    L3GOBJ.init();
    LSM303OBJ.init();
    L3GOBJ.writeReg(L3GOBJ.CTRL_REG1, 0x0F); //100hz
	L3GOBJ.writeReg(L3GOBJ.CTRL_REG4, 0x10); //500dps

    LSM303OBJ.writeAccReg(LSM303OBJ.CTRL_REG4_A, 0x10); //Continuous update little endian +/- 4g
    LSM303OBJ.writeAccReg(LSM303OBJ.CTRL_REG1_A, 0x57); //normal 100Hz all axes enabled
    LSM303OBJ.writeMagReg(LSM303OBJ.CRA_REG_M, 0x98); //75Hz temp compensation enabled
    LSM303OBJ.writeMagReg(LSM303OBJ.CRB_REG_M, 0xA0);
    LSM303OBJ.writeMagReg(LSM303OBJ.MR_REG_M, 0x00);
}

void IMU::poll() {
    L3GOBJ.read();
    LSM303OBJ.readMag();
    LSM303OBJ.readAcc();
    _rawGx = L3GOBJ.g.x;
    _rawGy = L3GOBJ.g.y;
    _rawGz = L3GOBJ.g.z;

    _rawAx = LSM303OBJ.a.x;
    _rawAy = LSM303OBJ.a.y;
    _rawAz = LSM303OBJ.a.z;

    _rawMx = LSM303OBJ.m.x;
    _rawMy = LSM303OBJ.m.y;
    _rawMz = LSM303OBJ.m.z;

    _ax = _rawAx;
    _ay = _rawAy;
    _az = _rawAz;
    _gx = _rawGx;
    _gy = _rawGy;
    _gz = _rawGz;
    _mx = _rawMx;
    _my = _rawMy;
    _mz = _rawMz; 
}

void IMU::get_acc(float &x, float &y, float &z) {
    x = _ax;
    y = _ay;
    z = _az;
}

void IMU::get_gyo(float &x, float &y, float &z) {
    x = _gx;
    y = _gy;
    z = _gz;
}

void IMU::get_mag(float &x, float &y, float &z) {
    x = _rawMx;
    y = _rawMy;
    z = _rawMz;
}

void IMU::cal_attitude() {
    _rollRadians = atan(_ax / sqrt(pow(_ay, 2.0) + pow(_az, 2.0)));
    _pitchRadians = atan(_ay / sqrt(pow(_ax, 2.0) + pow(_az, 2.0)));

    _rollDegrees = _rollRadians * 180.0 / M_PI;
    _pitchDegrees = _pitchRadians * 180.0 / M_PI;

    _rollDegrees *= -1;
}

void IMU::cal_heading() {
    if (_my == 0) {
        _heading = (_mx < 0.0) ? 180.0 : 0;
    }
    else {
        _heading = atan2(_mx, _my);
    }

    _heading -= DECLINATION * M_PI / 180.0;

    if (_heading > M_PI) {

        _heading -= 2.0 * M_PI;
    }
    else if (_heading < -M_PI) {

        _heading += 2.0 * M_PI;
    }
    else if (_heading < 0.0) {

        _heading += 2.0 * M_PI;
    }
    _heading = _heading * (180.0/M_PI);
}


float IMU::get_pitch_rad() {
    return _pitchRadians;
}

float IMU::get_roll_rad() {
    return _rollRadians;
}

float IMU::get_yaw_deg() {
    return _heading;
}

float IMU::get_pitch_deg() {
    return _pitchDegrees;
}

float IMU::get_roll_deg() {
    return _rollDegrees;
}


void IMU::mahony_filter() {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((_ax == 0.0f) && (_ay == 0.0f) && (_az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(_ax * _ax + _ay * _ay + _az * _az);
		_ax *= recipNorm;
		_ay *= recipNorm;
		_az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (_ay * halfvz - _az * halfvy);
		halfey = (_az * halfvx - _ax * halfvz);
		halfez = (_ax * halfvy - _ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			_gx += integralFBx;	// apply integral feedback
			_gy += integralFBy;
			_gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		_gx += twoKp * halfex;
		_gy += twoKp * halfey;
		_gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	_gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	_gy *= (0.5f * (1.0f / sampleFreq));
	_gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * _gx - qc * _gy - q3 * _gz);
	q1 += (qa * _gx + qc * _gz - q3 * _gy);
	q2 += (qa * _gy - qb * _gz + q3 * _gx);
	q3 += (qa * _gz + qb * _gy - qc * _gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

}

void IMU::madgwick_filter() {
    float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * _gx - q2 * _gy - q3 * _gz);
	qDot2 = 0.5f * (q0 * _gx + q2 * _gz - q3 * _gy);
	qDot3 = 0.5f * (q0 * _gy - q1 * _gz + q3 * _gx);
	qDot4 = 0.5f * (q0 * _gz + q1 * _gy - q2 * _gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((_ax == 0.0f) && (_ay == 0.0f) && (_az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(_ax * _ax + _ay * _ay + _az * _az);
		_ax *= recipNorm;
		_ay *= recipNorm;
		_az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(_mx * _mx + _my * _my + _mz * _mz);
		_mx *= recipNorm;
		_my *= recipNorm;
		_mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * _mx;
		_2q0my = 2.0f * q0 * _my;
		_2q0mz = 2.0f * q0 * _mz;
		_2q1mx = 2.0f * q1 * _mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = _mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + _mx * q1q1 + _2q1 * _my * q2 + _2q1 * _mz * q3 - _mx * q2q2 - _mx * q3q3;
		hy = _2q0mx * q3 + _my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - _my * q1q1 + _my * q2q2 + _2q2 * _mz * q3 - _my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + _mz * q0q0 + _2q1mx * q3 - _mz * q1q1 + _2q2 * _my * q3 - _mz * q2q2 + _mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - _ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - _ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - _mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - _my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - _mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - _ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - _ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - _az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - _mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - _my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - _mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - _ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - _ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - _az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - _mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - _my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - _mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - _ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - _ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - _mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - _my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - _mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}


float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}