#include "AHRS.h"
#include <math.h>
#include <iostream>

MadgwickAHRS::MadgwickAHRS()
{
	_Beta = BETA;
	_Quaternion[0] = 1.0; 
	_Quaternion[1] = 0.0;
	_Quaternion[2] = 0.0;
	_Quaternion[3] = 0.0;
}

MadgwickAHRS::~MadgwickAHRS()
{
	delete(_Quaternion);
}

void MadgwickAHRS::PrintQuat(void) {
	std::cout << "q0 : " << _Quaternion[0] << " q1 : " << _Quaternion[1] << " q2 : " << _Quaternion[2] << " q3 : " << _Quaternion[3] <<"\n";
}

void MadgwickAHRS::getYawPitchRoll(float* yaw, float* pitch, float* roll )
{
  float gx, gy, gz; // estimated gravity direction
  
  gx = 2 * (_Quaternion[1]*_Quaternion[3] - _Quaternion[0]*_Quaternion[2]);
  gy = 2 * (_Quaternion[0]*_Quaternion[1] + _Quaternion[2]*_Quaternion[3]);
  gz = _Quaternion[0]*_Quaternion[0] - _Quaternion[1]*_Quaternion[1] - _Quaternion[2]*_Quaternion[2] + _Quaternion[3]*_Quaternion[3];
  
  *yaw = atan2(2 * _Quaternion[1] * _Quaternion[2] - 2 * _Quaternion[0] * _Quaternion[3], 2 * _Quaternion[0]*_Quaternion[0] + 2 * _Quaternion[1] * _Quaternion[1] - 1) * 180/M_PI;
  *pitch = atan(gx / sqrt(gy*gy + gz*gz))  * 180/PI;
  *roll = atan(gy / sqrt(gx*gx + gz*gz))  * 180/PI;
}

/// <param name="gx">
/// Gyroscope x axis measurement in radians/s.
/// </param>
/// <param name="gy">
/// Gyroscope y axis measurement in radians/s.
/// </param>
/// <param name="gz">
/// Gyroscope z axis measurement in radians/s.
/// </param>
/// <param name="ax">
/// Accelerometer x axis measurement in any calibrated units.
/// </param>
/// <param name="ay">
/// Accelerometer y axis measurement in any calibrated units.
/// </param>
/// <param name="az">
/// Accelerometer z axis measurement in any calibrated units.
/// </param>
/// <param name="mx">
/// Magnetometer x axis measurement in any calibrated units.
/// </param>
/// <param name="my">
/// Magnetometer y axis measurement in any calibrated units.
/// </param>
/// <param name="mz">
/// Magnetometer z axis measurement in any calibrated units.
/// </param>
/// <remarks>

void MadgwickAHRS::Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float samplePeriod)
{
	float q1 = _Quaternion[0], q2 = _Quaternion[1], q3 = _Quaternion[2], q4 = _Quaternion[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0 * q1;
	float _2q2 = 2.0 * q2;
	float _2q3 = 2.0 * q3;
	float _2q4 = 2.0 * q4;
	float _2q1q3 = 2.0 * q1 * q3;
	float _2q3q4 = 2.0 * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = (float)sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0) return; // handle NaN
	norm = 1 / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = (float)sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0) return; // handle NaN
	norm = 1 / norm;        // use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0 * q1 * mx;
	_2q1my = 2.0 * q1 * my;
	_2q1mz = 2.0 * q1 * mz;
	_2q2mx = 2.0 * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = (float)sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0 * _2bx;
	_4bz = 2.0 * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q2 * (1 - 2.0 * q2q2 - 2.0 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q3 * (1 - 2.0 * q2q2 - 2.0 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
	norm = 1.0 / (float)sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of _Quaternion
	qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - _Beta * s1;
	qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - _Beta * s2;
	qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - _Beta * s3;
	qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - _Beta * s4;

	// Integrate to yield _Quaternion
	q1 += qDot1 * samplePeriod;
	q2 += qDot2 * samplePeriod;
	q3 += qDot3 * samplePeriod;
	q4 += qDot4 * samplePeriod;
	norm = 1.0 / (float)sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise _Quaternion
	_Quaternion[0] = q1 * norm;
	_Quaternion[1] = q2 * norm;
	_Quaternion[2] = q3 * norm;
	_Quaternion[3] = q4 * norm;
}

void MadgwickAHRS::Update(float gx, float gy, float gz, float ax, float ay, float az,float samplePeriod)
{
	float q1 = _Quaternion[0], q2 = _Quaternion[1], q3 = _Quaternion[2], q4 = _Quaternion[3];   // short name local variable for readability
	float norm;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1 = 2.0 * q1;
	float _2q2 = 2.0 * q2;
	float _2q3 = 2.0 * q3;
	float _2q4 = 2.0 * q4;
	float _4q1 = 4.0 * q1;
	float _4q2 = 4.0 * q2;
	float _4q3 = 4.0 * q3;
	float _8q2 = 8.0 * q2;
	float _8q3 = 8.0 * q3;
	float q1q1 = q1 * q1;
	float q2q2 = q2 * q2;
	float q3q3 = q3 * q3;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = (float)sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0) return; // handle NaN
	norm = 1 / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Gradient decent algorithm corrective step
	s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
	s2 = _4q2 * q4q4 - _2q4 * ax + 4.0 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
	s3 = 4.0 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
	s4 = 4.0 * q2q2 * q4 - _2q2 * ax + 4.0 * q3q3 * q4 - _2q3 * ay;
	norm = 1.0 / (float)sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of _Quaternion
	qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - _Beta * s1;
	qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - _Beta * s2;
	qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - _Beta * s3;
	qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - _Beta * s4;

	// Integrate to yield _Quaternion
	q1 += qDot1 * samplePeriod;
	q2 += qDot2 * samplePeriod;
	q3 += qDot3 * samplePeriod;
	q4 += qDot4 * samplePeriod;
	norm = 1.0 / (float)sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise _Quaternion
	_Quaternion[0] = q1 * norm;
	_Quaternion[1] = q2 * norm;
	_Quaternion[2] = q3 * norm;
	_Quaternion[3] = q4 * norm;
}