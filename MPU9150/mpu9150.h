////////////////////////////////////////////////////////////////////////////
//
//  This file is part of linux-mpu9150
//
//  Copyright (c) 2013 Pansenti, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef MPU9150_H
#define MPU9150_H

#include "AHRS.h"

#define BETA_AHRS 0.2

#define MPU9150_I2C_ADDRESS 0x76
#define MPU9150_I2C_DELAY 2

// The following LPF settings are supported: 188, 98, 42, 20, 10, 5 (Hz)
#define LOW_PASS_FILTER 0
#define MPUSAMPLERATE 100 //sample rate of gyro and accel (Hz)
#define MAGSAMPLERATE 100 //sample rate of compass (Hz)
#define PI 3.14159265359f
#define MAG_SENSOR_RANGE      4096


class MPU9150AHRS{
	private :	
		float _heading;
		float _Gyro[3]; //gyro (rad/s)
		float _gyroOffset[3];
		float _gyroSens; //gyro sensibility (deg/s)
		float _Acc[3]; //accelerometer (g's)
		float _accOffset[3];
		unsigned short _accSens; //accelerometer sensibility (g's)
		float _Mag[3];
		short _magOffset[3];
		short _magRange[3];
		long _rawTemp;
		unsigned long _lastMeasureTimestamp;
		MadgwickAHRS * _ahrs;
		
		int load_cal(void);
		void initGyroOffsets(void);

	public:
		MPU9150AHRS(void);
		~MPU9150AHRS(void);
		
		void updateData();
		void printRawData();
		void printYawPitchRoll();
		void printQuat();
};
#endif /* MPU9150_H */

