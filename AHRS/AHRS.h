#ifndef AHRS_H
#define AHRS_H

#define BETA 1
#define EPSILON		0.0001f
#define PI		3.14159265359f

class MadgwickAHRS {
	private :	
		float _SamplePeriod;
		float _Beta; //algorithm gain beta.
		float _Quaternion[4];
		
	public:
		MadgwickAHRS(void);
		~MadgwickAHRS(void);
		
		void getYawPitchRoll(float* yaw, float* pitch, float* roll);
		void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float samplePeriod);
		void Update(float gx, float gy, float gz, float ax, float ay, float az,float samplePeriod);
		void PrintQuat();
};
#endif