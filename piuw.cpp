#include <iostream>
#include "MS5803_14BA.h"
#include "mpu9150.h"


extern "C" {
#include "linux_glue.h"
}

int main(){
	MS5803_14BA profsensor;
	MPU9150AHRS mpu;
	int samplefreq = 100;//freq of measures in Hz
	int delayBetweenTwoMeasuresMs;
	unsigned long lastTimestamp;
	unsigned long currentTimestamp;
	unsigned long deltaT;
	
	delayBetweenTwoMeasuresMs = 1000 / samplefreq;
	
	while (1) {
		get_ms(&currentTimestamp);
		deltaT = currentTimestamp - lastTimestamp;
		
		if (deltaT < delayBetweenTwoMeasuresMs)  {
			delay_ms(delayBetweenTwoMeasuresMs - deltaT);
		}
		
		//profsensor.updateData();
		mpu.updateData();
		mpu.printYawPitchRoll();
		
		//std::cout << "Temperature = " << profsensor.getTemperature() << "\n";
		//std::cout << "Pressure in bars : " << profsensor.getPressure() << "\n";
	}
}