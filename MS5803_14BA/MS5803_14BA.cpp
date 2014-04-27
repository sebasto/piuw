extern "C" {
#include "linux_glue.h"
}
#include <linux/i2c-dev.h>
#include <iostream>
#include "MS5803_14BA.h"

MS5803_14BA::MS5803_14BA(){
#if DEBUG
	std::cout << "Init MS5803_14BA\n";
#endif
	_temperature = 0;
	_pressure = 0;

	int r;
	unsigned char buf[2];
	unsigned char command;
	
	linux_set_i2c_bus(1);
	
	//init the sensor by sending reset
	linux_i2c_write(MS5803_14BA_I2C_ADDRESS, Reset, 0, NULL);
    delay_ms(MS5803_14BA_I2C_DELAY);

	// read the calibration eeprom and initialize constants array
	for(int i = 0; i < 8; i++) {
		command = PromBaseAddress + (unsigned char)(i*2);// output enable | read input i

		if(linux_i2c_read(MS5803_14BA_I2C_ADDRESS, command,2, buf)) {
			perror("reading i2c device\n");
		}
		
		delay_ms(MS5803_14BA_I2C_DELAY);
		CalConstant[i] = (buf[0] << 8) + buf[1];
#if DEBUG
		printf("0x%02x 0x%02x 0x%02x  %d\n", 0xA0 + (i*2),buf[0], buf[1],(int)CalConstant[i]);
#endif
	}
  
}

MS5803_14BA::~MS5803_14BA(){
}

long MS5803_14BA::getAdcData(unsigned char command){
	unsigned char buf[3];
	int r;
	long AdcData;
	
	linux_i2c_write(MS5803_14BA_I2C_ADDRESS, command, 0, NULL);
	delay_ms(MS5803_14BA_I2C_DELAY);
	
	if(linux_i2c_read(MS5803_14BA_I2C_ADDRESS, AdcRead,3, buf)) {
		perror("reading i2c device in MS5803_14BA\n");
	}
	//printf ("got %d bytes in response : 0x%02x 0x%02x 0x%02x\n",r,buf[0],buf[1],buf[2]);
	  
	AdcData = ((long)buf[0] << 16) + ((long)buf[1] << 8) + (long)buf[2];

	return (AdcData);
}

void MS5803_14BA::updateData(void){
	float Temperature, Pressure, TempDifference, Offset, Sensitivity;
	long AdcTemperature = getAdcData(D2_512);
	long AdcPressure = getAdcData(D1_512);
	float T2, Off2, Sens2;  // Offsets for second-order temperature computation
	
	// Calculate the Temperature (first-order computation)
  
	TempDifference = (float)(AdcTemperature - ((long)CalConstant[5] << 8));
	Temperature = (TempDifference * (float)CalConstant[6])/ pow(2, 23);
	Temperature = Temperature + 2000;  // This is the temperature in hundredths of a degree C
  
	// Calculate the second-order offsets

	if (Temperature < 2000.0)  // Is temperature below or above 20.00 deg C ?
	{
		T2 = 3 * pow(TempDifference, 2) / pow(2, 33);
		Off2 = 1.5 * pow((Temperature - 2000.0), 2);
		Sens2 = 0.625 * pow((Temperature - 2000.0), 2);
	}
	else
	{
		T2 = (TempDifference * TempDifference) * 7 / pow(2, 37);
		Off2 = 0.0625 * pow((Temperature - 2000.0), 2); 
		Sens2 = 0.0;
	}
  
	// Print the temperature results
  
	Temperature = Temperature / 100;  // Convert to degrees C
	Temperature = Temperature -(T2 / 100); // Last compensation
	_temperature = Temperature;
		
	// Calculate the pressure parameters

	Offset = (float)CalConstant[2] * pow(2,16);
	Offset = Offset + ((float)CalConstant[4] * TempDifference / pow(2, 7));

	Sensitivity = (float)CalConstant[1] * pow(2, 15);
	Sensitivity = Sensitivity + ((float)CalConstant[3] * TempDifference / pow(2, 8));

	// Add second-order corrections

	Offset = Offset - Off2;
	Sensitivity = Sensitivity - Sens2;

	// Calculate absolute pressure in bars

	Pressure = (float)AdcPressure * Sensitivity / pow(2, 21);
	Pressure = Pressure - Offset;
	Pressure = Pressure / pow(2, 15);
	Pressure = Pressure / 10;  // Set output to mbars = hectopascal;
	_pressure = Pressure;
}

float  MS5803_14BA::getTemperature(void){
	return(_temperature);
}

float MS5803_14BA::getPressure(void){
	return(_pressure);
}