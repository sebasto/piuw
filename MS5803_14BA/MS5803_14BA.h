#ifndef __MS5803_14BA_H_
#define __MS5803_14BA_H_

#define MS5803_14BA_I2C_ADDRESS 0x76
#define MS5803_14BA_I2C_DELAY 5 

class MS5803_14BA {
	// Here are the commands that can be sent to the 5803
	// Page 6 of the data sheet
private :
	static const unsigned char Reset = 0x1E;
	static const unsigned char D1_256 = 0x40; 
	static const unsigned char D1_512 = 0x42;
	static const unsigned char D1_1024 = 0x44;
	static const unsigned char D1_2048 = 0x46;
	static const unsigned char D1_4096 = 0x48;
	static const unsigned char D2_256 = 0x50;
	static const unsigned char D2_512 = 0x52;
	static const unsigned char D2_1024 = 0x54;
	static const unsigned char D2_2048 = 0x56; 
	static const unsigned char D2_4096 = 0x58;
	static const unsigned char AdcRead = 0x00;
	static const unsigned char PromBaseAddress = 0xA0;
	
	unsigned int CalConstant[8];  // Array for calibration constants
	long getAdcData(unsigned char command);
	float _temperature;
	float _pressure;

public:
	MS5803_14BA(void);
	~MS5803_14BA(void);
	
	void updateData();
	float getTemperature();
	float getPressure();
};
#endif
