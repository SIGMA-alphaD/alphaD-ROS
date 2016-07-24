#include <wiringPi.h>
#include <wiringPiI2C.h>

#include "i2c_master/ads1115.h"

ads1115::handler::handler(){
	fd = 0;
	gain = 0x0000;
	data_rate = 0x0000;
	pga = 2.048;
}

int ads1115::handler::Setup(int mygain, int mydata_rate){
	
	// Check i2c address
	fd = wiringPiI2CSetup(ADS1115_ADDR);
	if (fd < 0)
		return fd;

	int ADS1115_GAIN[] = {
		0x0000,		// 2/3, FS = +-6.114V
		0x0200,		// 1  , FS = +-4.096V
		0x0400,		// 2  , FS = +-2.048V
		0x0600,		// 4  , FS = +-1.024V
		0x0800,		// 8  , FS = +-0.512V
		0x0A00		//16  , FS = +-0.256V
	};

	int ADS1115_DR[] = {
		0x0000,		// 8 sps
		0x0020,		// 16 sps
		0x0040,		// 32 sps
		0x0060,		// 64 sps
		0x0080,		// 128 sps (default)
		0x00A0,		// 250 sps
		0x00C0,		// 475 sps
		0x00E0,		// 860 sps	
	};

	if(mygain >= 0 && mygain <= 5)
		gain = ADS1115_GAIN[mygain];
	else
		gain = ADS1115_GAIN[0];

	switch(gain){
		case 0x0000:
			pga = 6.144;
			break;
		case 0x0020:
			pga = 4.096;
			break;
		case 0x0400:
			pga = 2.048;
			break;
		case 0x0600:
			pga = 1.024;
			break;
		case 0x0800:
			pga = 0.512;
			break;
		case 0x0A00:
			pga = 0.256;
			break;
	}
	
	if(mydata_rate >= 0 && mydata_rate <= 7)
		data_rate = ADS1115_DR[mydata_rate];
	else
		data_rate = ADS1115_DR[0];

	return fd;
}

float ads1115::handler::AnalogRead(int channel){
	
	int config, value;
	float voltage;
	
	// Go out of power-down mode for conversion
	config = ADS1115_CONFIG_OS_SINGLE;

	// Specify mux value. Select pin
	config |= ((channel + 0x04) & 0x07) << ADS1115_CONFIG_MUX_OFFSET;

	// Set the gain
	config |= gain;

	// Set the mode (continuouse or single shot)
	config |= ADS1115_CONFIG_MODE_SINGLE;

	// Set the data_rate
	config |= data_rate;

	// Disable comparator mode
	config |= ADS1115_CONFIG_COMP_QUE_DISABLE;

	// Switch LSB & MSB
	config = ((config & 0x00FF) << 8) | ((config >> 8) & 0x00FF);

	// Write
	wiringPiI2CWriteReg16(fd, ADS1115_POINTER_CONFIG, config);
	delay(100);

	// Read ADC value
	value = wiringPiI2CReadReg16(fd, ADS1115_POINTER_CONVERSION);

	// Switch LSB & MSB
	value = (value >> 8) | ((value & 0x00FF) << 8);

	if((value & 0x8000) != 0)
		value -= 1 << 16;
	
	voltage = (float)value/32768.0*pga*6.0;

	return voltage;
}
