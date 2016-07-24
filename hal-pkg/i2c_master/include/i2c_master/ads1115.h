#ifndef ADS1115_H
#define ADS1115_H

#define ADS1115_ADDR					0x48

#define ADS1115_CONFIG_OS_SINGLE		0x8000
#define ADS1115_CONFIG_MUX_OFFSET		12
#define ADS1115_CONFIG_MODE_CONTINUOUS	0X0000
#define ADS1115_CONFIG_MODE_SINGLE		0X0100
#define ADS1115_CONFIG_COMP_QUE_DISABLE	0x0003

#define ADS1115_POINTER_CONVERSION		0x00
#define ADS1115_POINTER_CONFIG			0x01

namespace ads1115{
	class handler{
		
		public:
			handler();
			int Setup(int mygain, int mydata_rate);
			float AnalogRead(int channel);
		private:
			int fd;
			int gain;
			int data_rate;
			float pga;
	};
}

#endif
