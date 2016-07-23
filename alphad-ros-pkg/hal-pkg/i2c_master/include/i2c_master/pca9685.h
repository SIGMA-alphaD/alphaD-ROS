#ifndef PCA9685_H
#define PCA9685_H

#define PCA9685_ADDR		0x40

// Setup registers
#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

// Define first LED and all LED. We calculate the rest
#define LED0_ON_L 0x6
#define LEDALL_ON_L 0xFA

#define PIN_ALL 16

namespace pca9685{
	class handler{

	public:
		handler();
		int Setup(float freq);
		void Reset();
		void AnalogWrite(int pin, int value);

	private:
		int fd;
		void pca9685PWMFreq(float freq);
		void pca9685PWMWrite(int pin, int on, int off);
		void pca9685PWMRead(int pin, int *on, int *off);
		void pca9685FullOn(int pin, int tf);
		void pca9685FullOff(int pin, int tf);
		int baseReg(int pin);

	};
}

#endif
