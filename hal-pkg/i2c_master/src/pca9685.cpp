#include <wiringPi.h>
#include <wiringPiI2C.h>

#include "i2c_master/pca9685.h"

pca9685::handler::handler(){
	fd = 0;
}

int pca9685::handler::Setup(float freq){

	// Turn on the pca9685
	wiringPiSetupSys();
	pinMode(27, OUTPUT);
	digitalWrite(27, LOW);

	// Check i2c address
	fd = wiringPiI2CSetup(PCA9685_ADDR);
	if (fd < 0)
		return fd;

	// Setup the chip. Enable auto-increment of registers.
	int settings = wiringPiI2CReadReg8(fd, PCA9685_MODE1) & 0x7F;
	int autoInc = settings | 0x20;

	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, autoInc);
	
	// Set frequency of PWM signals. Also ends sleep mode and starts PWM output.
	if (freq > 0)
		pca9685PWMFreq(freq);

	// Reset PWM
	Reset();

	return fd;
}

/**
 * Set all leds back to default values (: fullOff = 1)
 */
void pca9685::handler::Reset()
{
	wiringPiI2CWriteReg16(fd, LEDALL_ON_L	 , 0x0);
	wiringPiI2CWriteReg16(fd, LEDALL_ON_L + 2, 0x1000);
}

/**
 * Simple PWM control which sets on-tick to 0 and off-tick to value.
 * If value is <= 0, full-off will be enabled
 * If value is >= 4096, full-on will be enabled
 * Every value in between enables PWM output
 */
void pca9685::handler::AnalogWrite(int pin, int value)
{
	// pin mapping 1~8 to 3~10
	pin += 2;

	if (value >= 4096)
		pca9685FullOn(pin, 1);
	else if (value > 0)
		pca9685PWMWrite(pin, 0, value);	// (Deactivates full-on and off by itself)
	else
		pca9685FullOff(pin, 1);
}

/**
 * Write on and off ticks manually to a pin
 * (Deactivates any full-on and full-off)
 */
void pca9685::handler::pca9685PWMWrite(int pin, int on, int off)
{
	int reg = baseReg(pin);

	// Write to on and off registers and mask the 12 lowest bits of data to overwrite full-on and off
	wiringPiI2CWriteReg16(fd, reg	 , on  & 0x0FFF);
	wiringPiI2CWriteReg16(fd, reg + 2, off & 0x0FFF);
}

/**
 * Reads both on and off registers as 16 bit of data
 * To get PWM: mask each value with 0xFFF
 * To get full-on or off bit: mask with 0x1000
 * Note: ALL_LED pin will always return 0
 */
void pca9685::handler::pca9685PWMRead(int pin, int *on, int *off)
{
	int reg = baseReg(pin);

	if (on)
		*on  = wiringPiI2CReadReg16(fd, reg);
	if (off)
		*off = wiringPiI2CReadReg16(fd, reg + 2);
}

/**
 * Enables or deactivates full-on
 * tf = true: full-on
 * tf = false: according to PWM
 */
void pca9685::handler::pca9685FullOn(int pin, int tf)
{
	int reg = baseReg(pin) + 1;		// LEDX_ON_H
	int state = wiringPiI2CReadReg8(fd, reg);

	// Set bit 4 to 1 or 0 accordingly
	state = tf ? (state | 0x10) : (state & 0xEF);

	wiringPiI2CWriteReg8(fd, reg, state);

	// For simplicity, we set full-off to 0 because it has priority over full-on
	if (tf)
		pca9685FullOff(pin, 0);
}

/**
 * Enables or deactivates full-off
 * tf = true: full-off
 * tf = false: according to PWM or full-on
 */
void pca9685::handler::pca9685FullOff(int pin, int tf)
{
	int reg = baseReg(pin) + 3;		// LEDX_OFF_H
	int state = wiringPiI2CReadReg8(fd, reg);

	// Set bit 4 to 1 or 0 accordingly
	state = tf ? (state | 0x10) : (state & 0xEF);

	wiringPiI2CWriteReg8(fd, reg, state);
}

/**
 * Sets the frequency of PWM signals.
 * Frequency will be capped to range [40..1000] Hertz. Try 50 for servos.
 */
void pca9685::handler::pca9685PWMFreq(float freq){
	// Cap at min and max
	freq = (freq > 1000 ? 1000 : (freq < 40 ? 40 : freq));

	// To set pwm frequency we have to set the prescale register. The formula is:
	// prescale = round(osc_clock / (4096 * frequency))) - 1 where osc_clock = 25 MHz
	// Further info here: http://www.nxp.com/documents/data_sheet/PCA9685.pdf Page 24
	int prescale = (int)(25000000.0f / (4096 * freq) - 0.5f);

	// Get settings and calc bytes for the different states.
	int settings = wiringPiI2CReadReg8(fd, PCA9685_MODE1) & 0x7F;	// Set restart bit to 0
	int sleep	= settings | 0x10;									// Set sleep bit to 1
	int wake 	= settings & 0xEF;									// Set sleep bit to 0
	int restart = wake | 0x80;										// Set restart bit to 1

	// Go to sleep, set prescale and wake up again.
	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, sleep);
	wiringPiI2CWriteReg8(fd, PCA9685_PRESCALE, prescale);
	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, wake);

	// Now wait a millisecond until oscillator finished stabilizing and restart PWM.
	delay(1);
	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, restart);
}

/**
 * Helper function to get to register
 */
int pca9685::handler::baseReg(int pin)
{
	return (pin >= PIN_ALL ? LEDALL_ON_L : LED0_ON_L + 4 * pin);
}
