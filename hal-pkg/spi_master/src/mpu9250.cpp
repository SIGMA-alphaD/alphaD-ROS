#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "spi_master/mpu9250.h"

mpu9250::handler::handler(){
	fd = 0;
}

#define MPU_InitRegNum 17
int mpu9250::handler::Setup(void){
    int i;

    unsigned char MPU_Init_Data[MPU_InitRegNum][2] ={
        {BIT_H_RESET, MPUREG_PWR_MGMT_1},     // Reset Device
        {0x01, MPUREG_PWR_MGMT_1},     // Clock Source
        {0x00, MPUREG_PWR_MGMT_2},     // Enable Acc & Gyro
        {my_low_pass_filter, MPUREG_CONFIG},         // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
        {BITS_FS_250DPS, MPUREG_GYRO_CONFIG},    // +-250dps
        {BITS_FS_2G, MPUREG_ACCEL_CONFIG},   // +-2G
        {my_low_pass_filter_acc, MPUREG_ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
        {0x30, MPUREG_INT_PIN_CFG},    //
        //{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
        //{0x20, MPUREG_USER_CTRL},      // Enable AUX
        {0x20, MPUREG_USER_CTRL},       // I2C Master mode
        {0x0D, MPUREG_I2C_MST_CTRL}, //  I2C configuration multi-master  IIC 400KHz
        
        {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  //Set the I2C slave addres of AK8963 and set for write.
        //{0x09, MPUREG_I2C_SLV4_CTRL},
        //{0x81, MPUREG_I2C_MST_DELAY_CTRL}, //Enable I2C delay

        {AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
        {0x01, MPUREG_I2C_SLV0_DO}, // Reset AK8963
        {0x81, MPUREG_I2C_SLV0_CTRL},  //Enable I2C and set 1 byte

        {AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
#ifdef AK8963FASTMODE
        {0x16, MPUREG_I2C_SLV0_DO}, // Register value to 100Hz continuous measurement in 16bit
#else
        {0x12, MPUREG_I2C_SLV0_DO}, // Register value to 8Hz continuous measurement in 16bit
#endif
        {0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte
    };

	wiringPiSetupSys();

    fd = wiringPiSPISetup(MPU_CHANNEL, 500000);
	
	if(fd < 0)
		return 0;

    for(i=0;i<MPU_InitRegNum;i++){
        mpu9250Write(MPU_Init_Data[i][0], MPU_Init_Data[i][1]);
		delayMicroseconds(1000);
    }
	
        set_acc_scale(BITS_FS_2G);
    	set_gyro_scale(BITS_FS_1000DPS);
	return fd;
}

/*                                ACCELEROMETER SCALE
 * usage: call this function at startup, after initialization, to set the right range for the
 * accelerometers. Suitable ranges are:
 * BITS_FS_2G
 * BITS_FS_4G
 * BITS_FS_8G
 * BITS_FS_16G
 * returns the range set (2,4,8 or 16)
 */

unsigned int mpu9250::handler::set_acc_scale(int scale){
    unsigned int temp_scale;
    mpu9250Write(scale, MPUREG_ACCEL_CONFIG);
    
    switch (scale){
        case BITS_FS_2G:
            acc_divider=16384;
        break;
        case BITS_FS_4G:
            acc_divider=8192;
        break;
        case BITS_FS_8G:
            acc_divider=4096;
        break;
        case BITS_FS_16G:
            acc_divider=2048;
        break;   
    }
    temp_scale = mpu9250Read(MPUREG_ACCEL_CONFIG);
    
    switch (temp_scale){
        case BITS_FS_2G:
            temp_scale=2;
        break;
        case BITS_FS_4G:
            temp_scale=4;
        break;
        case BITS_FS_8G:
            temp_scale=8;
        break;
        case BITS_FS_16G:
            temp_scale=16;
        break;   
    }
    return temp_scale;
}

/*                                 GYROSCOPE SCALE
 * usage: call this function at startup, after initialization, to set the right range for the
 * gyroscopes. Suitable ranges are:
 * BITS_FS_250DPS
 * BITS_FS_500DPS
 * BITS_FS_1000DPS
 * BITS_FS_2000DPS
 * returns the range set (250,500,1000 or 2000)
 */

unsigned int mpu9250::handler::set_gyro_scale(int scale){
    unsigned int temp_scale;
    mpu9250Write(scale, MPUREG_GYRO_CONFIG);

    switch (scale){
        case BITS_FS_250DPS:   gyro_divider = 131;  break;
        case BITS_FS_500DPS:   gyro_divider = 65.5; break;
        case BITS_FS_1000DPS:  gyro_divider = 32.8; break;
        case BITS_FS_2000DPS:  gyro_divider = 16.4; break;   
    }

    temp_scale = mpu9250Read(MPUREG_GYRO_CONFIG);

    switch (temp_scale){
        case BITS_FS_250DPS:   temp_scale = 250;    break;
        case BITS_FS_500DPS:   temp_scale = 500;    break;
        case BITS_FS_1000DPS:  temp_scale = 1000;   break;
        case BITS_FS_2000DPS:  temp_scale = 2000;   break;   
    }
    return temp_scale;
}

unsigned char mpu9250::handler::mpu9250Read(unsigned char regNum){
    unsigned char buffer[2];

    buffer[0] = regNum | 0x80;
    buffer[1] = 0x00;
    wiringPiSPIDataRW(MPU_CHANNEL, buffer, 2);
    delayMicroseconds(1000);

    return buffer[1];
}

void mpu9250::handler::mpu9250Reads(unsigned char regNum, unsigned char* readBuf, int Bytes){
    readBuf[0] = regNum | 0x80;
    wiringPiSPIDataRW(MPU_CHANNEL, readBuf, Bytes + 1);
    delayMicroseconds(1000);
}

void mpu9250::handler::mpu9250Write(unsigned char value, unsigned char regNum){
    unsigned char buffer[2];

    buffer[0] = regNum;
    buffer[1] = value;

    wiringPiSPIDataRW(MPU_CHANNEL, buffer, 2);
    delayMicroseconds(1000);
}

float* mpu9250::handler::Read(){
    unsigned char buffer[10];
    short temp;
    float data;

	// Accelerometer Measurements (+-2g)
    mpu9250Reads(0x3B, buffer, 6);
    temp = ((short)buffer[1] << 8) | buffer[2];
    data = (float)temp/16384*9.81;
    mpu9250_data[0] = data;	

    temp = ((short)buffer[3] << 8) | buffer[4];
    data = (float)temp/16384*9.81;
    mpu9250_data[1] = data;
    
    temp = ((short)buffer[5] << 8) | buffer[6];
    data = (float)temp/16384*9.81;
    mpu9250_data[2] = data;   

	// Gyroscope Measurements (+-1000dps)
    mpu9250Reads(0x43, buffer, 6);
    temp = ((short)buffer[1] << 8) | buffer[2];
    data = (float)temp/32.8*3.14/180;
    mpu9250_data[3] = data;

    temp = ((short)buffer[3] << 8) | buffer[4];
    data = (float)temp/32.8*3.14/180;
    mpu9250_data[4] = data;
    
    temp = ((short)buffer[5] << 8) | buffer[6];
    data = (float)temp/32.8*3.14/180;	
	mpu9250_data[5] = data;

/*
	// Magnetometer measurements
    mpu9250Write(AK8963_I2C_ADDR | READ_FLAG, MPUREG_I2C_SLV0_ADDR); //Set the I2C slave addres of AK8963 and set for read.
    mpu9250Write(AK8963_HXL, MPUREG_I2C_SLV0_REG); //I2C slave 0 register address from where to begin data transfer
    mpu9250Write(0x87, MPUREG_I2C_SLV0_CTRL); //Read 6 bytes from the magnetometer

    delayMicroseconds(10000);

    mpu9250Reads(MPUREG_EXT_SENS_DATA_00, buffer, 7);
    temp = ((short)buffer[1] << 8) | buffer[2];
    mpu9250_data[6] = (float)temp;

    temp = ((short)buffer[3] << 8) | buffer[4];
    mpu9250_data[7] = (float)temp;
    
    temp = ((short)buffer[5] << 8) | buffer[6];
    mpu9250_data[8] = (float)temp;
*/
    return mpu9250_data;
}
