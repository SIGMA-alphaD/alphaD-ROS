#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "spi_master/mpu9250.h"

mpu9250::handler::handler(){
	fd = 0;
}

#define MPU_InitRegNum 14
int mpu9250::handler::Setup(void){
    int i;

    unsigned char MPU_Init_Data[MPU_InitRegNum][2] ={
        //{0x80, MPUREG_PWR_MGMT_1}, // Reset
        {0x01, MPUREG_PWR_MGMT_1}, // Auto select clock source
        {0x00, MPUREG_PWR_MGMT_2}, // Acc & Gyro enable
        {0x16, MPUREG_GYRO_CONFIG}, // +-1000dps
        {0x00, MPUREG_ACCEL_CONFIG}, // +-2G
        {0x30, MPUREG_INT_PIN_CFG},
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
        {0x12, MPUREG_I2C_SLV0_DO}, // Register value to continuous measurement in 16bit
        {0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte
    };

	wiringPiSetupSys();

    fd = wiringPiSPISetup(MPU_CHANNEL, 500000);
	
	if(fd < 0)
		return 0;

    for(i=0;i<MPU_InitRegNum;i++){
        mpu9250Write(MPU_Init_Data[i][0], MPU_Init_Data[i][1]);
		delayMicroseconds(10000);
    }
	
	return fd;
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

	// Accelerometer Measurements (Right hand coordinate)
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

	// Gyroscope Measurements (Right hand coordinate, +-2000dps)
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
