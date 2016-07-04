#include <wiringPiSPI.h>
#include <wiringPi.h>

#include "imu_baro_pkg/mpu9250.h"
#include "imu_baro_pkg/vector.h"

void mpu9250Initialize(void){
    int i;

    unsigned char MPU_Init_Data[MPU_InitRegNum][2] ={
        {0x80, MPUREG_PWR_MGMT_1}, // Reset
        {0x01, MPUREG_PWR_MGMT_1}, // Auto select clock source
        {0x00, MPUREG_PWR_MGMT_2}, // Acc & Gyro enable
        {0x18, MPUREG_GYRO_CONFIG}, // +-2000dps
        {0x08, MPUREG_ACCEL_CONFIG}, // +-4G
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

    wiringPiSPISetup(MPU_CHANNEL, 500000);

    for(i=0;i<MPU_InitRegNum;i++){
        mpu9250Write(MPU_Init_Data[i][0], MPU_Init_Data[i][1]);
    }

}

unsigned char mpu9250Read(unsigned char regNum){
    unsigned char buffer[2];

    buffer[0] = regNum | 0x80;
    buffer[1] = 0x00;
    wiringPiSPIDataRW(MPU_CHANNEL, buffer, 2);
    delayMicroseconds(10000);

    return buffer[1];
}

void mpu9250Reads(unsigned char regNum, unsigned char* readBuf, int Bytes){
    readBuf[0] = regNum | 0x80;
    wiringPiSPIDataRW(MPU_CHANNEL, readBuf, Bytes + 1);
    delayMicroseconds(10000);
}

void mpu9250Write(unsigned char value, unsigned char regNum){
    unsigned char buffer[2];

    buffer[0] = regNum;
    buffer[1] = value;

    wiringPiSPIDataRW(MPU_CHANNEL, buffer, 2);
    delayMicroseconds(10000);
}

imu_baro_pkg::vector mpu9250read_acc(){
    unsigned char buffer[7];
    short temp;
    float data;
    imu_baro_pkg::vector vector;

    // 우수 좌표계
    mpu9250Reads(0x3B, buffer, 6);
    temp = ((short)buffer[1] << 8) | buffer[2];
    data = (float)temp/8192*9.81;
    vector.x = -data;

    temp = ((short)buffer[3] << 8) | buffer[4];
    data = (float)temp/8192*9.81;
    vector.y = data;
    
    temp = ((short)buffer[5] << 8) | buffer[6];
    data = (float)temp/8192*9.81;
    vector.z = data;   

    return vector;
}

imu_baro_pkg::vector mpu9250read_mag(){
    unsigned char buffer[7];
    short temp;
    imu_baro_pkg::vector vector;

    mpu9250Write(AK8963_I2C_ADDR | 0x80, MPUREG_I2C_SLV0_ADDR); //Set the I2C slave addres of AK8963 and set for read.
    mpu9250Write(AK8963_HXL, MPUREG_I2C_SLV0_REG); //I2C slave 0 register address from where to begin data transfer
    mpu9250Write(0x86, MPUREG_I2C_SLV0_CTRL); //Read 6 bytes from the magnetometer

    delayMicroseconds(10000);

    mpu9250Reads(MPUREG_EXT_SENS_DATA_00, buffer, 6);
    temp = ((short)buffer[1] << 8) | buffer[2];
    vector.x = (float)temp;

    temp = ((short)buffer[3] << 8) | buffer[4];
    vector.y = (float)temp;
    
    temp = ((short)buffer[5] << 8) | buffer[6];
    vector.z = (float)temp;

    return vector;    
}

imu_baro_pkg::vector mpu9250read_gyro(){
    unsigned char buffer[7];
    short temp;
    float data;
    imu_baro_pkg::vector vector;

    // 우수 좌표계

    // +-2000dps
    mpu9250Reads(0x43, buffer, 6);
    temp = ((short)buffer[1] << 8) | buffer[2];
    data = (float)temp/16.384*3.14/180;
    vector.x = data;

    temp = ((short)buffer[3] << 8) | buffer[4];
    data = (float)temp/16.384*3.14/180;
    vector.y = -data;
    
    temp = ((short)buffer[5] << 8) | buffer[6];
    data = (float)temp/16.384*3.14/180;
    vector.z = -data;   

    return vector;   
}