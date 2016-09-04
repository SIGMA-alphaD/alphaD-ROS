#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdint.h>

#include "spi_master/mpu9250.h"

mpu9250::handler::handler(){
	fd = 0;
}

#define MPU_InitRegNum 17
int mpu9250::handler::Setup(void){
    calibrate(g_bias, a_bias);
    int i;

    unsigned char MPU_Init_Data[MPU_InitRegNum][2] ={
        {BIT_H_RESET, MPUREG_PWR_MGMT_1},     // Reset Device
        {0x01, MPUREG_PWR_MGMT_1},     // Clock Source
        {0x00, MPUREG_PWR_MGMT_2},     // Enable Acc & Gyro
        {BITS_DLPF_CFG_188HZ, MPUREG_CONFIG},         // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
        {BITS_FS_250DPS, MPUREG_GYRO_CONFIG},    // +-250dps
        {BITS_FS_2G, MPUREG_ACCEL_CONFIG},   // +-2G
        {BITS_DLPF_CFG_188HZ, MPUREG_ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
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
    data = (float)temp/acc_divider;
    mpu9250_data[0] = data - a_bias[0];	

    temp = ((short)buffer[3] << 8) | buffer[4];
    data = (float)temp/acc_divider;
    mpu9250_data[1] = data - a_bias[1];
    
    temp = ((short)buffer[5] << 8) | buffer[6];
    data = (float)temp/acc_divider;
    mpu9250_data[2] = data - a_bias[2];   

	// Gyroscope Measurements (+-1000dps)
    mpu9250Reads(0x43, buffer, 6);
    temp = ((short)buffer[1] << 8) | buffer[2];
    data = ((float)temp/gyro_divider - g_bias[0])*3.14/180;
    mpu9250_data[3] = data;

    temp = ((short)buffer[3] << 8) | buffer[4];
    data = ((float)temp/gyro_divider - g_bias[1])*3.14/180;
    mpu9250_data[4] = data;
    
    temp = ((short)buffer[5] << 8) | buffer[6];
    data = ((float)temp/gyro_divider - g_bias[2])*3.14/180;	
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

void mpu9250::handler::calibrate(float *dest1, float *dest2){  
    uint8_t data[20]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
    // reset device
    mpu9250Write(0x80, MPUREG_PWR_MGMT_1); // Write a one to bit 7 reset bit; toggle reset device
    delay(100);
   
    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
    // else use the internal oscillator, bits 2:0 = 001
    mpu9250Write(0x01, MPUREG_PWR_MGMT_1);  
    mpu9250Write(0x00, MPUREG_PWR_MGMT_2);
    delay(200);                                    

    // Configure device for bias calculation
    mpu9250Write(0x00, MPUREG_INT_ENABLE);   // Disable all interrupts
    mpu9250Write(0x00, MPUREG_FIFO_EN);      // Disable FIFO
    mpu9250Write(0x00, MPUREG_PWR_MGMT_1);   // Turn on internal clock source
    mpu9250Write(0x00, MPUREG_I2C_MST_CTRL); // Disable I2C master
    mpu9250Write(0x00, MPUREG_USER_CTRL);    // Disable FIFO and I2C master modes
    mpu9250Write(0x0C, MPUREG_USER_CTRL);    // Reset FIFO and DMP
    delay(15);
  
    // Configure MPU6050 gyro and accelerometer for bias calculation
    mpu9250Write(0x01, MPUREG_CONFIG);      // Set low-pass filter to 188 Hz
    mpu9250Write(0x00, MPUREG_SMPLRT_DIV);  // Set sample rate to 1 kHz
    mpu9250Write(0x00, MPUREG_GYRO_CONFIG);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    mpu9250Write(0x00, MPUREG_ACCEL_CONFIG); // Set accelerometer full-scale to 2 g, maximum sensitivity
    
    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g
    
      // Configure FIFO to capture accelerometer and gyro data for bias calculation
    mpu9250Write(0x40, MPUREG_USER_CTRL);   // Enable FIFO  
    mpu9250Write(0x78, MPUREG_FIFO_EN);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    mpu9250Write(0x00, MPUREG_FIFO_EN);        // Disable gyro and accelerometer sensors for FIFO
    mpu9250Reads(MPUREG_FIFO_COUNTH, data, 2); // read FIFO sample count
    fifo_count = ((uint16_t)data[1] << 8) | data[2];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
    
    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        mpu9250Reads(MPUREG_FIFO_R_W, data, 12); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[2]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[4]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[6]  ) ;    
        gyro_temp[0]  = (int16_t) (((int16_t)data[7] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[9] << 8) | data[10]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[11] << 8) | data[12]) ;
        
        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
            
    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accelsensitivity;}
   
    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
    // Push gyro biases to hardware registers
    mpu9250Write(data[0], MPUREG_XG_OFFS_USRH);
    mpu9250Write(data[1], MPUREG_XG_OFFS_USRL);
    mpu9250Write(data[2], MPUREG_YG_OFFS_USRH);
    mpu9250Write(data[3], MPUREG_YG_OFFS_USRL);
    mpu9250Write(data[4], MPUREG_ZG_OFFS_USRH);
    mpu9250Write(data[5], MPUREG_ZG_OFFS_USRL);
  
    // Output scaled gyro biases for display in the main program
    dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
    dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    mpu9250Reads(MPUREG_XA_OFFSET_H, data, 2); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t) (((int16_t)data[1] << 8) | data[2]);
    mpu9250Reads(MPUREG_YA_OFFSET_H, data, 2);
    accel_bias_reg[1] = (int32_t) (((int16_t)data[1] << 8) | data[2]);
    mpu9250Reads(MPUREG_ZA_OFFSET_H, data, 2);
    accel_bias_reg[2] = (int32_t) (((int16_t)data[1] << 8) | data[2]);
    
    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
    
    for(ii = 0; ii < 3; ii++) {
      if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }
    
    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);
  
    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
 
// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
    mpu9250Write(data[0], MPUREG_XA_OFFSET_H);
    mpu9250Write(data[1], MPUREG_XA_OFFSET_L);
    mpu9250Write(data[2], MPUREG_YA_OFFSET_H);
    mpu9250Write(data[3], MPUREG_YA_OFFSET_L);
    mpu9250Write(data[4], MPUREG_ZA_OFFSET_H);
    mpu9250Write(data[5], MPUREG_ZA_OFFSET_L);

// Output scaled accelerometer biases for display in the main program
    dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
    dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
    dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}
