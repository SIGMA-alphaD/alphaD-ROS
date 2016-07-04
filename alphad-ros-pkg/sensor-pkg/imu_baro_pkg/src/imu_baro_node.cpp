#include <ros/ros.h>
#include <std_msgs/String.h>
#include <wiringPiSPI.h>
#include <wiringPi.h>

#include <iostream>
#include "imu_baro_pkg/mpu9250.h"

#include "imu_baro_pkg/msgImuBaro.h"

using namespace std;


int main(int argc, char** argv){
    
    wiringPiSetupSys();

    ros::init(argc,argv,"imu_baro_node");
    ros::NodeHandle nh;
    ros::Publisher chatter_pub = nh.advertise<imu_baro_pkg::msgImuBaro>("imu_baro_msg",100);

    ros::Rate loop_rate(10); // 10Hz
    mpu9250Initialize();

    unsigned char buffer[2];
    mpu9250Write(AK8963_I2C_ADDR | 0x80, MPUREG_I2C_SLV0_ADDR);
    mpu9250Write(0x00, MPUREG_I2C_SLV0_REG);
    mpu9250Write(0x81, MPUREG_I2C_SLV0_CTRL);

	delayMicroseconds(10000);

	cout << hex << (int)mpu9250Read(MPUREG_EXT_SENS_DATA_00) << endl;


    while(ros::ok()){
    	imu_baro_pkg::msgImuBaro msg;

    	msg.Acc = mpu9250read_acc();
    	msg.Gyro = mpu9250read_gyro();
        msg.msTime = millis();
        chatter_pub.publish(msg);
    }

    return 0;
}
