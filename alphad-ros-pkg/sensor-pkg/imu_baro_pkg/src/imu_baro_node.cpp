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

    ros::Rate loop_rate(100); // 100Hz
    mpu9250Initialize();

    while(ros::ok()){
    	imu_baro_pkg::msgImuBaro msg;

    	msg.Acc = mpu9250read_acc();
    	msg.Gyro = mpu9250read_gyro();
        msg.msTime = millis();
        chatter_pub.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
