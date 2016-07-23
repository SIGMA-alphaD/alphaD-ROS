/*******************************************************************************
	package name : i2c_master
	node name : i2c_publish
	author : YJ Kim

	i2c bus info :
        | SENSOR  | ADDR | DESCRIPTION
		| pca9685 | 0x40 | 16 channel pwm generator
		| ads1115 | 0x48  | 16-bit I2C ADC 

*******************************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <iostream>

#include "i2c_master/pca9685.h"
#include "i2c_master/ads1115.h"

#include "i2c_master/i2c_control.h"
#include "i2c_master/i2c_info.h"

using namespace std;

void Callback(const i2c_master::i2c_control::ConstPtr& msg);
void timerCallback(const ros::TimerEvent&);

// Global ROS handler
ros::Publisher pub;
ros::Subscriber sub;
// Global sensor handler
pca9685::handler ph; // pwm handler
ads1115::handler ah; // adc handler

int main(int argc, char** argv){
	ros::init(argc,argv,"i2c_run");
	ros::NodeHandle nh;	
	
	sub = nh.subscribe("/i2c_master/control", 1000, Callback);
	pub = nh.advertise<i2c_master::i2c_info>("/i2c_master/info", 10);

	ph.Setup(100); // 100Hz pwm
	ah.Setup(0, 4);  // gain 2/3 , 128sps

	ros::Timer timer = nh.createTimer(ros::Duration(1), timerCallback); // 1Hz publish

	ros::spin();
	
	return 0;
}

void Callback(const i2c_master::i2c_control::ConstPtr& msg){
	int i, duty;

	for(i=0;i<8;i++){
		// map 0.0 ~ 100.0(float) to 0 ~ 4096(int)
		duty = (int)(msg->pwmDuty[i]*40.96);
		ph.AnalogWrite(i+1, duty);
	}
}

void timerCallback(const ros::TimerEvent&){
	i2c_master::i2c_info info;

	info.batteryVolt = ah.AnalogRead(1);
	pub.publish(info);
}
