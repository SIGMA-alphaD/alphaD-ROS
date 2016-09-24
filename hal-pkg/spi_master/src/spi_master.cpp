#include <ros/ros.h>

#include <wiringPiSPI.h>
#include <wiringPi.h>

#include "spi_master/mpu9250.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

using namespace std;

int main(int argc, char** argv){
	
	int i;

	ros::init(argc,argv,"spi_run");
	ros::NodeHandle nh;
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("info/imu/data_raw", 100);
	//ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 100);

	ros::Rate loop_rate(400); //400Hz

	mpu9250::handler mh; // mpu9250 handler
	mh.Setup();

	float* mpu_data;

	while(ros::ok()){
	sensor_msgs::Imu Imu;
	sensor_msgs::MagneticField Mag;

	// Read data
	mpu_data = mh.Read();
	
	Imu.linear_acceleration.x = 9.8f * mpu_data[0];
	Imu.linear_acceleration.y = 9.8f * mpu_data[1];
	Imu.linear_acceleration.z = 9.8f * mpu_data[2];
	
	Imu.angular_velocity.x = mpu_data[3];
	Imu.angular_velocity.y = mpu_data[4];
	Imu.angular_velocity.z = mpu_data[5];

	Imu.header.stamp.nsec = millis();

	//Mag.magnetic_field.x = mpu_data[6];
	//Mag.magnetic_field.y = mpu_data[7];
	//Mag.magnetic_field.z = mpu_data[8];

	imu_pub.publish(Imu);	
	//mag_pub.publish(Mag);

	ros::spinOnce();
	loop_rate.sleep();
	}
	
	return 0;
}
