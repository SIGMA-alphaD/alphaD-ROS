//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	512.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;				// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float qqq[4] = {q0, q1, q2, q3};
float acc[3] = {0.0f, 0.0f, 0.0f};
float gyro[3] = {0.0f, 0.0f, 0.0f};

int _prevT = -1;
int _sampleRate = 300;
//---------------------------------------------------------------------------------------------------
// Function declarations

ros::Publisher pub;
ros::Subscriber sub;
sensor_msgs::Imu msg;


float invSqrt(float x);
void Callback(const sensor_msgs::Imu::ConstPtr& data);
void MadgwickAHRSupdate(float* qtn, float* gyro, float* acc, float* mag);
void MadgwickAHRSupdateIMU(float* qtn, float* gyro, float* acc);


#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
