cmake_minimum_required(VERSION 2.8.3)
project(imu_filter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS bin/imu_filter_run
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
