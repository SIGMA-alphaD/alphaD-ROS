cmake_minimum_required(VERSION 2.8.3)
project(chung_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS bin/chung_control_run
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
