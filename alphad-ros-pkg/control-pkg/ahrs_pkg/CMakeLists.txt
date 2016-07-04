cmake_minimum_required(VERSION 2.8.3)
project(ahrs_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(FILES
  msgAhrs.msg
)

generate_messages(DEPENDENCIES
  	std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS bin/ahrs_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})