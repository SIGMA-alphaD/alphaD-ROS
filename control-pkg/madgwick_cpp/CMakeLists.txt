cmake_minimum_required(VERSION 2.8.3)

project(imu_filter)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp sensor_msgs
)

add_executable(i2c_run
  src/MadgwickAHRS.cpp
)

target_link_libraries(i2c_run
  ${catkin_LIBRARIES}
  ${WIRINGPI_LIBRARY}
)
