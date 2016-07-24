cmake_minimum_required(VERSION 2.8.3)

project(i2c_master)

FIND_LIBRARY(WIRINGPI_LIBRARY wiringPi ../wiringPi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp std_msgs
)

add_executable(i2c_run
  src/i2c_master.cpp
  src/pca9685.cpp
  src/ads1115.cpp
)

target_link_libraries(i2c_run
  ${catkin_LIBRARIES}
  ${WIRINGPI_LIBRARY}
)
