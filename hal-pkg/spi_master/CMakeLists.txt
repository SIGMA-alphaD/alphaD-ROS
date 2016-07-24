cmake_minimum_required(VERSION 2.8.3)

project(spi_master)

FIND_LIBRARY(WIRINGPI_LIBRARY wiringPi ../wiringPi)

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

add_executable(spi_run
  src/spi_master.cpp
  src/mpu9250.cpp
)

target_link_libraries(spi_run
  ${catkin_LIBRARIES}
  ${WIRINGPI_LIBRARY}
)
