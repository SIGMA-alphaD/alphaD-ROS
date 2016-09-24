cmake_minimum_required(VERSION 2.8.3)

project(madgwick_cpp)

FIND_LIBRARY(MADGWICK_CPP_LIBRARY madgwick_cpp ./include/madgwick_cpp)

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

add_executable(madgwick_run
  src/madgwick_cpp.cpp
)

target_link_libraries(madgwick_run
  ${catkin_LIBRARIES}
  ${WIRINGPI_LIBRARY}
)
