cmake_minimum_required(VERSION 2.8.3)

project(i2c_master)

FIND_LIBRARY(WIRINGPI_LIBRARY wiringPi ../wiringPi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(FILES
	i2c_info.msg
	i2c_control.msg
)

generate_messages(DEPENDENCIES
  	std_msgs
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp std_msgs message_runtime
)

add_executable(i2c_run
  src/i2c_master.cpp
  src/pca9685.cpp
  src/ads1115.cpp
)

add_dependencies(i2c_run
	wiringPi
	${i2c_master_EXPORTED_TARGETS}
)

target_link_libraries(i2c_run
  ${catkin_LIBRARIES}
  ${WIRINGPI_LIBRARY}
)
