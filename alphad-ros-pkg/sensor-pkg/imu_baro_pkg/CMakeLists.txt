cmake_minimum_required(VERSION 2.8.3)

project(imu_baro_pkg)

FIND_LIBRARY(WIRINGPI_LIBRARY wiringPi ../wiringPi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(FILES
  vector.msg
  msgImuBaro.msg
)

generate_messages(DEPENDENCIES
  	std_msgs
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_baro_node
  src/imu_baro_node.cpp
  src/mpu9250.cpp
)

add_dependencies(imu_baro_node
	wiringPi
	${imu_baro_pkg_EXPORTED_TARGETS}
)
target_link_libraries(imu_baro_node
  ${catkin_LIBRARIES}
  ${WIRINGPI_LIBRARY}
)
