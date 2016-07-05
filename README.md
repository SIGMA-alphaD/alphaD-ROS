alphaD ROS Packages
==================

1. 구성
------------------
### 1.1. sensor-pkg
- - -
센서값을 받아오는데 관련된 패키지들
* [imu_baro_pkg](https://github.com/SIGMA-alphaD/alphaD-ROS/tree/master/alphad-ros-pkg/sensor-pkg/imu_baro_pkg)
  * 쉴드에 부착되어 있는 MPU9250(IMU) 과 MS5611(Barometer) 로부터 값을 받아오는 노드

* [px-ros-pkg](https://github.com/SIGMA-alphaD/alphaD-ROS/tree/master/alphad-ros-pkg/sensor-pkg/px-ros-pkg)
  * px4flow 옵티컬 플로우 센서로부터 값을 받아오는 노드

### 1.2. control-pkg
- - -
로봇 컨트롤에 관련된 패키지들
* [ahrs_pkg](https://github.com/SIGMA-alphaD/alphaD-ROS/tree/master/alphad-ros-pkg/control-pkg/ahrs_pkg)
  * IMU 데이터를 통해 오일러 각을 추정하는 노드
  
### 1.3. www-pkg
- - -
웹 어플리케이션 관련 패키지들
* none
