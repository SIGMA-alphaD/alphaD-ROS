alphaD ROS Packages
==================

1. 구성
------------------
### 1.1. hal-pkg
- - -
센서값을 받아오는데 관련된 패키지들
* [i2c_master](https://github.com/SIGMA-alphaD/alphaD-ROS/tree/master/hal-pkg/i2c_master)
  * 쉴드에 i2c로 연결되어 있는 pca9685(16-channel pwm generator)과 ads1115(16bit ADC module)을 컨트롤 하는 모듈
  * Publishing Message
   * /info/battery
  * Subscribing Message
   * /control/pwm

* [spi_master](https://github.com/SIGMA-alphaD/alphaD-ROS/tree/master/alphad-ros-pkg/sensor-pkg/px-ros-pkg)
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
