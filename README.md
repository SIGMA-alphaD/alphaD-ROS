alphaD ROS Packages
==================

1. 구성
------------------
### 1.1. [hal-pkg](https://github.com/SIGMA-alphaD/alphaD-ROS/tree/master/hal-pkg)
- - -
센서값을 받아오는데 관련된 패키지들
* [i2c_master](https://github.com/SIGMA-alphaD/alphaD-ROS/tree/master/hal-pkg/i2c_master)
  * 쉴드에 i2c로 연결되어 있는 pca9685(16-channel pwm generator)과 ads1115(16bit ADC module)을 컨트롤 하는 모듈
  * **Publishing Message**
    * /info/battery
    * type : std_msgs/Float32
    * description : 현재 배터리의 전압을 나타낸다.
  * **Subscribing Message**
    * /control/pwm
    * type : std_msgs/Float32MultiArray
    * description : 해당 토픽으로 0~100 까지의 실수행렬(길이는 8)을 보내면 각각에 해당하는 채널에 pwm이 generate 된다.
    * example : [20.0, 20.0, 20.0, 20.0, 100.0, 0, 0, 0]

* [spi_master](https://github.com/SIGMA-alphaD/alphaD-ROS/tree/master/hal-pkg/spi_master)
  * 쉴드에 spi로 연결되어 있는 mpu9250과 통신하는 모듈
  * **Publishing Message**
    * /info/imu/data_raw
    * type : sensor_msgs/Imu
    * description : mpu9250으로 받은 가속도(m/s)와 각속도(rad/s)의 값을 퍼블리시 한다.

### 1.2. [control-pkg](https://github.com/SIGMA-alphaD/alphaD-ROS/tree/master/control-pkg)
- - -
로봇 컨트롤에 관련된 패키지들
* [imu_filter](https://github.com/SIGMA-alphaD/alphaD-ROS/tree/master/control-pkg/imu_filter)
  * spi_master 노드를 통해 퍼블리시 된 /info/imu/data_raw를 subscribe하여 필터링된 quaternion을 publish 한다.
  * **Publishing Message**
    * /info/imu
    * type : sensor_msgs/Imu
    * description : madgwick 알고리즘을 이용해 필터링된 quaternion값을 퍼블리시 한다.

### 1.3. [util-pkg](https://github.com/SIGMA-alphaD/alphaD-ROS/tree/master/util-pkg)
- - -
각종 유용한 유틸리티가 들어있는 파일이다.
* [Simple_web_controller](https://github.com/SIGMA-alphaD/alphaD-ROS/tree/master/util-pkg/Simple_web_controller)
