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
    * 사용자가 직접 데이터 보내는 방법 : > rostopic pub /control/pwm std_msgs/Float32MultiArray "{data:[20.0, 20.0, 20.0, 20.0, 100.0, 0, 0, 0]}" (수정바람)

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
    * /info/imu/data
    * type : sensor_msgs/Imu
    * description : madgwick 알고리즘을 이용해 필터링된 quaternion값을 퍼블리시 한다.

* [chung_control](https://github.com/SIGMA-alphaD/alphaD-ROS/tree/master/control-pkg/chung_control)
  * imu_filter 노드에서 퍼블리시하는 quaternion값을 받아서 연산후에 i2c_master의 /control/pwm으로 퍼블리시한다.
  * 앞에 chung-을 붙인 이유는, 여러가지 컨트롤 방법을 구분하기 위함이다.
  * **Subscribing Message**
    * /info/imu/data
    * type : sensor_msgs/Imu
    * description : quaternion Array (w,x,y,z)를 받는다. (쿼터니언 순서 수정바람)
  * **Publishing Message**
    * /control/pwm
    * type : std_msgs/Float32MultiArray
    * description : quaternion을 기반으로 계산한 pwm출력 값을 퍼블리시 한다.

  * **[Control Policy]**
    * z축(연직방향)의 움직임을 기준으로 판단함.
    * [Ver. 0.2.0] 시간 t에서의 z축 벡터를 z(t)라 하면, z(t)를 x(0)와 y(0)가 이루는 평면에 정사영시킨 벡터 v(t)와 z(0)와 z(t)의 각도 theta를 기반으로 v(t)는 4개의 pwm신호의 가감(+ -)을, theta는 P 제어에 이용된다.  즉, 이 Version은 단순 P제어 방식이다.
    * 다음 버전의 기술 바람.

### 1.3. [util-pkg](https://github.com/SIGMA-alphaD/alphaD-ROS/tree/master/util-pkg)
- - -
각종 유용한 유틸리티가 들어있는 파일이다.
* [Simple_web_controller](https://github.com/SIGMA-alphaD/alphaD-ROS/tree/master/util-pkg/Simple_web_controller)
