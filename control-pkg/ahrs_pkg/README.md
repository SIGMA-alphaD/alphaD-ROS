ahrs_pkg
===============
imu_bar_pkg로부터 받아온 가속도/자이로 센서값으로부터 오일러각을 계산하는 노드

1. 관련자료
---------------
* 링크로 들어간 뒤 Raw버튼 누르면 자료 다운받아짐
* [Quaternion](https://github.com/dydwo92/alphaD-ROS/blob/master/alphad-ros-pkg/control-pkg/ahrs_pkg/%EC%BF%BC%EB%93%9C%EC%8A%A4%ED%84%B0%EB%94%94_1%EC%B0%A8_part1.pptx)
* [AHRS - Attitude Heading Reference System](https://github.com/dydwo92/alphaD-ROS/blob/master/alphad-ros-pkg/control-pkg/ahrs_pkg/%EC%BF%BC%EB%93%9C%EC%8A%A4%ED%84%B0%EB%94%94_1%EC%B0%A8_part2(rev.1).pptx)


2. 주요 소스 코드
--------------
* [ahrs_node.py](https://github.com/dydwo92/alphaD-ROS/blob/master/alphad-ros-pkg/control-pkg/ahrs_pkg/src/ahrs_node_pkg/ahrs_node.py)
