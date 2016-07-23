#! /bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

/usr/local/bin/gpio export 27 out

cd ~/catkin_ws/src/alphaD-ROS/Utils/Simple_web_controller
node app &

cd ~/catkin_ws/src/alphaD-ROS
roslaunch myrobot.launch

