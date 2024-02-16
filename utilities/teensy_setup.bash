#!/bin/bash

set -e

# Source ROS2
source /home/mmrt/ros2_foxy/install/setup.bash
source /home/mmrt/mros_ws/install/setup.bash

screen -d -m -S microros ros2 run micro_ros_agent micro_ros_agent udp4 --port 9999
echo "Started microROS"







