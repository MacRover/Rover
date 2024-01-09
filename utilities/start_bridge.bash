#!/bin/bash

# source ROS 1
source /opt/ros/melodic/setup.bash
source /home/mmrt/Rover/ROS_WS/devel/setup.bash

# start roscore and start rosserial connection to DMC
echo "Starting roscore ..."
screen -d -m -S "roscore" roscore
echo "Waiting to start rosserial ..."
sleep 5
echo "Starting rosserial ..."
screen -d -m -S "rosserial" rosrun rosserial_python serial_node.py _port:=/dev/ttyS0 _baud:=128000

# source ROS 2
source /home/mmrt/ros2_foxy/install/setup.bash
echo "Starting ros bridge"
screen -d -m -S "ros1_bridge" ros2 run ros1_bridge dynamic_bridge
echo "----DEBUG----"
screen -ls
echo "-------------"
echo "DONE!"
