#!/bin/bash

# start roscore and start rosserial connection to DMC
echo "Starting roscore ..."
screen -d -m -S "roscore" roscore
echo "Waiting to start rosserial ..."
sleep 5
echo "Starting rosserial ..."
screen -d -m -S "rosserial" rosrun rosserial_python serial_node.py _port:=/dev/ttyS0 _baud:=128000
echo "----DEBUG----"
screen -ls
echo "-------------"
echo "DONE!"