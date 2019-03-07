#!/bin/bash

echo ""
echo "RUNNING agv_serial_server!"

echo ""
echo "Getting IO permission..."
sudo chmod 777 /dev/ttyUSB0

echo ""
echo "Refresh ROS environment..."
rospack profile&&

source devel/setup.bash&&

echo ""
echo "make..."
catkin_make

echo ""
echo "Start ROS node..."


rosrun agv_serial_server agv_serial_server




