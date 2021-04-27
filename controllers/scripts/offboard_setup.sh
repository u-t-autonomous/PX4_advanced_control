#!/bin/bash
roslaunch vrpn_client_ros sample.launch server:=192.168.1.100 &
sleep 10
# Change the udp IP to your laptop's IP
roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600 gcs_url:=udp://@192.168.1.150:14550 &
rosrun topic_tools relay /vrpn_client_node/PX4Vision/pose /mavros/vision_pose/pose &
rosrun topic_tools relay /vrpn_client_node/PX4Vision/twist /mavros/vision_speed/speed_twist
