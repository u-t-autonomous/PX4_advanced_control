#!/bin/bash
roslaunch vrpn_client_ros sample.launch server:=192.168.1.100 &
sleep 10
roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600 gcs_url:=udp://@192.168.1.101:14550 &
rosrun topic_tools relay /vrpn_client_node/PX4_Vision/pose /mavros/vision_pose/pose &
rosrun topic_tools relay /vrpn_client_node/PX4_Vision/twist /mavros/vision_speed/speed_twist
