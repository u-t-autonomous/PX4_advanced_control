# PX4_advanced_control

Repo consists of a base controller and advanced controllers for PX4-based drones. The advanced controllers extend the base controller. To implement an advanced controller, use `advanced_controller` and `advanced_cmd_line`. 

The code is set up to switch between base controller and advanced controller during flight. Switching happens either manually or automatically. To manually switch, the user can use pre-defined commands in `base_cmd_line` and `advanced_cmd_line`. The code automatically switches out of the advanced controller if the drone becomes unstable (see `check_security` method).

## Setup

To install the required packages, follow the ROS_Gazebo_PX4_Setup guide. Some information may be outdated, so check the PX4 documentation for updates. The following package versions have been successfully tested:

* PX4 - v1.12.0
* Mavros - 0.32.1 on Ubuntu 18.4 / 1.13.0 on Odroid
* Mavlink - release/kinetic/mavlink/2021.3.3-1 (we are running ROS melodic, but the kinetic branch is the right version for Mavlink)

To see which versions you are running, `roscd <pkg>` and run `git branch -a`. If the git head is not pointing to the same version as the one above, use `git checkout tags/v<x.xx.x>` to checkout the right version.


## Run

Use `sitl_gazebo.launch` to  test the controller in Gazebo. A seperate terminal will launch, which we use to command the drone. The specified commands are found in cmd `the_base_line` and `advanced_cmd_line` files. 

For experiments, we first launch `offboard.launch` to initialize the communication with the vehicle and then launch `cmd_line.launch` in a seperate termianl. The launch files are seperated, because we want to use two different terminals when ssh'ing to the drone's onboard computer.
