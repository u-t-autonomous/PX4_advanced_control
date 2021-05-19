# PX4_advanced_control

Repo consists of a base controller and advanced controllers for PX4-based drones. The advanced controllers extend the base controller. To implement an advanced controller, use `advanced_controller` and `advanced_cmd_line`. 

The code is set up to switch between base controller and advanced controller during flight. Switching happens either manually or automatically. To manually switch, the user can use pre-defined commands in `base_cmd_line` and `advanced_cmd_line`. The code automatically switches out of the advanced controller if the drone becomes unstable (see `check_security` method).

## Setup

To install the required packages, follow the ROS_Gazebo_PX4_Setup guide. Some information may be outdated, so check the PX4 documentation for updates. The following package versions have been successfully tested:

* PX4 - 1.9.2 on Ubuntu 18.4 / 1.9.0dev on Hexacopter
* Mavros - 0.32.1 on Ubuntu 18.4 / 1.3.0 on Odroid
* Mavlink - origin/debian/kinetic/mavlink (we are running ROS melodic, but the kinetic branch is the right version for Mavlink) / 1.0 (!?) on Odroid

To see which versions you are running, `roscd <pkg>` and run `git branch -a`. If the git head is not pointing to the same version as the one above, use `git checkout tags/v<x.xx.x>` to checkout the right version.


## Run

Use `sitl_gazebo.launch` to  test the controller in Gazebo. A seperate terminal will launch, which we use to command the drone. The specified commands are found in the `base_cmd_line` and `advanced_cmd_line` files. 

For experiments, we first launch `offboard.launch` to initialize the communication with the vehicle and then launch `cmd_line.launch` in a seperate termianl. The launch files are seperated, because we want to use two different terminals when ssh'ing to the drone's onboard computer.

## Running `geometric_controller`

Follow the instructions on the [mavros_controllers repo](https://github.com/Jaeyoung-Lim/mavros_controllers) to set it up, then copy the files from the `/mavros_controllers_mods` folder and overwrite the existing versions of those files in `mavros_controllers`, and rebuild your workspace.

To run this code in simulation, modify your `.bashrc` to include the following:

```
# SIM
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
```

To run this code on a drone, modify your `.bashrc` to include the following:

```
# HARDWARE
export ROS_MASTER_URI=http://192.168.1.244:11311 # replace this with the IP of your drone
export ROS_IP=192.168.1.150 # replace this with the IP of the laptop from which you will be launching things
```

Change line 5 in `/scripts/offboard_setup.sh` to contain your laptop's IP, and make sure the topics on the following two lines are set to the correct values.

To launch everything:
```
# TERMINAL A
# launch QGroundControl
```
```
# TERMINAL B
ssh px4vision@192.168.1.244 # password is px4vision
roslaunch controllers offboard_traj.launch # or offboard_depth_traj.launch
```
```
# TERMINAL C
ssh px4vision@192.168.1.244 # password is px4vision
roslaunch controllers cmd_line.launch
```

<hr>

### If the drone starts beeping:
If the drone starts beeping, turn on the remote control and push the left stick to the bottom right. This should arm the drone. Then push the left stick to the bottom left to disarm it. If this doesn't work, connect to the Pixhawk4 WiFi on a laptop (password is `pixhawk4`), launch QGroundControl, go to Vehicle Setup, Parameters, Tools (top right), Reboot Vehicle.

### Notes on Depth Camera:
The depth camera has some dynamic parameters. One way to access these parameters is to run `rosrun rqt_reconfigure rqt_reconfigure` (I'm not sure if this method shows every single possible parameter). These parameters can be modified using `dynparam`. An example on how to do this can be seen in `rqt_param_toggle.sh` on the drone.

> All of Kadhir's notes can be seen in `KadhirNotes.md`

<!-- Setting up new drone  -->