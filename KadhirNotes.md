# Kadhir's Notes

This README contains a dump of all the notes that Kadhir took down about the Holybro PX4 Vision drone. I've tried to compile my notes as neatly as possible, but I wanted to put all my observations here for the next person.

In `geometric_controller`'s `sitl_trajectory_track_circle.launch`, `Initpos` in the launch file sets the center of the circle.

Replace the code in `geometric_controller` with the code in `mavros_controller_mods`.

`geometric_controller` uses the `/command/trajectory` topic which takes MultiDOFJointTrajectory messages.

The PX4 Vision drone isn't computationally very powerful. It crashes when you run `catkin build` because it tries to run many builds in parallel. Use `catkin build <PACKAGE_NAME> -j 2 -p 2`. -j is max jobs, -p is max packages in parallel. Running it without `<PACKAGE_NAME>` might cause it to take a really long time to finish building, as it would build all packages.

The error message `Could not find connection between 'local_origin_ned' and 'fcu' because they are not a part of the same tree. Tf has two or more unconnected trees` can be ignored.

The Structure Core SDK has some sample apps.

The avoidance service on the drone needs to be disabled. Right now it is set to be off by default. This can be changed using the instructions [here](https://docs.px4.io/master/en/complete_vehicles/px4_vision_kit.html#developing-extending-px4-avoidance).

Found that Hybrid mode is the best depth mode. Different exposure and gain settings may affect the fineness of the depth images.

We set up a boolean to control when geometric_controller is on and off. A bit hacky but it works.

Typical procedure to fly the drone in the Vicon space:
```
# TERMINAL A
# launch QGroundControl

# TERMINAL B
ssh px4vision@192.168.1.244 # password is px4vision
roslaunch controllers offboard_traj.launch # or offboard_depth_traj.launch

# TERMINAL C
ssh px4vision@192.168.1.244 # password is px4vision
roslaunch controllers cmd_line.launch
arm
takeoff 1
traj
```

Change `traj_type` in `self.trajectory_controller.trajectory_callback()` in `advanced_controller.py` to make it take a different type of trajectory. The idea is that `bspline` will be implemented to be an acceptable argument and if traj is run, it will follow a B-Spline.

We had an issue where the drone would shoot up to a higher z when we ran `traj`. We fixed that by playing with `norm_thrust_const`.

`/camera_main/depth/image` is the topic for the depth camera feed.