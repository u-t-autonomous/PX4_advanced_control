#!/usr/bin/env python2

import numpy as np
import rospy

from geometry_msgs.msg import Transform, Twist
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


class TrajectoryController(object):
    def __init__(self):
        # The desired_pos and desired_vel variables are used to set the trajectory message
        self.desired_pos = np.array([[0.0], [0.0], [0.0]])
        self.desired_vel = np.array([[0.0], [0.0], [0.0]])
        self.desired_acc = np.array([[0.0], [0.0], [0.0]])

        self.coordinate_publisher = rospy.Publisher(
            "/command/trajectory", MultiDOFJointTrajectory, queue_size=1
        )

        self.bool_publisher = rospy.Publisher(
            "/ext_traj/bool", Bool, queue_size=1
        )

        # start_time stores the t = 0 for each segment of flight
        self.start_time = 0

    def trajectory_callback(self, traj_type):

        if traj_type == "circle":
            self.follow_circle()

        msg = MultiDOFJointTrajectory()
        msg.points = [MultiDOFJointTrajectoryPoint()]
        msg.points[0].transforms = [Transform()]

        msg.points[0].transforms[0].translation.x = self.desired_pos[0][0]
        msg.points[0].transforms[0].translation.y = self.desired_pos[1][0]
        msg.points[0].transforms[0].translation.z = self.desired_pos[2][0]

        q = quaternion_from_euler(0.0, 0.0, self.calculate_yaw())
        msg.points[0].transforms[0].rotation.x = q[0]
        msg.points[0].transforms[0].rotation.y = q[1]
        msg.points[0].transforms[0].rotation.z = q[2]
        msg.points[0].transforms[0].rotation.w = q[3]

        msg.points[0].velocities = [Twist()]
        msg.points[0].velocities[0].linear.x = self.desired_vel[0][0]
        msg.points[0].velocities[0].linear.y = self.desired_vel[1][0]
        msg.points[0].velocities[0].linear.z = self.desired_vel[2][0]

        msg.points[0].accelerations = [Twist()]
        msg.points[0].accelerations[0].linear.x = self.desired_acc[0][0]
        msg.points[0].accelerations[0].linear.y = self.desired_acc[1][0]
        msg.points[0].accelerations[0].linear.z = self.desired_acc[2][0]

        self.coordinate_publisher.publish(msg)

    def follow_circle(self):
        current_time = rospy.get_time()
        if self.start_time == 0:
            self.start_time = current_time
        time_difference = current_time - self.start_time

        # Functions are evaluated at each time step, and published in a message to the
        # controller
        factor = 0.5
        radius = 2.5
        x_des, y_des = self.calculate_position(time_difference, radius, factor)
        vx_des, vy_des = self.calculate_velocity(time_difference, radius, factor)
        ax_des, ay_des = self.calculate_acceleration(time_difference, radius, factor)

        self.desired_pos[0][0] = x_des
        self.desired_pos[1][0] = y_des
        self.desired_pos[2][0] = 1.5
        self.desired_vel[0][0] = vx_des
        self.desired_vel[1][0] = vy_des
        self.desired_vel[2][0] = 0
        self.desired_acc[0][0] = ax_des
        self.desired_acc[1][0] = ay_des
        self.desired_acc[2][0] = 0

    # In the methods below, x = 3cos(t) and y = 3sin(t) were chosen arbitrarily. Any
    # mathematical functions can be chosen, the only requirement is that the first and
    # second derivatives of the functions must be put in the velocity and acceleration
    # functions respectively
    def calculate_position(self, time, radius, factor):
        x = radius * np.sin(time * factor)
        y = radius * np.cos(time * factor) - radius

        return x, y

    def calculate_velocity(self, time, radius, factor):
        vx = factor * radius * np.cos(time * factor)
        vy = -factor * radius * np.sin(time * factor)

        return vx, vy

    def calculate_acceleration(self, time, radius, factor):
        ax = -(factor ** 2) * radius * np.sin(time * factor)
        ay = -(factor ** 2) * radius * np.cos(time * factor)

        return ax, ay

    def calculate_yaw(self):
        return np.arctan2(self.desired_vel[1][0], self.desired_vel[0][0])

    def publish_bool(self, value):
        bool_msg = Bool()
        bool_msg.data = value
        if not value:
            self.start_time = 0
        self.bool_publisher.publish(bool_msg)