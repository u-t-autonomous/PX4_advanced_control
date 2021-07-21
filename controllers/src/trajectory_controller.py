#!/usr/bin/env python2

import numpy as np
import rospy
import scipy.interpolate

from controllers.msg import Bspline
from geometry_msgs.msg import Transform, Twist
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


MIN_Z = 0.8
MAX_Z = 2.0
MAX_VEL = 1.0
MAX_ACC = 2.0
MAX_JERK = 4.0
MAX_ANG_VEL = np.pi / 12


class TrajectoryController(object):
    def __init__(self):
        # The desired_pos and desired_vel variables are used to set the trajectory message
        self.desired_pos = np.array([[0.0], [0.0], [0.0]])
        self.desired_vel = np.array([[0.0], [0.0], [0.0]])
        self.desired_acc = np.array([[0.0], [0.0], [0.0]])

        self.coordinate_publisher = rospy.Publisher(
            "/command/trajectory", MultiDOFJointTrajectory, queue_size=1
        )

        self.bool_publisher = rospy.Publisher("/ext_traj/bool", Bool, queue_size=1)

        self.bspline_subscriber = rospy.Subscriber(
            name="/planning/bspline",
            data_class=Bspline,
            callback=self.bspline_callback,
            queue_size=1,
        )

        # start_time stores the t = 0 for each segment of flight
        self.start_time = 0
        self.bspline_start_time = None
        self.in_hover = False

        self.yaw = 0
        self.prev_eval_t = rospy.get_time()

    def trajectory_callback(self, traj_type, curr_pos):
        self.eval_t = rospy.get_time()
        if traj_type == "circle":
            self.follow_circle(curr_pos)
        elif traj_type == "point":
            self.hover_at_point(curr_pos)
        elif traj_type == "bspline":
            self.follow_bspline(curr_pos)

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

    def follow_circle(self, curr_pos):
        current_time = rospy.get_time()

        # If self.start_time is 0 (i.e. the drone has just switched to using geometric
        # controller), the current time and starting pos will be updated
        if self.start_time == 0:
            self.start_time = current_time
            self.starting_x = curr_pos.x
            self.starting_y = curr_pos.y
            self.starting_z = curr_pos.z

        time_difference = current_time - self.start_time

        # Functions are evaluated at each time step, and published in a message to the
        # controller
        factor = 1.5
        radius = 1.5

        self.desired_pos[0][0] = radius * np.sin(time_difference * factor) + self.starting_x
        self.desired_pos[1][0] = radius * np.cos(time_difference * factor) - radius + self.starting_y
        self.desired_pos[2][0] = self.starting_z
        self.desired_vel[0][0] = factor * radius * np.cos(time_difference * factor)
        self.desired_vel[1][0] = -factor * radius * np.sin(time_difference * factor)
        self.desired_vel[2][0] = 0
        self.desired_acc[0][0] = -(factor ** 2) * radius * np.sin(time_difference * factor)
        self.desired_acc[1][0] = -(factor ** 2) * radius * np.cos(time_difference * factor)
        self.desired_acc[2][0] = 0

    def hover_at_point(self, curr_pos):
        current_time = rospy.get_time()

        # If self.start_time is 0 (i.e. the drone has just switched to using geometric
        # controller), the current time and desired pos will be updated
        if self.start_time == 0:
            self.start_time = current_time
            self.desired_x = curr_pos.x
            self.desired_y = curr_pos.y
            self.desired_z = curr_pos.z

        self.desired_pos[0][0] = self.desired_x
        self.desired_pos[1][0] = self.desired_y
        self.desired_pos[2][0] = self.desired_z
        self.desired_vel[0][0] = 0
        self.desired_vel[1][0] = 0
        self.desired_vel[2][0] = 0
        self.desired_acc[0][0] = 0
        self.desired_acc[1][0] = 0
        self.desired_acc[2][0] = 0

    def bspline_callback(self, msg):
        rospy.loginfo("Entered bspline callback")

        pos_pts = msg.pos_pts

        x = []
        y = []
        z = []

        for pt in pos_pts:
            x.append(pt.x)
            y.append(pt.y)
            z.append(pt.z)

        self.pos_x = scipy.interpolate.BSpline(
            t=msg.knots, c=x, k=msg.order, extrapolate=False
        )
        self.vel_x = self.pos_x.derivative()
        self.acc_x = self.vel_x.derivative()

        self.pos_y = scipy.interpolate.BSpline(
            t=msg.knots, c=y, k=msg.order, extrapolate=False
        )
        self.vel_y = self.pos_y.derivative()
        self.acc_y = self.vel_y.derivative()

        self.pos_z = scipy.interpolate.BSpline(
            t=msg.knots, c=z, k=msg.order, extrapolate=False
        )
        self.vel_z = self.pos_z.derivative()
        self.acc_z = self.vel_z.derivative()

        self.bspline_start_time = msg.start_time.to_sec()
        self.in_hover = False

    def get_in_hover(self, curr_pos):
        if not self.in_hover:
            rospy.loginfo("hover until received next bspline.")
            self.in_hover = True
            self.desired_x = curr_pos.x
            self.desired_y = curr_pos.y
            self.desired_z = curr_pos.z

        self.desired_pos[0][0] = self.desired_x
        self.desired_pos[1][0] = self.desired_y
        self.desired_pos[2][0] = self.desired_z
        self.desired_vel[0][0] = 0
        self.desired_vel[1][0] = 0
        self.desired_vel[2][0] = 0
        self.desired_acc[0][0] = 0
        self.desired_acc[1][0] = 0
        self.desired_acc[2][0] = 0

    def follow_bspline(self, curr_pos):
        try:
            current_time = rospy.get_time() - self.bspline_start_time

            # Previously found Bsplines are evaluated
            x_des = self.pos_x(current_time)
            y_des = self.pos_y(current_time)
            z_des = self.pos_z(current_time)
            vx_des = self.vel_x(current_time)
            vy_des = self.vel_y(current_time)
            vz_des = self.vel_z(current_time)
            ax_des = self.acc_x(current_time)
            ay_des = self.acc_y(current_time)
            az_des = self.acc_z(current_time)

            if (
                np.isnan(x_des)
                or np.isnan(y_des)
                or np.isnan(z_des)
                or np.isnan(vx_des)
                or np.isnan(vy_des)
                or np.isnan(vz_des)
                or np.isnan(ax_des)
                or np.isnan(ay_des)
                or np.isnan(az_des)
            ):
                self.get_in_hover(curr_pos)
            else:
                self.desired_pos[0][0] = self.clip_by_derivative(self.desired_pos[0][0], x_des, MAX_VEL)
                self.desired_pos[1][0] = self.clip_by_derivative(self.desired_pos[1][0], y_des, MAX_VEL)
                self.desired_pos[2][0] = self.clip_by_derivative(self.desired_pos[2][0], z_des, MAX_VEL)
                self.desired_pos[2][0] = np.clip(self.desired_pos[2][0], MIN_Z, MAX_Z)
                self.desired_vel[0][0] = self.clip_by_derivative(self.desired_vel[0][0], vx_des, MAX_ACC)
                self.desired_vel[1][0] = self.clip_by_derivative(self.desired_vel[1][0], vy_des, MAX_ACC)
                self.desired_vel[2][0] = self.clip_by_derivative(self.desired_vel[2][0], vz_des, MAX_ACC)
                self.desired_vel = np.clip(self.desired_vel, -MAX_VEL, MAX_VEL)
                self.desired_acc[0][0] = self.clip_by_derivative(self.desired_acc[0][0], ax_des, MAX_JERK)
                self.desired_acc[1][0] = self.clip_by_derivative(self.desired_acc[1][0], ay_des, MAX_JERK)
                self.desired_acc[2][0] = self.clip_by_derivative(self.desired_acc[2][0], az_des, MAX_JERK)
                self.desired_acc = np.clip(self.desired_acc, -MAX_ACC, MAX_ACC)

        except Exception:
            self.get_in_hover(curr_pos)
            rospy.logwarn_once("Run `roslaunch ego_planner run_with_vicon.launch` now")

    def clip_by_derivative(self, prev_val, val, max_der):
        max_change = max_der * (self.eval_t - self.prev_eval_t)
        return np.clip(val, prev_val - max_change, prev_val + max_change)

    def calculate_yaw(self):
        prev_yaw = self.yaw
        new_yaw = np.arctan2(self.desired_vel[1][0], self.desired_vel[0][0])
        max_yaw_change = MAX_ANG_VEL * (self.eval_t - self.prev_eval_t)
        if new_yaw - prev_yaw > np.pi:
            if new_yaw - prev_yaw - 2 * np.pi < -max_yaw_change:
                new_yaw = prev_yaw - max_yaw_change
        elif new_yaw - prev_yaw < -np.pi:
            if new_yaw - prev_yaw + 2 * np.pi > max_yaw_change:
                new_yaw = prev_yaw + max_yaw_change
        else:
            if new_yaw - prev_yaw < -max_yaw_change:
                new_yaw = prev_yaw - max_yaw_change
            elif new_yaw - prev_yaw > max_yaw_change:
                new_yaw = prev_yaw + max_yaw_change
        if new_yaw > np.pi:
            new_yaw -= 2 * np.pi
        if new_yaw < -np.pi:
            new_yaw += 2 * np.pi
        self.yaw = new_yaw
        return new_yaw

    def publish_bool(self, value):
        bool_msg = Bool()
        bool_msg.data = value
        if not value:
            self.start_time = 0
        self.bool_publisher.publish(bool_msg)
