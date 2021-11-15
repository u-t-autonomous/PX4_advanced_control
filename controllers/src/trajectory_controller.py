#!/usr/bin/env python2

import numpy as np
import rospy
from scipy.interpolate import BSpline

from controllers.msg import Bspline
from geometry_msgs.msg import Transform, Twist
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


MIN_Z = 0.8
MAX_Z = 2.0
MAX_VEL = 0.2
MAX_ACC = 1.0
MAX_JERK = 1.0
MAX_ANG_VEL = np.pi / 8


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

        # bspline
        self.pos_bspline, self.vel_bspline, self.acc_bspline = None, None, None

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
        msg.header.stamp = rospy.Time.now()
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
        pos = []

        for pt in pos_pts:
            pos.append([pt.x, pt.y, pt.z])

        new_pos = BSpline(t=msg.knots, c=pos, k=msg.order, extrapolate=False)
        new_vel = new_pos.derivative()
        new_acc = new_vel.derivative()
        new_start_time = msg.start_time.to_sec()
        new_knots = msg.knots

        # if self.pos_bspline is not None:
        #     # Bspline smoothing
        #     knot_t = new_knots[1] - new_knots[0]
        #     num_eval_in_1st_part = 20
        #     num_eval_in_2nd_part = 50
        #     num_1st_part_knots = 1

        #     # pos and vel from prev bspline in 1st knot
        #     t_1st_part = np.linspace(new_knots[3], new_knots[3 + num_1st_part_knots],
        #                              num_eval_in_1st_part, endpoint=False)
        #     t_1st_part_relative = t_1st_part + new_start_time - self.bspline_start_time
        #     pos_1st_part = self.pos_bspline(t_1st_part_relative)
        #     vel_1st_part = self.vel_bspline(t_1st_part_relative)
        #     acc_1st_part = self.acc_bspline(t_1st_part_relative)

        #     t_2nd_part = np.linspace(new_knots[3 + num_1st_part_knots], new_knots[-4],
        #                              num_eval_in_2nd_part, endpoint=False)
        #     pos_2nd_part = new_pos(t_2nd_part)
        #     vel_2nd_part = new_vel(t_2nd_part)
        #     acc_2nd_part = new_acc(t_2nd_part)

        #     # linear fitting
        #     # y (2 * T, 3), T: num of eval timestamps
        #     # X (2 * T, Nc), Nc: num of control pts
        #     y = np.concatenate([pos_1st_part, pos_2nd_part,
        #                         vel_1st_part, vel_2nd_part,
        #                         acc_1st_part, acc_2nd_part])

        #     pos_coef_bspline = BSpline(t=msg.knots, c=np.eye(len(pos)), k=msg.order, extrapolate=False)
        #     vel_coef_bspline = pos_coef_bspline.derivative()
        #     acc_coef_bspline = vel_coef_bspline.derivative()

        #     coef_t = np.concatenate([t_1st_part, t_2nd_part])

        #     X = np.concatenate([pos_coef_bspline(coef_t),
        #                         vel_coef_bspline(coef_t),
        #                         acc_coef_bspline(coef_t)])

        #     XTX_inv = np.linalg.inv(np.matmul(X.T, X))
        #     new_pos_pts = np.matmul(np.matmul(XTX_inv, X.T), y)

        #     new_pos = BSpline(t=msg.knots, c=new_pos_pts, k=msg.order, extrapolate=False)
        #     new_vel = new_pos.derivative()
        #     new_acc = new_vel.derivative()

        self.pos_bspline = new_pos
        self.vel_bspline = new_vel
        self.acc_bspline = new_acc

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
        if self.pos_bspline is not None:
            current_time = rospy.get_time() - self.bspline_start_time

            # Previously found Bsplines are evaluated
            x_des, y_des, z_des = self.pos_bspline(current_time)
            vx_des, vy_des, vz_des = self.vel_bspline(current_time)
            ax_des, ay_des, az_des = self.acc_bspline(current_time)

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
                desired_vel_norm = np.linalg.norm(self.desired_vel)
                self.desired_vel *= np.clip(desired_vel_norm, 0, MAX_VEL) / desired_vel_norm
                self.desired_acc[0][0] = self.clip_by_derivative(self.desired_acc[0][0], ax_des, MAX_JERK)
                self.desired_acc[1][0] = self.clip_by_derivative(self.desired_acc[1][0], ay_des, MAX_JERK)
                self.desired_acc[2][0] = self.clip_by_derivative(self.desired_acc[2][0], az_des, MAX_JERK)
                desired_acc_norm = np.linalg.norm(self.desired_acc)
                self.desired_acc *= np.clip(desired_acc_norm, 0, MAX_ACC) / desired_acc_norm
        else:
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
