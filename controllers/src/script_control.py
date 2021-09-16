#!/usr/bin/env python2

import sys
import threading 

# ROS python API
import rospy

# Stamped Pose msgs
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16, String
from sensor_msgs.msg import BatteryState

# mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# custom classes
from base_cmd_line import CommandLine
from base_controller import OffboardControl
from advanced_cmd_line import CommandLineExt
from advanced_controller import AdvancedController


def main():
    rospy.init_node('script_control', anonymous=True)
    updateFreq = 100
    rate = rospy.Rate(updateFreq)

    # check whether advanced controller should be started
    bool_adv_ctrl = rospy.get_param('~adv_ctrl', False)
    bool_sim = rospy.get_param('~bool_sim', False)
    if bool_sim:
        hoverVal = 0.565  # Value used only when simulation in gazebo
    else:
        hoverVal = rospy.get_param('~hoverThrust', 0.44)
    offboard_controller = OffboardControl(hoverVal=hoverVal, updateTime=1.0/updateFreq)

    if bool_sim:
        rospy.Subscriber('mavros/local_position/pose', PoseStamped,
                         offboard_controller.pos_callback)
    else:
        rospy.Subscriber('mavros/vision_pose/pose', PoseStamped,
                         offboard_controller.pos_callback)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped,
                     offboard_controller.estimate_callback)
    rospy.Subscriber('mavros/state', State,
                     offboard_controller.state_callback)
    rospy.Subscriber('mavros/setpoint_raw/target_attitude', AttitudeTarget,
                     offboard_controller.euler_callback)
    rospy.Subscriber('mavros/setpoint_raw/target_local', PositionTarget,
                     offboard_controller.check_moving)


    # Sleep some time for the system to be ready
    for _ in range(1000):
        rate.sleep()

    if bool_sim:
        rospy.loginfo(" --- STARTING SIMULATION! --- ")
    else:
        rospy.loginfo(" --- STARTING EXPERIMENT! --- ")

    rospy.loginfo("Arming and setting to offboard")
    offboard_controller.setTakeoff()

    while not rospy.is_shutdown():
        offboard_controller.update()
        if not offboard_controller.is_moving:
            offboard_controller.next_point()
        rate.sleep()


if __name__ == "__main__":
    main()
