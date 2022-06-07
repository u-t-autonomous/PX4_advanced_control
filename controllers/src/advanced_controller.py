import rospy
from base_controller import OffboardControl

class AdvancedController(OffboardControl):
    """ Extends basic drone controller with advanced control methods """

    def __init__(self, hoverVal=0.5, updateTime=0.01, bool_sim=False):
        super(AdvancedController, self).__init__(hoverVal=hoverVal, updateTime=updateTime)
        self.bool_sim = bool_sim
        self.reset_automatic()
        self.auto_traj_freq = rospy.get_param('~traj_freq', 50)

        self.angle_pub_counter = 0
        self.angle_pub_freq = 5 # Hz

        rospy.loginfo('Advanced controller initalized!')

    def reset_automatic(self):
        """ Reset all automatic flags and counters """
        self.auto_traj_flag = False
        self.auto_traj_counter = 0

    def update(self):
        """ Enhance base method with advanced controller """
        self.check_security()

        # automatic modes for advanced controllers
        if self.auto_traj_flag:
            # input trajectory controller
            self.trajectory_control()

        else:
            # uses standard PX4 control
            if self.is_posctl:
                self.sp_positionTarget.publish(self.setpoint_target)
                self.setpoint_target.header.seq += 1
            else:
                self.update_des_throttle()
                self.sp_attitudeTarget.publish(self.attitude_target)
                self.attitude_target.header.seq += 1

        # only for rqt_multiplot (can be commented out if rqt_multiplot is not used)
        if self.sleep_checker(self.angle_pub_counter, self.angle_pub_freq):
            self.local_euler_pub.publish(self.radToDegVec(self.curr_orientation))
            self.euler_sp_pub.publish(self.radToDegVec(self.sp_orientation))
            self.est_euler_pub.publish(self.radToDegVec(self.est_orientation))
            self.angle_pub_counter = 0
        else:
            self.angle_pub_counter += 1

    def trajectory_control(self):
        """ Triggers new trajectory control based on desired frequency """
        # enter automatic mode if counter is high enough
        # breaks automatic when manual input, is_point_in_cube(), or ground contact is triggered
        if self.sleep_checker(self.auto_traj_counter, self.auto_traj_freq):
            self.next_traj_step()
            self.auto_traj_counter = 0
        else:
            self.auto_traj_counter += 1

    def next_traj_step(self):
        """
        Input code for trajectory controller here
        """
        pass

    def start_automatic(self, mode):
        """ Set automatic flags for advanced controllers """
        # descend
        if mode == 0:
            self.reset_automatic()
            self.auto_traj_flag = True
            rospy.loginfo('Starting traj control ...')

    def is_point_in_cube(self, coord, corner1, corner2):
        """ Enhance base method with setting automatic flag if position outside of cube """
        bool = super(AdvancedController, self).is_point_in_cube(coord, corner1, corner2)
        if not bool:
            self.reset_automatic()
        return bool

    def sleep_checker(self, counter, freq):
        """ Checks if counter is high enough compared to heartbeat frequency """
        bool = (counter >= int((1 / self.updateTime) / freq) - 1)
        return bool