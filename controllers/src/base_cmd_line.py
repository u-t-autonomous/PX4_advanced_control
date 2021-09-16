""" Contains base commands to control drone via mavros """

import rospy


class CommandLine(object):
    def __init__(self, offboard_controller):
        self.offboard_controller = offboard_controller
        # Command line key word used to call offboardController commands
        # List contains offboard command, number of expected inputs, and special arguments
        self.dict_command = {'arm': self.offboard_controller.setArm,
                             'disarm': self.offboard_controller.setDisarm,
                             'takeoff': self.offboard_controller.setTakeoff,
                             'land': self.offboard_controller.setLand,
                             'KILL': self.offboard_controller.setKill,
                             'offboard': self.offboard_controller.setOffboardMode,
                             'pos': self.offboard_controller.move,
                             'vel': self.offboard_controller.move,
                             'rpy': self.offboard_controller.update_des_rpy,
                             'rpyRate': self.offboard_controller.update_des_rpy,
                             'yaw': self.offboard_controller.update_des_yaw,
                             'yawRate': self.offboard_controller.update_des_yaw,
                             'b': self.offboard_controller.move,
                             'xpos': self.offboard_controller.relative_position,
                             'ypos': self.offboard_controller.relative_position,
                             'zpos': self.offboard_controller.relative_position,
                             'setpoint': self.offboard_controller.log_setpoint,
                             'stop': self.offboard_controller.stop,
                             'm': self.offboard_controller.jump,
                             'start_script': self.offboard_controller.start_script,
                             }
        self.warn_args = False

    def check_input_length(self, nmbr_req, nmbr_is):
        """ Checks if the right number of inputs is given """
        if nmbr_req == nmbr_is:
            return True
        else:
            self.warn_args = True
            return False

    def command(self, val):
        """ Checks if input requirements are met and calls offboard_controller """
        if val[0] == 'takeoff' and self.check_input_length(2, len(val)):
            self.dict_command[val[0]](float(val[1]))
        elif val[0] == 'yaw' and self.check_input_length(2, len(val)):
            self.dict_command[val[0]](float(val[1]), False)
        elif val[0] == 'yawRate' and self.check_input_length(2, len(val)):
            self.dict_command[val[0]](float(val[2]), True)
        elif val[0] == 'pos' and self.check_input_length(4, len(val)):
            self.dict_command[val[0]](float(val[1]), float(val[2]), float(val[3]), False)
        elif val[0] == 'vel' and self.check_input_length(4, len(val)):
            self.dict_command[val[0]](float(val[1]), float(val[2]), float(val[3]), True)
        elif val[0] == 'rpy' and self.check_input_length(4, len(val)):
            self.dict_command[val[0]](float(val[1]), float(val[2]), float(val[3]), False)
        elif val[0] == 'rpyRate' and self.check_input_length(4, len(val)):
            self.dict_command[val[0]](float(val[1]), float(val[2]), float(val[3]), True)
        # fail safe command that quickly turns copter back to safe point
        elif val[0] == 'b':
            self.dict_command[val[0]](float(self.offboard_controller.safePoint.x),
                                      float(self.offboard_controller.safePoint.y),
                                      float(self.offboard_controller.safePoint.z), False)
            rospy.loginfo('RETURNING TO SAFETY!')
        elif (val[0] == 'xpos' or val[0] == 'ypos' or val[0] == 'zpos') \
                and self.check_input_length(2, len(val)):
            self.dict_command[val[0]](val[0], float(val[1]))
        elif val[0] == 'start_script':
            self.dict_command[val[0]]()
        else:
            return False
        # returns true if val was an implemented command
        return True

    def process_command(self, val):
        """ Checks if command line input is valid """
        self.warn_args = False
        # Turn off script control
        if self.offboard_controller.script_ctl == True:
            rospy.loginfo("Entering command line control. Use start_script to resume scripted control")
            self.offboard_controller.script_ctl = False
        if val[0] in self.dict_command:
            # see if special condition applies for input
            bool_success = self.command(val)
            if not bool_success:
                if val[0] in self.dict_command and self.check_input_length(1, len(val)):
                    self.dict_command[val[0]]()
                elif self.warn_args:
                    rospy.logerr("Wrong number of arguments!")
                    self.warn_args = False
        else:
            rospy.logerr('Invalid command!')

    def get_command_from_console(self):
        while not rospy.is_shutdown():
            try:
                val = raw_input().split(' ')

                self.process_command(val)

            except ValueError as ex:
                rospy.logerr('Last command threw ' + ex.__doc__ + ' Check input!')
            except TypeError as ex:
                rospy.logerr('Last command threw ' + ex.__doc__ + ' Check number of inputs!')
            except Exception as ex:
                rospy.logerr('Last command threw general error ' + ex.__doc__)
