"""
Extends commands for advanced controller
"""

from base_cmd_line import CommandLine


class CommandLineExt(CommandLine):
    def __init__(self, offboard_controller):
        super(CommandLineExt, self).__init__(offboard_controller)
        self.dict_command.update(
            {
                'traj': self.offboard_controller.start_automatic,
             }
        )

        self.warn_args = False

    def command(self, val):
        """ Enhance parent method with more commands """
        # any manual input breaks automatic mode
        if val[0] is not None:
            if (val[0] == 'b' or val[0] == 'm' or val[0] == 'pos' or val[0] == 'stop' \
                    or val[0] == 'land' or val[0] == 'KILL'):
                self.offboard_controller.reset_automatic()

        # return true if input is old command and matches requirement for number of inputs
        bool_success = super(CommandLineExt, self).command(val)

        # check if input is in advanced controller
        if not bool_success:
            if val[0] == 'traj' and self.check_input_length(1, len(val)):
                self.dict_command[val[0]](0)

            # add more commands here

            else:
                return False
        # returns true if input is an implemented command
        return True
