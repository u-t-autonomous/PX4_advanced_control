# PX4_advanced_control

Repo consists of a base controller that uses mavros to communicate with PX4-based drones and advanced controllers that extend the base controller. The code is set up to switch between base controller and advanced controller during flight. Switching happens either manually or automatically. To manually switch, the user can use pre-defined commands in 'base_cmd_line' and 'advanced_cmd_line'. The code automatically switches out of the advanced controller if the drone becomes unstable.

To implement an advanced controller, use 'advanced_controller' and 'advanced_cmd_line'. 
