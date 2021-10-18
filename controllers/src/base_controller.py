#!/usr/bin/env python

# ROS python API
import rospy
import tf
import math
import random
import csv
import json

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, Vector3, PoseStamped, PolygonStamped

# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

CATKIN_WS = '/home/undergrad/catkin_ws/src/'

# Class that can be used for any sort of offboard control
class OffboardControl(object):
    def __init__(self, hoverVal=0.5, updateTime=0.01):

        # To uncomment if euler angle from Vicon are required
        # Vicon estimation euler angles
        # Current euler angle (rad) from vicon estimate x<->roll, y<->pitch, z<->yaw
        self.est_euler_pub = rospy.Publisher('estimate_position/euler', Point, queue_size=5)
        self.est_orientation = Point()

        # safe point and cubes
        self.setPosCubeSafety()
        self.setSafePointPos()
        self.safeEuleurAngle()
        self.limit_err_msg = 0

        # Publisher topics
        # Topic for x, y z yaw/yawrate setpoint
        self.sp_positionTarget = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget,
                                                 queue_size=1)
        # Topic for roll pitch yaw throttle setpoint
        self.sp_attitudeTarget = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget,
                                                 queue_size=1)
        # The following two topics are for debugging
        # Give estimated roll pitch yaw in degree in ENU
        self.local_euler_pub = rospy.Publisher('local_position/euler', Point, queue_size=5)
        # Give roll pitch yaw setpoint in degree in ENU
        self.euler_sp_pub = rospy.Publisher('setpoint_position/euler', Point, queue_size=5)

        # Define the setpoint position target message
        # http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
        # Define the PositionTarget setpoint that will be always used
        self.setpoint_target = PositionTarget() # Frame is the world frame
        self.setpoint_target.header.frame_id = 'world'
        # We use the local_frame_ned coordinate system
        self.setpoint_target.coordinate_frame = 1
        # the type_mask here ignore all field except position x,y,z and yaw_rate
        self.setpoint_target.type_mask = int('100111111000', 2)
        self.setpoint_target.yaw = 1.5708
        # Define the setpoint attitude target message
        # http://docs.ros.org/api/mavros_msgs/html/msg/AttitudeTarget.html
        self.attitude_target = AttitudeTarget()
        self.attitude_target.header.frame_id = 'world' # Frame is the world frame
        # Ignoring roll rate, pitch rate and yaw rate
        self.attitude_target.type_mask = int('00111',2)

        # Define the information to save from callback
        self.curr_position = Point()
        # Current euler angle (rad) of the drone x<->roll, y<->pitch, z<->yaw
        self.curr_orientation = Point()
        # Setpoint euler angle (rad) published on Attitude Target
        self.sp_orientation = Point()
        # Current drone states <-> can be used else to check mode of drone.
        self.state = State()

        # Define the current mode of the setpoint (attitude or poisition control)
        # By default we are in Position control
        self.is_posctl = True
        # Set the PD coefficient and initialize Vz estimation
        if hoverVal <= 0.6 or hoverVal >= 0.4:
            self.hoverThrust = hoverVal
        else:
            raise ValueError('hoverThrust is outside of safe range!')
        self.updateTime = updateTime
        self.Kp_alt = 0.8
        self.Kd_alt = 0.2
        self.last_zerror = 0.0

        rospy.loginfo ("[PARAMS] Update period [s] = "+ str(self.updateTime)
               + " , Hover thrust = "+ str(self.hoverThrust))

        self.rel_pos_limit = rospy.get_param('~rel_pos_limit', 0.15) # limit in [m]
        if self.rel_pos_limit < 0:
            raise ValueError
        self.target_reached = True
        self.coords_index = -1
        self.script_ctl = False
        # Data structure: dict of dict of list
        # {
        #     source_target:
        #         {
        #            dest_target:
        #               {
        #                   x: 
        #                       {
        #                           y: [left, right, up, down]
        #                       }
        #               }
        #         }
        # }
        self.policy_dict = dict()
        for source in range(1,5):
            self.policy_dict[source] = dict()
            for dest in range(1,5):
                self.policy_dict[source][dest] = dict()
                try:
                    with open(CATKIN_WS + 'PX4_advanced_control/controllers/src/target_pairs/{}_to_{}.csv'.format(source, dest), 'r') as f:
                        reader = csv.reader(f)
                        for row in reader:
                            # Extract coordinates
                            x_coord = (row[0].strip("[]").split()[0])
                            y_coord = (row[0].strip("[]").split()[1])                
                            # Extract probabilities
                            raw_probs = [float(x) for x in row[1].strip("[]").split()]
                            prob_values = list()
                            for probability in raw_probs:
                                if probability < 0.01:
                                    prob_values.append(0)
                                else:
                                    prob_values.append(probability)
                            normalized_prob_values = [prob/sum(prob_values) for prob in prob_values]
                            # Insert into dictionary
                            if x_coord not in self.policy_dict[source][dest].keys():
                                self.policy_dict[source][dest][x_coord] = dict()
                            self.policy_dict[source][dest][x_coord][y_coord] = normalized_prob_values
                except:
                    pass
        # destination targets
        # 1 - (-4.5, -3.5)
        # 2 - (2.5, -3.5)
        # 3 - (1.5, 0.5)
        # 4 - (-2.5, 4.5)
        self.current_dest = -1
        self.current_src = -1
        rospy.loginfo(json.dumps(self.policy_dict, indent=4, sort_keys=True))


    def update(self):
        """ Heartbeat of drone """
        self.check_security()
        if self.is_posctl:
            self.sp_positionTarget.publish(self.setpoint_target)
            self.setpoint_target.header.seq += 1
        else:
            self.update_des_throttle()
            self.sp_attitudeTarget.publish(self.attitude_target)
            self.attitude_target.header.seq += 1

        self.local_euler_pub.publish(self.radToDegVec(self.curr_orientation))
        self.euler_sp_pub.publish(self.radToDegVec(self.sp_orientation))
        self.est_euler_pub.publish(self.radToDegVec(self.est_orientation))

    def move(self, x, y, z, rate=False):
        """ Gives position or velocity commands to drone"""
        if rate == False:
            rospy.logdebug("Target: {} {} {}".format(x, y, z))
        self.is_posctl = True # This is definitely posctl -> so update the variable
        self.target_reached = False
        # If this is velocity control change the type mask
        if rate:
            self.setpoint_target.type_mask =  \
                self.setpoint_target.type_mask | int('000000000111',2) # Ignore x,y,z values
            self.setpoint_target.type_mask =  \
                self.setpoint_target.type_mask & int('111111000111',2) # Activate Vx, Vy, Vz values

            # in velocity control, we use BODY FRAME coordinate system
            self.setpoint_target.coordinate_frame = 8
            self.setpoint_target.velocity  =  Vector3(x, y, z)
            self.setpoint_target.header.stamp = rospy.Time.now()

        # makes sure commanded pos is within position cube but also above some altitude (> 0.2 m)
        elif not self.is_point_in_cube(Point(x,y,z),
                Point(self.posCubeCorner1.x, self.posCubeCorner1.y, 0.2), self.posCubeCorner2):
            rospy.logerr("[MOVE] Target point (" + str(x)+ ", "+str(y)+", "
                         +str(z)+") not inside the Position Cube! IGNORED")

        else:
            self.setpoint_target.type_mask =  \
                self.setpoint_target.type_mask | int('000000111000',2) # Ignore Vx,Vy,Vz values
            self.setpoint_target.type_mask =  \
                self.setpoint_target.type_mask & int('111111111000',2) # Activate x, y, z values
            self.setpoint_target.coordinate_frame = 1
            self.setpoint_target.position = Point(x,y,z)
            self.setpoint_target.header.stamp = rospy.Time.now()

    def relative_position(self, dir, magnitude):
        """
        Args:
            dir (): Specifies the cartesian axis the drone should travel along
            magnitude (): The relative distance that the drone will travel in [m]
        """
        if dir == 'xpos' and abs(magnitude) <= self.rel_pos_limit:
            self.move(self.setpoint_target.position.x + magnitude,
                      self.setpoint_target.position.y, self.setpoint_target.position.z)
            self.log_setpoint()

        elif dir == 'ypos' and abs(magnitude) <= self.rel_pos_limit:
            self.move(self.setpoint_target.position.x,
                      self.setpoint_target.position.y + magnitude, self.setpoint_target.position.z)
            self.log_setpoint()

        elif dir == 'zpos' and abs(magnitude) <= self.rel_pos_limit:
            self.move(self.setpoint_target.position.x, self.setpoint_target.position.y,
                      self.setpoint_target.position.z + magnitude)
            self.log_setpoint()

        elif abs(magnitude) > self.rel_pos_limit:
            rospy.logerr("Relative distance too large! Rejected.")

        else:
            rospy.logwarn("Relative_position command not correct!")

    def log_setpoint(self):
        rospy.loginfo("Target setpoint: {:.3f} {:.3f} {:.3f}".format(
            self.setpoint_target.position.x,
            self.setpoint_target.position.y,
            self.setpoint_target.position.z))

    def jump(self):
        """ Immmediately sets altitude target 10 cm higher """
        if self.rel_pos_limit > 0.1:
            self.relative_position('zpos', 0.1)
        else:
            self.relative_position('zpos', self.rel_pos_limit - 0.01)

    def stop(self):
        """ Stop drone at current location or setting velocities to zero """

        if self.setpoint_target.type_mask & int('000000111000', 2) > 0:
            self.move(self.curr_position.x, self.curr_position.y, self.curr_position.z)
            self.log_setpoint()
        elif self.setpoint_target.type_mask & int('000000000111', 2) > 0:
            self.move(0, 0, 0, rate=True)
            rospy.loginfo('Velocity setpoint set to zero!')
        else:
            rospy.logwarn('Unknown bit mask. Stop rejected!')


    def setPosCubeSafety(self):
        point_1=rospy.get_param('~safe_cube_corner_1', [-2, -2, -0.5])
        point_2=rospy.get_param('~safe_cube_corner_2', [2, 2, 2])
        self.posCubeCorner1 = Point(point_1[0], point_1[1], point_1[2])
        self.posCubeCorner2 = Point(point_2[0], point_2[1], point_2[2])
        if self.posCubeCorner1.x > self.posCubeCorner2.x \
                or self.posCubeCorner1.y > self.posCubeCorner2.y \
                or self.posCubeCorner1.z > self.posCubeCorner2.z:
            rospy.logerr("[POS_CUBE] Corner 1 should be bot right back and Corner 2 the top front "
                         "left cube corner! Failed")
            sys.exit()
        else:
            rospy.loginfo("[POS_CUBE] Corner 1: "+str(self.posCubeCorner1.x) + ", "
                          +str(self.posCubeCorner1.y)+", "+str(self.posCubeCorner1.z))
            rospy.loginfo("[POS_CUBE] Corner 2: " + str(self.posCubeCorner2.x) + ", "
                          +str(self.posCubeCorner2.y)+", "+str(self.posCubeCorner2.z))

    def setSafePointPos(self):
        point_safe = rospy.get_param('~safe_point', [0, 0, 1.2])
        safePoint = Point(point_safe[0],point_safe[1],point_safe[2])
        if not self.is_point_in_cube(safePoint, self.posCubeCorner1, self.posCubeCorner2):
            raise ValueError("[SAFE_POINT] Safe point not inside Attitude Cube! Failed")
        else:
            self.safePoint = safePoint
            rospy.loginfo("[SAFE POINT] " + str(self.safePoint.x) + ", " + str(self.safePoint.y)
                          + ", " + str(self.safePoint.z))

    def safeEuleurAngle(self):
        point_euler=rospy.get_param('~euler_limit', [10, 10, 45])
        self.eulerLimit = Point(point_euler[0], point_euler[1], point_euler[2])
        rospy.loginfo("[SAFE_EULER] "+ str(self.eulerLimit.x) + ", "+ str(self.eulerLimit.y)+ ", "
                      + str(self.eulerLimit.z))

    def radToDegVec(self, angle):
        """ Convert an euler angle vector in radians to degree """
        return Point(math.degrees(angle.x), math.degrees(angle.y), math.degrees(angle.z))

    def check_security(self):
        if (not self.is_point_in_cube(self.curr_position, self.posCubeCorner1, self.posCubeCorner2)):
            if self.limit_err_msg != 2:
                rospy.logwarn("DRONE IS OUT OF POS CUBE! Going back to Safe Target point...")
                self.limit_err_msg = 2
            self.move(self.safePoint.x,self.safePoint.y,self.safePoint.z,rate=False)

        elif (not self.is_posctl and not self.is_point_in_cube(self.radToDegVec(self.curr_orientation),
                Point(-self.eulerLimit.x,-self.eulerLimit.y, -self.eulerLimit.z), self.eulerLimit)):
            if self.limit_err_msg != 1:
                rospy.logwarn("DRONE IS OUTSIDE EULER LIMIT! "
                              "Going back to Safe Target point...")
                self.limit_err_msg = 1
            self.move(self.safePoint.x,self.safePoint.y,self.safePoint.z,rate=False)

        # is drone is not in position control, catch drone if below 0.6 m altitude
        elif (not self.is_posctl and not self.is_point_in_cube(self.curr_position,
                    Point(self.posCubeCorner1.x, self.posCubeCorner1.y, 0.6), self.posCubeCorner2)):
            if self.limit_err_msg != 2:
                rospy.logwarn("DRONE IS OUT OF POS CUBE! Going back to Safe Target point...")
                self.limit_err_msg = 2
            self.move(self.safePoint.x,self.safePoint.y,self.safePoint.z,rate=False)

        else:
            if self.limit_err_msg != 4:
                rospy.loginfo('DRONE IS WITHIN SAFETY BOX!')
                self.limit_err_msg = 4

    def is_point_in_cube(self, coord, corner1, corner2):
        """ Check if a point is inside a Cube given by end points Corner1 and Corner2 """
        return coord.x <= corner2.x and coord.y <= corner2.y and coord.z <= corner2.z \
               and coord.x >= corner1.x and coord.y >= corner1.y and coord.z >= corner1.z

    def update_des_yaw(self, yaw, rate=False):
        """ Gives yaw angle or yaw rate commands """
        self.is_posctl = True
        # Check if it's yaw or yaw rate control and change the type mask according
        if rate:
            self.setpoint_target.type_mask = \
                self.setpoint_target.type_mask | int('010000000000', 2)
            self.setpoint_target.type_mask = \
                self.setpoint_target.type_mask & int('011111111111', 2)
            self.setpoint_target.yaw_rate = math.radians(yaw)
            #self.sp_orientation = Point() # in rate control -> this value means nothing
        else:
            self.setpoint_target.type_mask = \
                self.setpoint_target.type_mask | int('100000000000', 2)
            self.setpoint_target.type_mask = \
                self.setpoint_target.type_mask & int('101111111111', 2)
            self.setpoint_target.yaw = math.radians(yaw)
            #self.sp_orientation = Point(0,0,math.radians(yaw))
        self.setpoint_target.header.stamp = rospy.Time.now()

    def update_des_rpy(self, roll, pitch, yaw, rate=False):
        """ Gives angle or angle rate commands to drone """
        self.is_posctl = False
        self.last_zerror = 0.0 # Reset last error of the z controller
        self.setpoint_target.position.z = \
            self.curr_position.z # In attitude control, Start by hovering at the current z position
        # Update the attitude Target for control in roll yaw pitch
        if rate:
            self.attitude_target.type_mask = 128 #int('10000',2)
            self.attitude_target.body_rate = \
                Vector3(math.radians(roll), math.radians(pitch), math.radians(yaw))
            #self.sp_orientation = Point() # in rate control -> this value means nothing
            self.attitude_target.header.stamp = rospy.Time.now()
        else:
            self.attitude_target.type_mask = 1 + 2 + 4 #int('00111',2)
            quatVal = tf.transformations.quaternion_from_euler(math.radians(roll),
                                                               math.radians(pitch), math.radians(yaw))
            #self.sp_orientation = Point(math.radians(roll), math.radians(pitch), math.radians(yaw))
            self.attitude_target.orientation.x = quatVal[0]
            self.attitude_target.orientation.y = quatVal[1]
            self.attitude_target.orientation.z = quatVal[2]
            self.attitude_target.orientation.w = quatVal[3]
            self.attitude_target.header.stamp = rospy.Time.now()

    def update_des_throttle(self):
        """ Manual throttle commands """
        err_z = self.curr_position.z - self.setpoint_target.position.z
        self.attitude_target.thrust = self.hoverThrust - self.Kp_alt * err_z \
                                      - self.Kd_alt * (err_z - self.last_zerror)/ self.updateTime
        self.last_zerror = err_z

    def pos_callback(self, msg):
        self.curr_position.x = msg.pose.position.x
        self.curr_position.y = msg.pose.position.y
        self.curr_position.z = msg.pose.position.z

        euler = tf.transformations.euler_from_quaternion((msg.pose.orientation.x,
                                                          msg.pose.orientation.y,
                                                          msg.pose.orientation.z,
                                                          msg.pose.orientation.w))
        self.curr_orientation.x = euler[0]#roll
        self.curr_orientation.y = euler[1]#pitch
        self.curr_orientation.z = euler[2]#yaw
        if self.is_posctl and not self.target_reached:
            self.target_reached = self.check_target_reached()

    def estimate_callback(self, msg):
        euler = tf.transformations.euler_from_quaternion((msg.pose.orientation.x,
                                                          msg.pose.orientation.y,
                                                          msg.pose.orientation.z,
                                                          msg.pose.orientation.w))
        self.est_orientation.x = euler[0]
        self.est_orientation.y = euler[1]
        self.est_orientation.z = euler[2]
        # print euler

    def euler_callback(self, msg):
        euler = tf.transformations.euler_from_quaternion((msg.orientation.x,
                                                          msg.orientation.y,
                                                          msg.orientation.z,
                                                          msg.orientation.w))
        self.sp_orientation.x = euler[0]
        self.sp_orientation.y = euler[1]
        self.sp_orientation.z = euler[2]

    def state_callback(self, msg):
        self.state = msg

    def check_target_reached(self, epsilon=0.1):
        return ((abs(self.curr_position.x - self.setpoint_target.position.x) < epsilon) and \
                (abs(self.curr_position.y - self.setpoint_target.position.y) < epsilon))

    def check_destination_reached(self, epsilon=0.05):
        if self.current_dest == 1:
            return (abs(self.setpoint_target.position.x + 4.5) < epsilon) and \
                   (abs(self.setpoint_target.position.y + 3.5) < epsilon)
        if self.current_dest == 2:
            return (abs(self.setpoint_target.position.x - 2.5) < epsilon) and \
                   (abs(self.setpoint_target.position.y + 3.5) < epsilon)
        if self.current_dest == 3:
            return (abs(self.setpoint_target.position.x - 1.5) < epsilon) and \
                   (abs(self.setpoint_target.position.y - 0.5) < epsilon)
        if self.current_dest == 4:
            return (abs(self.setpoint_target.position.x + 2.5) < epsilon) and \
                   (abs(self.setpoint_target.position.y - 4.5) < epsilon)
        return False

    def set_current_src(self, epsilon=0.2):
        if (abs(self.curr_position.x + 4.5) < epsilon) and \
           (abs(self.curr_position.y + 3.5) < epsilon):
            self.current_src = 1
            return True
        if (abs(self.curr_position.x - 2.5) < epsilon) and \
           (abs(self.curr_position.y + 3.5) < epsilon):
            self.current_src = 2
            return True
        if (abs(self.curr_position.x - 1.5) < epsilon) and \
           (abs(self.curr_position.y - 0.5) < epsilon):
            self.current_src = 3
            return True
        if (abs(self.curr_position.x + 2.5) < epsilon) and \
           (abs(self.curr_position.y - 4.5) < epsilon):
            self.current_src = 4
            return True
        return False

    def set_new_dest(self):
        while (self.current_dest == self.current_src) or (self.current_dest == -1):
            self.current_dest = random.randint(1,4)
        new_x = 0
        new_y = 0
        # destination targets
        # 1 - (-4.5, -3.5)
        # 2 - (2.5, -3.5)
        # 3 - (1.5, 0.5)
        # 4 - (-2.5, 4.5)
        if self.current_dest == 1:
            new_x = -4.5
            new_y = -3.5
        elif self.current_dest == 2:
            new_x = 2.5
            new_y = -3.5
        elif self.current_dest == 3:
            new_x = 1.5
            new_y = 0.5
        else:
            new_x = -2.5
            new_y = 4.5
        rospy.loginfo("New destination: ({}, {})".format(new_x, new_y))

    def next_point(self):
        # Assumes that setpoint_target is the current position has been reached
        # Get probability values list [left, right, up, down] based on current state
        if self.check_destination_reached():
            self.current_src = self.current_dest
            self.set_new_dest()
        prob_values = self.policy_dict[self.current_src][self.current_dest][str(self.setpoint_target.position.x)][str(self.setpoint_target.position.y)]
        left = prob_values[0]
        right = prob_values[1]
        up = prob_values[2]
        new_x = self.setpoint_target.position.x
        new_y = self.setpoint_target.position.y
        # Generate random value
        rand_float = random.uniform(0, 1)
        rospy.loginfo(prob_values)
        rospy.loginfo(rand_float)
        if rand_float < left:
            new_x -= 1
        elif rand_float < right + left:
            new_x += 1
        elif rand_float < up + right + left:
            new_y += 1
        else: # move down
            new_y -= 1
        # Ensure new point is not out of bounds
        new_point = Point(new_x, new_y, self.setpoint_target.position.z)
        # Only move to new point if in bounds, otherwise do nothing
        if self.is_point_in_cube(new_point, self.posCubeCorner1, self.posCubeCorner2):
            self.move(new_x, new_y, self.setpoint_target.position.z)


    def start_script(self):
        if self.set_current_src():
            self.set_new_dest()
            rospy.loginfo("Starting scripted control. Enter any command (valid or invalid) "
                          "into command line to stop script and switch to command line control")
            self.target_reached = True
            self.script_ctl = True
        else:
            rospy.logerr("Failed to start script control. Must start script control on a target")

    def setTakeoff(self, height=1.5):
        self.setArm()
        self.setOffboardMode()
        self.move(self.curr_position.x, self.curr_position.y, height)

    def setLand(self):
        try:
            rospy.wait_for_service('mavros/cmd/land', 0.5)
            landService = rospy.ServiceProxy('mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            landService()
        except rospy.ServiceException as e:
            rospy.logerr("Service land call failed: %s"%e)

    def setArm(self):
        try:
            rospy.wait_for_service('mavros/cmd/arming', 0.5)
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            rospy.logerr("Service arming call failed: %s"%e)

    def setDisarm(self):
        try:
            rospy.wait_for_service('mavros/cmd/arming', 0.5)
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            rospy.logerr("Service disarming call failed: %s"%e)

    def setOffboardMode(self):
        try:
            rospy.wait_for_service('mavros/set_mode', 0.5)
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            rospy.logerr("service set_mode call failed: %s. Offboard Mode could not be set."%e)

    def setKill(self):
        try:
            rospy.wait_for_service('mavros/cmd/command', 0.5)
            killService = rospy.ServiceProxy('mavros/cmd/command', mavros_msgs.srv.CommandLong)
            killService(command=400, confirmation=0, param1=0.0, param2=21196.0, param3=0.0,
                        param4=0.0, param5= 0.0, param6=0.0, param7=0.0)
        except rospy.ServiceException as e:
            rospy.logerr("Service disarming call failed: %s"%e)
