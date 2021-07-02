#!/usr/bin/env python

# controller for the odometry. sets robot in motion.

from math import atan2, atan, sqrt, pi, fmod
import rospy
from sys import exit
# from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion  # doesn't currently work
from geometry_msgs.msg import Point, Twist, Pose  # , Quaternion
from nav_msgs.msg import Odometry
# from std_msgs.msg import String
from scripts.srv import *
import angles


class Controller:
    def __init__(self):
        # params for turtlebot
        # see config file for more information
        # ------
        # log_level_config gets set in main ...
        # ... because Controller won't be initialized before setting it
        self.localization_topic = rospy.get_param("/localization_topic")
        self.v_max_lin = rospy.get_param("/v_max_lin")
        self.v_max_ang = rospy.get_param("/v_max_ang")
        self.accuracy_dist = rospy.get_param("/accuracy_dist")  # [m]
        self.accuracy_ang = rospy.get_param(
            "/accuracy_ang")  # [rad]. 0.17 rad = 10 deg
        self.K_vel_lin = rospy.get_param("/K_vel_lin")
        self.K_vel_ang = rospy.get_param("/K_vel_ang")
        self.K_vel_rot = rospy.get_param("/K_vel_rot")
        self.rate = rospy.Rate(10)  # Hz

        # declaration of some global variables
        self.speed = Twist()
        self.Pose = Pose()
        self.CurrPos = Point()  # x,y,z. gets /odometry/filtered value
        self.goal = Point()  # goal coordinate
        self.yaw_goal = float()
        self.alpha = float()  # is the angle to goal
        self.angle_diff = float()  # angle to turn to goal
        self.roll = float()
        self.pitch = float()
        self.yaw = float()  # angle of z-axis
        self.PosDiff = float()
        self.coord_count = 0
        self.distance_okay = False
        self.yaw_okay = False

        # subscriber, publisher, serviceproxies
        self.subCurrPos = rospy.Subscriber(
            self.localization_topic, Odometry, self.driveToGoal)
        self.pubVel = rospy.Publisher(
            '/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.send_next_coord_client = rospy.ServiceProxy(
            'coordinate_server', coordinate_managing)
        # self.subGoal = rospy.Subscriber("/coordination_handling", Point, self.updateNextCoord)
        # self.pubArrival = rospy.Publisher('/arrival', String, queue_size=10)

    def req_next_coord(self, coord_count):
        #####
        # requests next coordinate from coordinate server
        #####
        rospy.wait_for_service('/coordinate_server')
        try:
            req_next_coord_client = self.send_next_coord_client(
                self.coord_count)

            self.coord_count = req_next_coord_client.next_coord_num

            if req_next_coord_client.stop_robot is False:
                self.goal.x = req_next_coord_client.x
                self.goal.y = req_next_coord_client.y
                self.yaw_goal = req_next_coord_client.yaw
            else:
                rospy.loginfo(
                    "\nOdoemtryController:\n Last coordinate reached, OdometryController exits now.\
                    \n Bye bye.\n-------\n-------")
                rospy.signal_shutdown('finished')

        except rospy.ServiceException as e:
            rospy.logerr("\nService call coordinate_server failed: %s" % e)

    def turn_to_yaw_goal(self, pose_data):
        # turns robot to desired yaw after arriving at coordinate
        self.update_Orientation(pose_data)
        # print("pose_data = %s" % pose_data)
        yaw_diff = angles.normalize_angle_positive(self.yaw_goal - self.yaw)
        # print(" yaw_goal = %s\n yaw = %s\n yaw_diff = %s" %(self.yaw_goal, self.yaw, yaw_diff))

        if abs(yaw_diff) < (self.accuracy_ang) or \
           (abs(yaw_diff > (2.0 * pi - self.accuracy_ang))):
            self.speed.angular.z = 0
            self.yaw_okay = True
            self.pubVel.publish(self.speed)
            # print("yaw_diff = %s" % yaw_diff)
        else:
            # print("------\n yaw_goal = %s\n yaw = %s\n yaw_diff = %s"
            # % (self.yaw_goal, self.yaw, yaw_diff))
            v_ang = yaw_diff * self.K_vel_ang
            # print(" v_ang = %s" % v_ang)
            self.set_angular_vel(v_ang)
            self.speed.angular.z *= self.K_vel_rot
            # print("yaw_goal = %s\n yaw = %s \n yaw_diff = %s\n v_ang = %s\n------"
            #      % (self.yaw_goal, (self.yaw*(180/pi)), (yaw_diff*(180/pi)), self.speed.angular.z))
            self.pubVel.publish(self.speed)

    def update_Orientation(self, pose_data):
        CurrOrient = pose_data.pose.pose.orientation
        CurrOrientQuaternion = [CurrOrient.x,
                                CurrOrient.y, CurrOrient.z, CurrOrient.w]

        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(
            CurrOrientQuaternion)
        self.yaw = angles.normalize_angle_positive(self.yaw)
        # print("update_Orientation: yaw = %s" % self.yaw)

    def go_to_goal_coord(self, pose_data):
        self.CurrPos = pose_data.pose.pose.position

        x_diff = self.goal.x - self.CurrPos.x
        y_diff = self.goal.y - self.CurrPos.y
        self.PosDiff = sqrt((x_diff)**2.0 + (y_diff)**2.0)

        self.update_Orientation(pose_data)

        # alpha is the angle to goal in [Bogenmass]
        # atan2(y,x) takes 2 arguments, atan(x) just 1 argument
        self.alpha = angles.normalize_angle_positive(
            atan2(y_diff, x_diff))  # % (2*pi)

        self.angle_diff = (self.alpha - self.yaw)
        # self.angle_diff = fmod((self.alpha - self.yaw), pi)
        # self.angle_diff = angles.shortest_angular_distance(self.alpha, self.yaw)

        v_lin = self.PosDiff * self.K_vel_lin
        # x to the power of 3 to have smoother v_ang when small angle_diff
        # v_ang = (self.angle_diff)**3 * self.K_vel_ang
        v_ang = self.angle_diff * self.K_vel_ang

        # sets linear speed to max value if the calculated one is too big
        self.speed.linear.x = min(v_lin, self.v_max_lin)
        # sets angular speed
        self.set_angular_vel(v_ang)
        # print(" angle_diff = %s\n v_ang = %s\n------" \
        # % (self.angle_diff, self.speed.angular.z))

        # infos just for debugging
        # DON'T log each var into own function call. is too heavy performance-wise
        rospy.logdebug("\n goal #%s = ( % s , % s )\
                            \n x = % s [m]\
                            \n y = % s [m]\
                            \n PosDiff = % s [m]\
                            \n yaw = % s [deg]\
                            \n alpha = % s [deg]\
                            \n angle_diff = % s [deg]\
                            \n v_lin = % s [m/s]\
                            \n v_ang = % s [rad/s]\
                            \n------\
                            \n------"
                       % (self.coord_count, self.goal.x, self.goal.y,
                          self.CurrPos.x,
                          self.CurrPos.y,
                          self.PosDiff,
                          (self.yaw * 180.0/pi),
                           (self.alpha * 180.0/pi),
                          (self.angle_diff*180.0/pi),
                           self.speed.linear.x,
                           self.speed.angular.z))
        # these might be interesting, too
        # rospy.logdebug("x_diff = %s" % x_diff)
        # rospy.logdebug("y_diff = %s" % y_diff)

        if abs(self.PosDiff) < self.accuracy_dist:
            # coordinates reached, only rotating now
            self.speed.linear.x = 0
            self.pubVel.publish(self.speed)
            self.distance_okay = True
            # print("distance to goal = %s" % self.PosDiff)
        else:
            # goal not reached, publish velocity
            self.pubVel.publish(self.speed)

    def set_angular_vel(self, v_ang):
        # sets angular speed to max value if the calculated one is too big
        # considers signed value (+ or -)
        if v_ang > 0:
            if v_ang < pi:
                # ... 0 < v_ang < 3.14 < ...
                if v_ang < self.v_max_ang:
                    self.speed.angular.z = v_ang
                    # print("------\n+if if\nv_ang = %s" % v_ang)
                    # print("self.speed.angular.z = %s\n------" %self.speed.angular.z)
                else:
                    self.speed.angular.z = self.v_max_ang
                    # print("------\n+if else\nv_ang = %s" % v_ang)
                    # print("self.speed.angular.z = %s\n------" %self.speed.angular.z)
            elif v_ang > pi:
                # ... < 0 < 3.14 < v_ang < ...
                v_ang -= (2.0 * pi)
                if abs(v_ang) < self.v_max_ang:
                    self.speed.angular.z = v_ang
                    # print("------\n+elif if\nv_ang = %s" % v_ang)
                    # print("self.speed.angular.z = %s\n------" %self.speed.angular.z)
                else:
                    self.speed.angular.z = -self.v_max_ang
                    # print("------\n+elif else\nv_ang = %s" % v_ang)
                    # print("self.speed.angular.z = %s\n------" %self.speed.angular.z)
        elif v_ang < 0:
            if v_ang < - pi:
                # ... < v_ang < -3.14 < 0 < ...
                v_ang += (2.0 * pi)
                if v_ang < self.v_max_ang:
                    self.speed.angular.z = v_ang
                    # print("------\n-if if\nv_ang = %s" % v_ang)
                    # print("self.speed.angular.z = %s\n------" %self.speed.angular.z)
                else:
                    #
                    self.speed.angular.z = self.v_max_ang
                    # print("------\n-if else\nv_ang = %s" % v_ang)
                    # print("self.speed.angular.z = %s\n------" %self.speed.angular.z)
            else:
                # ... < 3.14 < v_ang < 0 < ...
                if v_ang > (-self.v_max_ang):
                    self.speed.angular.z = v_ang
                    # print("------\n-else if\nv_ang = %s" % v_ang)
                    # print("self.speed.angular.z = %s\n------" %self.speed.angular.z)
                else:
                    self.speed.angular.z = -self.v_max_ang
                    # print("------\n-else else\nv_ang = %s" % v_ang)
                    # print("self.speed.angular.z = %s\n------" %self.speed.angular.z)
        elif v_ang == 0.0:
            self.speed.angular.z = v_ang
            print("SURPRISE!!! v_ang = 0!!! NEVER HAPPENED BEFORE!!!")

        if self.speed.angular.z > self.v_max_ang or self.speed.angular.z < -self.v_max_ang:
            print("------\n------\nv_ang = %s\nspeed.angular.z = %s" %
                  (v_ang, self.speed.angular.z))

    def driveToGoal(self, curr_pose_data):
        # calculates speed parameter for twist publisher
        #
        # drive to coordinate first
        if self.distance_okay == False:
            self.go_to_goal_coord(curr_pose_data)
        # then turn to desired yaw
        elif self.distance_okay == True and self.yaw_okay == False:
            self.turn_to_yaw_goal(curr_pose_data)
        # arrival. get next pose
        elif self.distance_okay == True and self.yaw_okay == True:
            rospy.loginfo(
                "\nOdometryController.py:\nI arrived at\n x = %s\n y = %s\
                    \n yaw = %s\n PosDiff = %s\n------\n------"
                % (self.goal.x, self.goal.y, (self.yaw_goal * 180/pi), self.PosDiff))
            self.distance_okay = False
            self.yaw_okay = False
            self.req_next_coord(self.coord_count)
            print("------")
            # print(self.req_next_coord)


def main():
    # sleep for easier storing the laptop on the turtlebot
    # rospy.sleep(3)
    verbosity_level = rospy.get_param("/verbosity_level")
    if verbosity_level == "INFO":
        rospy.init_node('OdometryController', log_level=rospy.INFO)
    elif verbosity_level == "DEBUG":
        rospy.init_node('OdometryController', log_level=rospy.DEBUG)
    print("node OdometryController successfully initialised")
    # function to set first coordinate? to prevent driving to (0,0) in the beginning!
    controller = Controller()
    Controller.req_next_coord(controller, controller.coord_count)
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    main()
else:
    print("OdometryController.py shouldn't have been called!")
