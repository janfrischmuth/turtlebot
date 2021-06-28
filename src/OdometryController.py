#!/usr/bin/env python

# controller for the odometry. sets robot in motion.

from math import atan2, sqrt, pi, fmod
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
        # subscriber, publisher, serviceproxies
        self.subCurrPos = rospy.Subscriber(
            "/odometry/filtered", Odometry, self.driveToGoal)
        self.pubVel = rospy.Publisher(
            '/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.send_next_coord_client = rospy.ServiceProxy(
            'coordinate_server', coordinate_managing)
        # self.subGoal = rospy.Subscriber("/coordination_handling", Point, self.updateNextCoord)
        # self.pubArrival = rospy.Publisher('/arrival', String, queue_size=10)

        # declaration of some global variables
        self.speed = Twist()
        self.CurrPos = Point()  # x,y,z. gets /odometry/filtered value
        self.goal = Point()  # goal coordinate
        self.alpha = float()  # is the angle to goal
        self.angle_diff = float()  # angle to turn to goal
        self.roll = float()
        self.pitch = float()
        self.yaw = float()  # angle of z-axis
        self.PosDiff = float()
        self.coord_count = 0

        # params for turtlebot
        self.v_max_lin = 0.3  # max. linear velocity of turtlebot2/kobuki
        self.v_max_ang = 1.0  # 3.14  # in [rad/s], max. rotational velocity
        self.accuracy_dist = 0.1  # [m]
        self.accuracy_ang = 0.17  # [rad]. 0.17 rad = 10 deg
        self.K_vel_lin = 1.0  # proportional gain
        self.K_vel_ang = 1.0  # proportional gain
        self.rate = rospy.Rate(10)  # Hz

    ### obsolete ###
    def updateNextCoord(self, nextCoordVar):
        # sets next goal after arriving. callback from subGoal
        self.goal = nextCoordVar
        print("OdometryController.py: I heard the goal is:")
        print(self.goal)
        print("------")

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
                self.goal.z = 0  # '0' since 'z' var is 'nan' in yaml
                #print("OdometryController: I heard the goal is:\n%s" %self.goal)
            else:
                # doesn't work yet, fix this!!!
                rospy.loginfo(
                    "Last coordinate reached, OdometryController exits now. Bye bye.")
                # raise SystemExit
                # sys.exit()
                # quit()
                rospy.is_shutdown()

        except rospy.ServiceException as e:
            print("Service call coordinate_server failed: %s" % e)

    def driveToGoal(self, CurrPosVar):
        #####
        # calculates speed parameter for twist publisher
        #####
        self.CurrPos = CurrPosVar.pose.pose.position

        x_diff = self.goal.x - self.CurrPos.x
        y_diff = self.goal.y - self.CurrPos.y
        self.PosDiff = sqrt((x_diff)**2.0 + (y_diff)**2.0)

        CurrOrient = CurrPosVar.pose.pose.orientation
        CurrOrientQuaternion = [CurrOrient.x,
                                CurrOrient.y, CurrOrient.z, CurrOrient.w]

        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(
            CurrOrientQuaternion)
        self.yaw = angles.normalize_angle_positive(self.yaw)

        # alpha is the angle to goal in [Bogenmass]
        self.alpha = angles.normalize_angle_positive(
            atan2(y_diff, x_diff))  # % (2*pi)

        self.angle_diff = (self.alpha - self.yaw)
        #self.angle_diff = angles.shortest_angular_distance(self.alpha, self.yaw)

        v_lin = self.PosDiff * self.K_vel_lin
        v_ang = self.angle_diff * self.K_vel_ang

        # sets linear speed to max value if the calculated one is too big
        self.speed.linear.x = min(v_lin, self.v_max_lin)

        # sets angular speed to max value if the calculated one is too big
        # considers signed value (+ or -)
        if v_ang < pi:
            if v_ang < self.v_max_ang:
                self.speed.angular.z = v_ang
                print("------\nif if\nv_ang = %s" % v_ang)
                print("self.speed.angular.z = %s\n------" %
                      self.speed.angular.z)
            else:
                self.speed.angular.z = self.v_max_ang
                print("------\nif else\nv_ang = %s" % v_ang)
                print("self.speed.angular.z = %s\n------" %
                      self.speed.angular.z)
        elif v_ang > pi:
            v_ang -= pi
            if v_ang < self.v_max_ang:
                self.speed.angular.z = -v_ang
                print("------\nelif if\nv_ang = %s" % v_ang)
                print("self.speed.angular.z = %s\n------" %
                      self.speed.angular.z)
            else:
                self.speed.angular.z = -self.v_max_ang
                print("------\nelif else\nv_ang = %s" % v_ang)
                print("self.speed.angular.z = %s\n------" %
                      self.speed.angular.z)
        elif v_ang == 0.0:
            self.speed.angular.z = v_ang
            print("SURPRISE!!! v_ang = 0!!! NEVER HAPPENED BEFORE!!!")

        self.pubVel.publish(self.speed)

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

        if abs(self.PosDiff) < self.accuracy_dist:  # self.alpha < self.accuracy_ang +
            rospy.loginfo(
                "OdometryController.py: I arrived at\n %s" % self.goal)
            # print("OdometryController.py: I arrived at")
            # print(self.goal)
            print("------")
            self.req_next_coord(self.coord_count)
            print("------")
            # print(self.req_next_coord)


def main():
    rospy.sleep(3)
    rospy.init_node('OdometryController', log_level=rospy.INFO)
    print("node OdometryController successfully initialised")
    # function to set first coordinate? to prevent driving to (0,0) in the beginning!
    Controller()
    rospy.spin()


if __name__ == '__main__':
    main()
else:
    print("OdometryController.py shouldn't have been called!")
