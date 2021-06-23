#!/usr/bin/env python

# controller for the odometry. sets robot in motion.

from math import atan2, sqrt  # , pi
import rospy
from sys import exit
# from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion  # doesn't currently work
from geometry_msgs.msg import Point, Twist, Pose  # , Quaternion
# from std_msgs.msg import String
from scripts.srv import *


class Controller:
    def __init__(self):
        # self.subGoal = rospy.Subscriber("/coordination_handling", Point, self.updateNextCoord)
        self.subCurrPos = rospy.Subscriber("/position", Pose, self.driveToGoal)
        self.pubVel = rospy.Publisher(
            '/cmd_vel_mux/input/navi', Twist, queue_size=10)
        # self.pubArrival = rospy.Publisher('/arrival', String, queue_size=10)

        self.send_next_coord_client = rospy.ServiceProxy(
            'coordinate_server', coordinate_managing)

        self.speed = Twist()
        self.CurrPos = Point()  # x,y,z. gets value from /position
        self.goal = Point()  # gets value of NextCoord from subscriber
        self.alpha = float()  # angle to goal
        self.angle_diff = float()
        self.roll = float()
        self.pitch = float()
        self.yaw = float()  # angle of z-axis
        self.PosDiff = float()

        self.coord_count = 0

        self.v_max_lin = 0.3  # max. linear velocity of turtlebot2/kobuki
        self.v_max_ang = 1  # 3.14  # in [rad/s], max. rotational velocity
        self.accuracy_dist = 0.1
        self.accuracy_ang = 0.17  # equals 10deg
        self.K_vel_lin = 1  # proportional gain
        self.K_vel_ang = 1.0  # proportional gain
        self.rate = rospy.Rate(10)  # Hz

    def updateNextCoord(self, nextCoordVar):
        self.goal = nextCoordVar
        print("OdometryController.py: I heard the goal is:")
        print(self.goal)
        print("------")

    def req_next_coord(self, coord_count):
        rospy.wait_for_service('/coordinate_server')
        try:
            req_next_coord_client = self.send_next_coord_client(
                self.coord_count)

            self.coord_count = req_next_coord_client.next_coord_num
            if req_next_coord_client.stop_robot is False:
                # self.goal = (req_next_coord_client.x,
                #             req_next_coord_client.y, req_next_coord_client.z)
                self.goal.x = req_next_coord_client.x
                self.goal.y = req_next_coord_client.y
                self.goal.z = 0  # '0' since 'z' var is 'nan' in yaml
                # print("OdometryController: I heard the goal is:")
                # print(self.goal)
            else:
                rospy.loginfo(
                    "Last coordinate reached, OdometryController exits now. Bye bye.")
                # raise SystemExit
                # sys.exit()
                # quit()
                rospy.is_shutdown()

        except rospy.ServiceException as e:
            print("Service call coordinate_server failed: %s" % e)

    def driveToGoal(self, CurrPosVar):
        # print("controller hear the pose is")
        # print(CurrPosVar)
        # print("----")
        self.CurrPos = CurrPosVar.position
        # print("CurrPos is ", self.CurrPos)

        # print("---------")
        # print("goal.x is", self.goal.x)
        # print("goal.y is", self.goal.y)
        # print("CurrPos.x is", self.CurrPos.x)
        # print("CurrPos.y is", self.CurrPos.y)
        x_diff = self.goal.x - self.CurrPos.x
        # print("x_diff is", x_diff)
        y_diff = self.goal.y - self.CurrPos.y
        # print("y_diff is", y_diff)
        self.PosDiff = sqrt((x_diff)**2 + (y_diff)**2)
        # print("PosDiff results in ", self.PosDiff)

        # %(2*pi)#angle_to_goal in [Bogenmass]
        self.alpha = (atan2(y_diff, x_diff))  # % (2*pi)
        # print("alpha is ", self.alpha)

        CurrOrient = CurrPosVar.orientation
        # else: TypeError: 'Quaternion' object has no attribute '__getitem__'
        CurrOrientQuaternion = [CurrOrient.x,
                                CurrOrient.y, CurrOrient.z, CurrOrient.w]

        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(
            CurrOrientQuaternion)
        self.angle_diff = (self.alpha - self.yaw)
        # self.angle_diff = (self.yaw-self.alpha) #should be incorrect
        # print("angle_diff is ", self.angle_diff)

        v_lin = self.PosDiff * self.K_vel_lin
        v_ang = self.angle_diff * self.K_vel_ang

        self.speed.linear.x = min(v_lin, self.v_max_lin)
        # self.speed.linear.x = v_lin
        # print("v_lin is set to ", self.speed.linear.x)

        if v_ang > 0.0:
            if v_ang < self.v_max_ang:
                self.speed.angular.z = v_ang
            else:
                self.speed.angular.z = self.v_max_ang
        elif v_ang < 0.0:
            if v_ang > -(self.v_max_ang):
                self.speed.angular.z = v_ang
            else:
                self.speed.angular.z = -(self.v_max_ang)

        # self.speed.angular.z = v_ang
        # print("v_ang is set to ", self.speed.angular.z)
        self.pubVel.publish(self.speed)
        # print("I published the speed at: ")
        # print(self.speed)
        if abs(self.PosDiff) < self.accuracy_dist:  # self.alpha < self.accuracy_ang +
            rospy.loginfo(
                "OdometryController.py: I arrived at\n %s" % self.goal)
            # print("OdometryController.py: I arrived at")
            # print(self.goal)
            print("------")
            self.req_next_coord(self.coord_count)
            print("------")
            # print(self.req_next_coord)
            # blabla


def main():
    rospy.sleep(3)
    rospy.init_node('OdometryController')
    print("node OdometryController successfully initialised")

    Controller()
    rospy.spin()


if __name__ == '__main__':
    main()
else:
    print("OdometryController.py shouldn't have been called!")
