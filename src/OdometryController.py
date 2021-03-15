#!/usr/bin/env python

# controller for the odometry. sets robot in motion.

from math import atan2, sqrt
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion  # doesn't currently work
from geometry_msgs.msg import Point, Twist, Pose  # , Quaternion
from std_msgs.msg import String


class Controller:
    def __init__(self):
        self.subGoal = rospy.Subscriber(
            "/CoordinationHandling", Point, self.updateNextCoord)
        self.subCurrPos = rospy.Subscriber("/position", Pose, self.driveToGoal)
        self.pubVel = rospy.Publisher(
            '/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.pubArrival = rospy.Publisher('/arrival', String, queue_size=10)

        self.speed = Twist()
        self.CurrPos = Point()  # x,y,z. gets value from /position
        self.goal = Point()  # gets value of NextCoord from subscriber
        self.alpha = float()  # angle to goal
        self.roll = float()
        self.pitch = float()
        self.yaw = float()  # angle of z-axis
        self.PosDiff = float()

        self.rate = rospy.Rate(10)  # Hz

    def updateNextCoord(self, nextCoordVar):
        print("I heard ", nextCoordVar.x)
        self.goal = nextCoordVar
        print("OdometryController.py: nextCoordVar is", nextCoordVar)

    def driveToGoal(self, CurrPosVar):
        self.CurrPos = CurrPosVar.position
        print("CurrPos is ", self.CurrPos)
        CurrOrient = CurrPosVar.orientation
        # else: TypeError: 'Quaternion' object has no attribute '__getitem__'
        CurrOrientQuaternion = [CurrOrient.x,
                                CurrOrient.y, CurrOrient.z, CurrOrient.w]
        K_vel_lin = 0.67  # proportional gain
        K_vel_ang = 0.67  # proportional gain

        x_diff = self.goal.x - self.CurrPos.x
        print("x_diff is", x_diff)
        y_diff = self.goal.y - self.CurrPos.y
        print("y_diff is", y_diff)
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(
            CurrOrientQuaternion)

        self.alpha = atan2(y_diff, x_diff)  # angle_to_goal
        print("alpha is ", self.alpha)
        self.PosDiff = sqrt((x_diff)**2 + (y_diff)**2)
        print("PosDiff is ", self.PosDiff)
        v_lin = self.PosDiff * K_vel_lin
        v_ang = self.alpha * K_vel_ang

        self.speed.linear.x = v_lin
        self.speed.angular.z = v_ang
        self.pubVel.publish(self.speed)
        print("I published the speed at: ")
        print(self.speed)
        if abs(self.alpha - self.yaw) < 0.1 and abs(self.PosDiff) < 10:
            self.pubArrival.publish("I arrived")


def main():
    rospy.init_node('OdometryController')
    print("node OdometryController successfully initialised")
    abc = Controller()
    rospy.spin()
    # abc.rate.sleep()


if __name__ == '__main__':
    main()
else:
    print("OdometryController.py shouldn't have been called!")
