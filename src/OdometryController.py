#!/usr/bin/env python

#controller for the odometry. sets robot in motion.

import rospy
from nav_msgs.msg import Odometry
#from tf.transformations import euler_from_quaternion #### doesn't currently work
from geometry_msgs.msg import Point, Twist, Pose, Quaternion
from math import atan2, sqrt
from std_msgs.msg import String

class Controller:
    def __init__(self):
        self.subGoal=rospy.Subscriber("/CoordinationHandling", Point, self.updateNextCoord)
        self.subCurrPos=rospy.Subscriber("/position", Pose, self.distance)
        self.pubVel=rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.pubArrival=rospy.Publisher('/arrival', String, queue_size=10)

        self.speed=Twist()
        self.CurrPos=Point() #x,y,z. gets value from /position
        self.goal=Point() #gets value of NextCoord from subscriber
        self.alpha=float() #angle_to_goal
        self.yaw=float() #angle of z-axis
        self.PosDiff=float()
    
    def updateNextCoord(self, nextCoordVar):
        self.goal=nextCoordVar
        print("OdometryController.py: nextCoordVar is", nextCoordVar)
    
    def distance(self, CurrPosVar):
        self.CurrPos=CurrPosVar #[CurrPosVar[0], CurrPosVar[1], CurrPosVar[2]]
        x_diff=self.goal.Point.x - self.CurrPos.x
        y_diff=self.goal.Point.y - self.CurrPos.y

        self.alpha = atan2(y_diff, x_diff) #angle_to_goal
        self.PosDiff=sqrt((x_diff)**2 + (y_diff)**2)
    
    def driveToGoal(self):
        self.yaw=0.0
        if abs(self.alpha - self.yaw) < 0.1:
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.3 #yaw
        else:
            self.speed.linear.x = 0.3
            self.speed.angular.z = 0.0 #yaw'
        print("hi there")


def main():
    rospy.init_node('OdometryController')
    print("node OdometryController successfully initialised")
    Controller()
    rospy.spin()


if __name__=='__main__':
    main()
else:
    print("OdometryController.py shouldn't have been called!")


