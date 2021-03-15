#!/usr/bin/env python

# get pose from odometry (and later Qualisys Camera System) and publish it to /position topic

import rospy
import sys
from nav_msgs.msg import Odometry
#from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Quaternion, Twist, Pose
from scripts.msg import CustomMsg


class OdomPose:
    def __init__(self):
        self.odomSub = rospy.Subscriber("/odom", Odometry, self.OdomCallback)
        self.pub = rospy.Publisher('/position', Pose, queue_size=10)
        self.OdomPoseVar = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pose = Pose()
        self.point = Point()
        self.quaternion = Quaternion()

    def OdomCallback(self, msg):
        self.point = msg.pose.pose.position
        self.quaternion = msg.pose.pose.orientation
        #self.euler = euler_from_quaternion(quaternion)
        # print(self.euler)
        # self.roll=self.euler[0]
        # self.pitch=self.euler[1]
        # self.yaw=self.euler[2]
        #self.OdomPoseVar=[self.x, self.y, self.z, self.roll, self.pitch, self.yaw]
        self.pose = [self.point, self.quaternion]
        self.pub.publish(self.point, self.quaternion)
        # print("KalmanPos.py: Current pose is: ")#%s")%self.pose
        # print(self.pose)


def main():
    rospy.init_node('KalmanPosition')
    print("node KalmanPosition successfully initialised")
    abc = OdomPose()
    print("the starting pose is: ")  # %s") %abc.OdomPoseVar
    print(abc.OdomPoseVar)
    rospy.spin()


if __name__ == '__main__':
    main()
else:
    print("KalmanPos.py shouldn't have been called!")
