#!/usr/bin/env python

# get pose from odometry (and later Qualisys Camera System) and publish it to /position topic

import rospy
# import sys
from nav_msgs.msg import Odometry
#from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Quaternion, Pose  # , Twist
# from scripts.msg import CustomMsg


class OdomPose:
    def __init__(self):
        self.odomSub = rospy.Subscriber(
            "/odometry/filtered", Odometry, self.OdomCallback)
        self.pub = rospy.Publisher('/position', Pose, queue_size=10)
        self.OdomPoseVar = rospy.wait_for_message(
            "/qualisys/mobile_base/odom", Odometry)  # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.point = Point()
        self.quaternion = Quaternion()

    def OdomCallback(self, msg):
        self.point = msg.pose.pose.position
        #print("point is")
        # print(self.point)
        self.quaternion = msg.pose.pose.orientation
        #print("quaternions are")
        # print(self.quaternion)
        #self.pose = [self.point, self.quaternion]
        self.pub.publish(self.point, self.quaternion)
        # print("KalmanPos.py: Current pose is: ")#%s")%self.pose
        # print("----")


def main():
    rospy.sleep(3)
    rospy.init_node('KalmanPosition')
    print("node KalmanPosition successfully initialised")
    # rospy.sleep(1.5)
    self = OdomPose()
    print("KalmanPos.py: the starting pose is: ")  # %s") %abc.OdomPoseVar
    print(self.OdomPoseVar.pose.pose.position)
    rospy.spin()


if __name__ == '__main__':
    main()
else:
    print("KalmanPos.py shouldn't have been called!")
