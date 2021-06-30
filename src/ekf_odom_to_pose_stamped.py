#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


######
# only purpose of this script is to
# convert a Odometry nav_msgs to a geometry_msgs PoseStamped message
######


class PublisherConverter:
    def __init__(self):
        self.sub = rospy.Subscriber(
            "/odometry/filtered", Odometry, self.callback)
        self.pub = rospy.Publisher(
            '/ekf_odom_to_pose_stamped', PoseStamped, queue_size=10)
        self.pose = PoseStamped()

    def callback(self, data):
        # header
        self.pose.header.seq = data.header.seq
        self.pose.header.stamp = data.header.stamp
        self.pose.header.frame_id = data.header.frame_id

        # point
        self.pose.pose.position = data.pose.pose.position
        #self.pose.pose.position.x = data.pose.pose.position.x
        #self.pose.pose.position.y = data.pose.pose.position.y
        #self.pose.pose.position.z = data.pose.pose.position.z

        # quaternion
        self.pose.pose.orientation = data.pose.pose.orientation
        #self.pose.pose.orientation.x = data.pose.pose.orientation.x
        #self.pose.pose.orientation.y = data.pose.pose.orientation.y
        #self.pose.pose.orientation.z = data.pose.pose.orientation.z
        #self.pose.pose.orientation.w = data.pose.pose.orientation.w

        self.pub.publish(self.pose)
        print(self.pose)


def main():
    rospy.init_node('odom_filtered_msg_to_posestamped', anonymous=False)
    PublisherConverter()
    rospy.spin()


if __name__ == '__main__':
    main()
