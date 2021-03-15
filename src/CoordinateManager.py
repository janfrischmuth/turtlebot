#!/usr/bin/env python

# load coordinates from YAML file, publish nextCoord to path planning node once arrived on nextCoord
import rospy
import sys
import yaml
from std_msgs.msg import String
from geometry_msgs.msg import Point


class CoordinateManager:
    def __init__(self):
        self.CoordCount = 0  # self. makes the var "global"
        self.z = 0.0
        self.poses = rospy.get_param("/poses")
        self.rate = rospy.Rate(10)  # Hz
        # self.accuracy=0.1 #[m]???
        self.nextCoord = Point()  # add yaw later
        self.pub = rospy.Publisher(
            '/CoordinationHandling', Point, queue_size=10)
        self.sub = rospy.Subscriber("/arrival", String, self.pubNextCoord)

    def getPoseFromYAML(self):
        try:
            RowInPoseElement = self.poses[self.CoordCount]
            PoseInformation = RowInPoseElement[1]
            # tupleIndices must be integer or slices, not tuple
            self.nextCoord = (PoseInformation[0], PoseInformation[1], self.z)
            print("The nextPose from the YAML is:")
            print(self.nextCoord)
            #rospy.loginfo(self.nextCoord, "is the next pose from the YAML file")
        except:
            print("getPoseFromYAML() failed!!!")
            rospy.loginfo("getPoseFromYAML() failed!")

    def pubNextCoord(self):  # ,Point)
        try:
            self.pub.publish(
                self.nextCoord[0], self.nextCoord[1], self.nextCoord[2])
            print("I published the nextCoord")
            self.CoordCount += 1
            self.getPoseFromYAML()
            print("the nextCoord is:")  # %s") %self.nextCoord
            print(self.nextCoord)
            self.rate.sleep()
        except:
            print("pubNextCoord() failed")


def main():
    rospy.init_node('CoordinateManager')
    print("node CoordinateManager successfully initialised")
    abc = CoordinateManager()
    abc.getPoseFromYAML()
    # abc.pub.publish()
    abc.pubNextCoord()
    # print("CoordinateManager.py: the nextCoord is:")# %s") %abc.nextCoord
    # print(abc.nextCoord)
    # while not rospy.is_shutdown:
    rospy.spin()


if __name__ == '__main__':
    main()
else:
    print("ManagingCoordinates.py shouldn't have been called!")
