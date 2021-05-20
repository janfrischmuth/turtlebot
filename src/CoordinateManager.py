#!/usr/bin/env python

# load coordinates from YAML file, publish nextCoord to path planning node once arrived on nextCoord
import rospy
# import sys
# import yaml
from std_msgs.msg import String
from geometry_msgs.msg import Point


class CoordinateManager:
    def __init__(self):
        self.CoordCount = 0  # self. makes the var "global"
        #self.z = 0.0
        self.poses = rospy.get_param("/poses")
        self.rate = rospy.Rate(10)  # Hz
        # self.accuracy=0.1 #[m]???
        self.nextCoord = Point()  # add yaw later
        self.pub = rospy.Publisher(
            "/coordination_handling", Point, queue_size=10)
        self.sub = rospy.Subscriber("/arrival", String, self.pubNextCoord)

    def getPoseFromYAML(self):
        try:
            RowInPoseElement = self.poses[self.CoordCount]
            PoseInformation = RowInPoseElement[1]
            # tupleIndices must be integer or slices, not tuple
            self.nextCoord = (PoseInformation[0], PoseInformation[1], 0.0)
            self.pub.publish(
                self.nextCoord[0], self.nextCoord[1], self.nextCoord[2])
            # print("The nextPose from the YAML is:")
            # print(self.nextCoord)
            # rospy.loginfo(self.nextCoord, "is the next pose from the YAML file")
            # here: added from previous pubNextCoord which used to be callback from /arrival
        except:
            print("getPoseFromYAML() failed!!!")
            rospy.loginfo("getPoseFromYAML() failed!")

    def pubNextCoord(self, msg):  # , self.nextCoord):  # ,Point)
        msg_var = msg.data
        print("------")
        print("CoordinateManager.py: From the TurtleBot I heard")
        print(msg_var)
        try:
            self.pub.publish(
                self.nextCoord[0], self.nextCoord[1], self.nextCoord[2])
            print("CoordinateManager.py: I published the nextCoord")
            self.CoordCount += 1
            self.getPoseFromYAML()
            # %s") %self.nextCoord
            print("CoordinateManager.py: the nextCoord is:")
            print(self.nextCoord)
            print("------")
            self.rate.sleep()
        except:
            print("pubNextCoord() failed")


def main():
    rospy.sleep(3)
    rospy.init_node('CoordinateManager')
    print("node CoordinateManager successfully initialised")
    # self=CoordinateManager()
    # rospy.sleep(1.5)
    self = CoordinateManager()
    # rospy.sleep(1.5)  # so that the getPoseFromYAML message will be received
    self.getPoseFromYAML()
    # while not rospy.is_shutdown:
    # self.getPoseFromYAML()
    # rospy.spin()
    # self.rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()
else:
    print("ManagingCoordinates.py shouldn't have been called!")
