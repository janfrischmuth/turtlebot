#!/usr/bin/env python

#load coordinates from YAML file, publish nextCoord to path planning node once arrived on nextCoord

import rospy
import sys
import yaml
from std_msgs.msg import String
from geometry_msgs.msg import Point

CoordCount = 0
z=0.0
nextCoord=[0.0, 0.0, z] #add yaw later
accuracy=0.1 #[m]???
poses=[0]

def getPoseFromYAML():
    global nextCoord,z, poses
    try:
        RowInPoseElement=poses[CoordCount]
        PoseInformation=RowInPoseElement[1]
        nextCoord = (PoseInformation[0], PoseInformation[1], z) #tupleIndices must be integer or slices, not tuple
        print(nextCoord, "is the nextPose from the YAML")
        rospy.loginfo(nextCoord, "is the next pose from the YAML file")
    except:
        print("getPoseFromYAML() failed!!!")
        rospy.loginfo("getPoseFromYAML() failed!")

def pubNextCoord(data):
    global CoordCount
    CoordCount+=1
    getPoseFromYAML()
    print("pubNextCoord was called")



def main():
    global poses, nextCoord
    poses=rospy.get_param("/poses")
    #print(poses[0])
    getPoseFromYAML()
    pub=rospy.Publisher('CoordinationHandling',Point,queue_size=10)
    rospy.init_node("ManagingCoordinates",anonymous=True)
    rospy.Subscriber("/arrival",String,pubNextCoord)
    #rate=rospy.Rate(4)#Hz
    # while not rospy.is_shutdown(): //if uncommented, tab everything below
        #rospy.Subscriber("/arrival",String,pubNextCoord)
    print("Publish nextCoord:", nextCoord) #change to rospy.loginfo() later
    pub.publish(nextCoord[0],nextCoord[1],nextCoord[2])
        #rate.sleep()
    rospy.spin()

if __name__=='__main__':
    main()
else:
    print("ManagingCoordinates.py shouldn't have been called!")