#!/usr/bin/env python

#load coordinates from YAML file, publish nextCoord to path planning node once arrived on nextCoord

import rospy
import sys
import yaml
from std_msgs.msg import String
from geometry_msgs.msg import Point
#from KalmanPosition import RobCoord

CoordCount = 0
z=0
nextCoord=[0.0, 0.0, z] #add yaw later
accuracy=0.1 #in [m]???
poses=[0]


def getPoseFromYAML():
    global nextCoord, z
    try:
        RowOfNextPose=poses[CoordCount]
        nextPose=RowOfNextPose[1]
        nextCoord=nextPose[0,1,z] #x and y component. z for geometry_msgs/Point usage. Add yaw(on 5th position) at a later time
        rospy.loginfo(nextCoord + "is set as next published nextCoord") #print(nextCoord)
    except: 
        print("getPoseFromYAML() failed!")

def pubNextCoord(nextCoord):
    global CoordCount#, nextCoord
    rate=rospy.Rate(10) #in [Hz]
    pub=rospy.Publisher('/CoordinationHandling', Point, queue_size=10)
    pub.publish(nextCoord)
    rospy.loginfo(nextCoord + "is where the robot will now drive to")
    CoordCount += 1
    getPoseFromYAML()
    #rospy.loginfo(rospy.get_caller_id() + "Robot arrived at pose %s", data.data)
    rate.sleep()

def RobotArrived():
    rospy.Subscriber("Arrival",String,pubNextCoord)



def main():
    global CoordCount, poses
    rospy.init_node('ManageCoordinates')
    rate=rospy.Rate(10) # in Hz
    poses=rospy.get_param("/poses")
    #getPoseFromYAML()
    while not rospy.is_shutdown:
        #getPoseFromYAML()
        pubNextCoord(nextCoord)
        rospy.Subscriber("Arrival",String,pubNextCoord) #every time robot arrives, publish next Coordinates
        rate.sleep()


if __name__=='__main__':
    main()
else:
    print("ManageCoordinates.py shouldn't have been called!")
