#!/usr/bin/env python

#load coordinates from YAML file, publish nextCoord to path planning node once arrived on nextCoord

import rospy
import sys
import yaml
from std_msgs.msg import String
from KalmanPosition import RobCoord

CoordCount = 0
nextCoord=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#nextCoord=[None]*6
accuracy=0.1 #in [m]???
poses=[0]

def getPoseFromYAML():
    global nextCoord
    try:
        RowOfNextPose=poses[CoordCount]
        nextPose=RowOfNextPose[1]
        nextCoord=nextPose[0,1] #x and y component, add 5/yaw later
        print (nextCoord)
    except: 
        print("getPoseFromYAML failed!")

#def waitForArriving():
 #   rospy.Subscriber("CoordHandling",list)

def pubNextCoord(data):
    global nextCoord, CoordCount
    rate=rospy.Rate(10) #in [Hz]
    pub=rospy.Publisher('/CoordinationHandling',list)
    pub.publish(nextCoord)
    CoordCount += 1
    getPoseFromYAML()
    #rospy.loginfo(rospy.get_caller_id() + "Robot arrived at pose %s", data.data)
    rate.sleep()

def RobotArrived():
    rospy.Subscriber("GoalReached",String,pubNextCoord)


def main():
    global CoordCount, poses
    rospy.init_node('ManageCoordinates')
    rate=rospy.Rate(10) # in Hz
    poses=rospy.get_param("/poses")
    while not rospy.is_shutdown:
        #getPoseFromYAML()
        rospy.Subscriber("GoalReached",String,pubNextCoord) #every time robot arrives, publish next Coordinates
        rate.sleep()


if __name__=='__main__':
    main()
else:
    print("ManageCoordinates.py shouldn't have been called!")
