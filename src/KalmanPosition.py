#!/usr/bin/env python

#get pose from odometry (and later Qualisys Camera System) and publish it to /position topic

import rospy
import sys
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
#from scripts.msg import PoseMsg

roll = pitch = yaw = 0.0
RobCoord = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def printOdomPos(msg): #via callback from RobCoord
    print (msg.pose.pose) #pose from odometry

def getOdomRobotPos(msg): #get coordinates and orientation
    global roll, pitch, yaw, RobCoord
    xCoord = msg.pose.pose.position.x
    yCoord = msg.pose.pose.position.y
    zCoord = msg.pose.pose.position.z
    orient_quaterion = msg.pose.pose.orientation
    orient_list = [orient_quaterion.x, orient_quaterion.y, orient_quaterion.z, orient_quaterion.w]
    (roll,pitch,yaw) = euler_from_quaternion(orient_list)
    RobCoord=[xCoord,yCoord,zCoord,roll,pitch,yaw]
    print (RobCoord)

#def weighPos():
    #add once connected with qualisys

def main():
    rospy.init_node('KalmanPosition')
    rospy.Subscriber("/odom",Odometry,getOdomRobotPos) #pose from odometry
    #QualSub = rospy.Subscriber('/qual_pos',data???) #pose from Qualisys camera system
    pub=rospy.Publisher('/position',Point)
    while not rospy.is_shutdown:
        #RobCoord=weighPos(OdomPos,QualPos)
        pub.publish(RobCoord)

        rospy.spin()

if __name__=='__main__':
    main()
else:
    print("KalmanPosition.py shouldn't have been called!")
