#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from std_msgs.msg import String
#from KalmanPosition import RobCoord
#from ManageCoordinates import CoordCount, nextCoord
#from scripts.msg import PoseMsg

x = RobCoord[0]
y = RobCoord[1]
z = 0.0
roll = 0.0
pitch = 0.0
yaw = RobCoord[5]
accuracy=0.1
speed = Twist()
goal = Point()
goal.x = nextCoord[0]
goal.y = nextCoord[1]
goal.yaw = nextCoord[5]

def OdomFeedb(msg):
    global x,y,yaw
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll,pitch,yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def main():
    rospy.init_node("OdomController")
    rospy.Subscriber("/odom", Odometry,OdomFeedb) #("/position")
    pubVel = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size = 1)
    #pubManCoord = rospy.Publisher("/GoalReached",String) # queue_size????????
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        x_diff = goal.x - x
        y_diff = goal.y - y
        angle_to_goal = atan2(y_diff, x_diff)
        PosError=nextCoord-RobCoord
        while PosError < accuracy: #all of WHILE function should be eingerueckt
            if abs(angle_to_goal - yaw) > 0.3:
                speed.linear.x = 0.0
                speed.angular.z = 0.3 #yaw'
            else:
                speed.linear.x = 0.5
                speed.angular.z = 0.0 #yaw'
            #rospy.loginfo(speed)
            pubVel.publish(speed)
            r.sleep()  
        else:
            #rospy.loginfo("Robot aarived at next coordinate")
            pubManCoord.publish("Robot arrived at next coordinate")


if __name__=='__main__':
    main()
else:
    print("OdomController.py shouldn't have been called!")