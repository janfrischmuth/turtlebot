#!/usr/bin/env python
import rospy
from math import pi
from scripts.srv import *


class coordinate_server_class:
    def __init__(self):
        # global vars
        self.coord_count = 0
        self.stop_robot = False
        self.pose_amount = int
        # get poses from file
        self.poses = rospy.get_param("/poses")
        # initialize service
        self.srv = rospy.Service('coordinate_server',
                                 coordinate_managing, self.send_next_coord)

    def send_next_coord(self, req):
        # callback for managing the coordinates

        if req.curr_coord_num == 0:
            # sends first coordinate
            row_in_pose_element = self.poses[self.coord_count]
            pose_information = row_in_pose_element[1]
            x = pose_information[0]
            y = pose_information[1]
            yaw = pose_information[5] * (pi / 180)

            last_row = self.poses[-1]
            self.pose_amount = last_row[0]
            print('------')
            rospy.loginfo('\ncoordinate_server:\nThe turtlebot will now drive to %s different coordinates\n------' %
                          self.pose_amount)

            rospy.loginfo('\ncoordinate_server:\nDestination #%s is\n x = %s\n y = %s\n yaw = %s'
                          % (self.coord_count+1, x, y, (yaw*(180/pi))))  # logs destination and coordinate.

            self.coord_count += 1

            return{'next_coord_num': self.coord_count, 'x': x, 'y': y, 'yaw': yaw, 'stop_robot': self.stop_robot}

        elif req.curr_coord_num < self.pose_amount:
            # sends next coordinate
            row_in_pose_element = self.poses[self.coord_count]
            pose_information = row_in_pose_element[1]
            x = pose_information[0]
            y = pose_information[1]
            yaw = pose_information[5] * (pi / 180)

            rospy.loginfo('\ncoordinate_server:\nDestination #%s is\n x = %s\n y = %s\n yaw = %s' % (
                self.coord_count+1, x, y, (yaw*(180/pi))))  # logs destination and coordinate

            self.coord_count = req.curr_coord_num + 1

            return{'next_coord_num': self.coord_count, 'x': x, 'y': y, 'yaw': yaw, 'stop_robot': self.stop_robot}

        elif req.curr_coord_num == self.pose_amount:
            # when last pose is reached
            self.stop_robot = True

            rospy.loginfo(
                '\ncoordinate_server:\nLast coordinate reached! Bye bye.\n-------')

            return{'next_coord_num': self.coord_count, 'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'stop_robot': self.stop_robot}

        elif req.curr_coord_num > self.pose_amount:
            # ERROR: requested coordinate is higher than total available
            rospy.logerr(
                'Last coordinate in list already reached.\nServer will be closed.\n-------')
            exit()


def main():
    # sleep for easier storing the laptop on the turtlebot
    # rospy.sleep(3)
    rospy.init_node('coordinate_server')
    print("node coordinate_server successfully initialised")
    coordinate_server_class()
    rospy.spin()


if __name__ == '__main__':
    main()
else:
    print("ManagingCoordinates.py shouldn't have been called!")
