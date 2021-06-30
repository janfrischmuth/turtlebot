#!/usr/bin/env python
import rospy
from scripts.srv import *


class coordinate_server_class:
    def __init__(self):
        self.srv = rospy.Service('coordinate_server',
                                 coordinate_managing, self.send_next_coord)
        self.poses = rospy.get_param("/poses")
        self.coord_count = 0
        self.stop_robot = False
        self.pose_amount = int

    def send_next_coord(self, req):
        # callback for managing the coordinates

        #rospy.loginfo('serv_callback was called')

        if req.curr_coord_num == 0:
            #print("coord count is")
            # print(self.coord_count)

            row_in_pose_element = self.poses[self.coord_count]
            pose_information = row_in_pose_element[1]
            x = pose_information[0]
            y = pose_information[1]
            z = pose_information[2]

            last_row = self.poses[-1]
            self.pose_amount = last_row[0]
            print('------')
            rospy.loginfo('\ncoordinate_server:\nThe turtlebot will now drive to %s different coordinates\n------' %
                          self.pose_amount)

            rospy.loginfo('\ncoordinate_server:\nDestination #%s is\n x = %s\n y = %s' % (
                self.coord_count+1, x, y))  # logs destination and coordinate. z is .NaN -> z=0

            # print('coordinate_server: Destination #%s is' %(self.coord_count+1))
            # print('x = %s' % x)
            # print('y = %s' % y)
            # print('z = 0')  # 'z' is 'nan' in yaml
            # print('------')

            self.coord_count += 1

            return{'next_coord_num': self.coord_count, 'x': x, 'y': y, 'z': z, 'stop_robot': self.stop_robot}

        elif req.curr_coord_num < self.pose_amount:

            row_in_pose_element = self.poses[self.coord_count]
            pose_information = row_in_pose_element[1]
            x = pose_information[0]
            y = pose_information[1]
            z = pose_information[2]

            rospy.loginfo('\ncoordinate_server:\nDestination #%s is\n x = %s\n y = %s' % (
                self.coord_count+1, x, y))  # logs destination and coordinate

            self.coord_count = req.curr_coord_num + 1

            return{'next_coord_num': self.coord_count, 'x': x, 'y': y, 'z': z, 'stop_robot': self.stop_robot}

        elif req.curr_coord_num == self.pose_amount:
            self.stop_robot = True

            rospy.loginfo(
                '\ncoordinate_server:\nLast coordinate reached! Bye bye.\n-------')

            return{'next_coord_num': self.coord_count, 'x': 0.0, 'y': 0.0, 'z': 0.0, 'stop_robot': self.stop_robot}

        elif req.curr_coord_num > self.pose_amount:
            rospy.logerr(
                'Last coordinate in list already reached.\nServer will be closed.\n-------')
            exit()


def main():
    rospy.sleep(3)
    rospy.init_node('coordinate_server')
    print("node coordinate_server successfully initialised")
    coordinate_server_class()
    rospy.spin()


if __name__ == '__main__':
    main()
else:
    print("ManagingCoordinates.py shouldn't have been called!")
