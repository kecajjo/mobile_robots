#!/usr/bin/env python

# run: python subscriber_scan.py 5  (where 5 is robot number)
import sys
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from robotParams import *

_SONAR_TOPIC = "/PIONIER"+ROBOT_NUMBER+"/RosAria/sonar_pointcloud2"

class SonarScanListener(object):
    pc = []
    def __init__(self):
        # Subscribe topics and bind with callback functions
        rospy.Subscriber(_SONAR_TOPIC, PointCloud2, self.__callback_scan)
        self.pc2_msg = []

    def __callback_scan(self, msg):
        ## extract ranges from message
        self.pc2_msg = msg
        
    def get_point_generator(self):
        if self.pc2_msg != []:
            return pc2.read_points(self.pc2_msg)
        else:
            return []

    def get_point_list(self):
        if self.pc2_msg != []:
            return pc2.read_points_list(self.pc2_msg)
        else:
            return []

if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.stderr.write('Usage: sys.argv[0] \n')
        sys.exit(1)

    nr=sys.argv[1]
    listener()  