#!/usr/bin/env python

# run: python subscriber_scan.py 5  (where 5 is robot number)
import sys
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
import math
from robotParams import *

_SCAN_TOPIC = "/PIONIER"+ROBOT_NUMBER+"/scan"
_FRONT_ROBOT_POINT = 256

class LaserScanListener(object):
    def __init__(self):
        # Subscribe topics and bind with callback functions
        rospy.Subscriber(_SCAN_TOPIC, LaserScan, self.__callback_scan)
        self.__lp = lg.LaserProjection()
        self.pc2_msg = []

    def __callback_scan(self, msg):
        self.pc2_msg = self.__lp.projectLaser(msg)
        # convert it to a generator of the individual points

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