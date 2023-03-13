#!/usr/bin/env python

# run: python subscriber_scan.py 5  (where 5 is robot number)
import rospy
import pickle
import json
from sensor_msgs.msg import LaserScan
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
from robotParams import *

_SCAN_TOPIC = "/PIONIER"+ROBOT_NUMBER+"/scan"
_FRONT_ROBOT_POINT = 256

class LaserScanListener(object):
    def __init__(self):
        # Subscribe topics and bind with callback functions
        self.__sub = rospy.Subscriber(_SCAN_TOPIC, LaserScan, self.__callback_scan)
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

    def load_json_scan(self, i=4):
        json_data = open('data/map_boxes_1.json')
        data = json.load(json_data)
        with open("data/message.pickle", "rb") as f:
                msg = pickle.load(f)
        msg.ranges = data[i]["scan"]
        # self.__callback_scan(msg)
        return msg, data[i]["pose"]
