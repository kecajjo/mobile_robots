#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy
import math
from bag_reader import BagReader
from map import OccupancyMap
from laserSub import LaserScanListener
from geometry_msgs.msg import Pose2D


class Plotter(object):
    def __init__(self):
        pass

    def plot_heatmap(self, intensity_map):
        plt.imshow(intensity_map, cmap='plasma')
        plt.colorbar()
        plt.show()

def map_from_bag(msg_count):
    bag_reader = BagReader()
    occupancy_map = OccupancyMap()
    scan_msgs = bag_reader.get_messages("/scan")
    pose_msgs = bag_reader.get_messages("/RosAria/pose")

    for i in range(msg_count):
        scan_msg = scan_msgs[i]
        pose2D_msg = occupancy_map.make_pose_2D(pose_msgs[i])
        occupancy_map.update_map(scan_msg, pose2D_msg)
        occupancy_map.publish_map()
    return occupancy_map

def map_from_json(msg_count):
    laser_scan = LaserScanListener()
    occupancy_map = OccupancyMap()

    for i in range(msg_count):
        scan_msg, pose = laser_scan.load_json_scan(i)
        pose2D_msg = Pose2D(pose[0], pose[1], math.radians(pose[2]))
        occupancy_map.update_map(scan_msg, pose2D_msg)
        occupancy_map.publish_map()
    return occupancy_map

if __name__ == '__main__':
    rospy.init_node('lab_5_main')
    plotter = Plotter()
    # occupancy_map = map_from_json(9)
    occupancy_map = map_from_bag(1500)
    plotter.plot_heatmap(occupancy_map.occupancy_map)
