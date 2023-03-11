#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy
from bag_reader import BagReader
from map import OccupancyMap


class Plotter(object):
    def __init__(self):
        pass

    def plot_heatmap(self, intensity_map):
        plt.imshow(intensity_map, cmap='plasma')
        plt.colorbar()
        plt.show()

if __name__ == '__main__':
    rospy.init_node('lab_5_main')
    plotter = Plotter()
    bag_reader = BagReader()
    scan_msgs = bag_reader.get_messages("/scan")
    pose_msgs = bag_reader.get_messages("/RosAria/pose")

    occupancy_map = OccupancyMap()

    for i in range(700):
        scan_msg = scan_msgs[i]
        pose2D_msg = occupancy_map.make_pose_2D(pose_msgs[i])
        occupancy_map.update_map(scan_msg, pose2D_msg)
        occupancy_map.publish_map()
    plotter.plot_heatmap(occupancy_map.occupancy_map)
