#!/usr/bin/env python

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion

MAP_SIZE = 300
CELL_SIZE_METER = 1

class OccupancyMap(object):
    def __init__(self):
        self.occupancy_map = np.ones((MAP_SIZE, MAP_SIZE)) / 2
    
    def update_map(self, laser_scan: LaserScan, pose: Pose2D):
        return

    def make_pose_2D(odometry_msg: Odometry):
        orientation_q = odometry_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        x = odometry_msg.pose.pose.position.x
        y = odometry_msg.pose.pose.position.y

        return Pose2D(x, y, yaw)
        
    def cell_probability_update(cell, p):
        return

        
if __name__ == '__main__':
    occupancy_map = OccupancyMap()
