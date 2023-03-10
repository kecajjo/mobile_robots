#!/usr/bin/env python

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion
from robotParams import *
import bresenham
import math

class OccupancyMap(object):
    def __init__(self):
        self.occupancy_map = np.ones((MAP_SIZE, MAP_SIZE)) / 2
    
    def update_map(self, laser_scan: LaserScan, pose: Pose2D):
        ranger_pose = pose
        ranger_pose.x = math.cos(pose.theta) * LASER_SHIFT_METER + pose.x
        ranger_pose.y = math.sin(pose.theta) * LASER_SHIFT_METER + pose.y 
        angle = laser_scan.angle_min
        for ray_range in laser_scan.ranges:
            if np.isinf(ray_range):
                ray_range = laser_scan.range_max+1
            if not np.isnan(ray_range):
                empty_tiles, obstacle_tiles = bresenham.laser_through_tiles(ray_range, angle, ranger_pose, laser_scan.range_max)
                for tile in empty_tiles:
                    self.cell_probability_update(tile, False)
                for tile in obstacle_tiles:
                    self.cell_probability_update(tile, True)
            angle+=laser_scan.angle_increment

    def make_pose_2D(self, odometry_msg: Odometry):
        orientation_q = odometry_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        x = odometry_msg.pose.pose.position.x
        y = odometry_msg.pose.pose.position.y

        return Pose2D(x, y, yaw)
        
    def cell_probability_update(self, cell: bresenham.grid_pos_t, p: bool):
        if p is True:
            update = 0.1
        else:
            update = -0.05
        self.occupancy_map[cell.grid_y][cell.grid_x] += update
        if self.occupancy_map[cell.grid_y][cell.grid_x] < 0:
            self.occupancy_map[cell.grid_y][cell.grid_x] = 0
        elif self.occupancy_map[cell.grid_y][cell.grid_x] > 1:
            self.occupancy_map[cell.grid_y][cell.grid_x] = 1

        
if __name__ == '__main__':
    occupancy_map = OccupancyMap()
