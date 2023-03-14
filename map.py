#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion
from robotParams import *
import bresenham
import math

class OccupancyMap(object):
    def __init__(self):
        self.occupancy_map = np.ones((MAP_SIZE, MAP_SIZE)) / 2
        self.obstacle_radius = int(INFLATION_RADIUS / CELL_SIZE_METER) + 1
        self.pub = rospy.Publisher('occupancy_grid', OccupancyGrid, queue_size=10, latch=True)
        self.skip_inf = True # flag for inf scan values skiping mode

    def update_map(self, laser_scan: LaserScan, pose: Pose2D):
        ranger_pose = pose
        ranger_pose.x = math.cos(pose.theta) * LASER_SHIFT_METER + pose.x
        ranger_pose.y = math.sin(pose.theta) * LASER_SHIFT_METER + pose.y
        angle = laser_scan.angle_min
        for ray_range in laser_scan.ranges:
            if np.isinf(ray_range):
                if self.skip_inf:
                    ray_range = np.NaN
                else:
                    ray_range = laser_scan.range_max+1
            if not np.isnan(ray_range):
                empty_tiles, obstacle_tiles = bresenham.laser_through_tiles(ray_range, angle, ranger_pose, laser_scan.range_max)
                for tile in empty_tiles:
                    self.cell_probability_update(tile, False)
                for tile in obstacle_tiles:
                    self.cell_probability_update(tile, True)
            angle+=(laser_scan.angle_max - laser_scan.angle_min)/len(laser_scan.ranges)

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

    def publish_map(self):
        msg = OccupancyGrid()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'

        msg.info.width = self.occupancy_map.shape[1]
        msg.info.height = self.occupancy_map.shape[0]
        msg.info.resolution = CELL_SIZE_METER

        data = np.array(self.occupancy_map.flatten() * 100, dtype=np.int8)
        msg.data = list(data)
        self.pub.publish(msg)

    def inflate_single_obstacle(self, center):
        x,y = np.meshgrid(np.arange(self.occupancy_map.shape[1]), np.arange(self.occupancy_map.shape[0]))
        distance = np.sqrt((x - center[1])**2 + (y - center[0])**2)
        mask = distance <= self.obstacle_radius
        self.occupancy_map[mask] = 1

    def inflate_obstacles(self, threshold=0.51):
        obstacles = np.argwhere(self.occupancy_map >= threshold)
        for obstacle in obstacles:
            self.inflate_single_obstacle(obstacle)


if __name__ == '__main__':
    occupancy_map = OccupancyMap()
