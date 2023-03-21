#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy
from threading import Lock
from copy import deepcopy
from map import OccupancyMap
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
import path_planning
import bresenham
from robotParams import *

_SCAN_TOPIC = "/PIONIER"+ROBOT_NUMBER+"/scan"
_POSE_TOPIC = "/PIONIER"+ROBOT_NUMBER+"/RosAria/pose"
_CMD_VEL_TOPIC = "/PIONIER"+ROBOT_NUMBER+"/RosAria/cmd_vel"
_FRONT_ROBOT_POINT = 256

SKIP_INF = False
class Plotter(object):
    def __init__(self):
        pass

    def plot_heatmap(self, intensity_map):
        plt.imshow(intensity_map, cmap='plasma')
        plt.colorbar()
        plt.show()


class RobotController(object):
    def __init__(self):
        self._occupancy_map = OccupancyMap(skip_inf=SKIP_INF)
        self._inflated_occupancy_map = OccupancyMap(skip_inf=SKIP_INF)
        # Subscribe topics and bind with callback functions
        self._scan_sub = rospy.Subscriber(_SCAN_TOPIC, LaserScan, self.__callback_scan)
        self._pose_sub = rospy.Subscriber(_POSE_TOPIC, Odometry, self._callback_odom)
        self._update_map_timer = rospy.Timer(rospy.Duration(0.1), self._update_map_callback)
        self._update_path_timer = rospy.Timer(rospy.Duration(0.1), self._plan_path_callback)
        self._velocities_timer = rospy.Timer(rospy.Duration(0.1), self._publish_velocities)

        self._cmd_vel_pub = rospy.Publisher(_CMD_VEL_TOPIC, Twist)

        self.scan_msg = None
        self.pose2D = None
        self.first_pose2D = None
        self.path = []

        self._map_lock = Lock()

    def __callback_scan(self, msg):
        self.scan_msg = msg

    def _callback_odom(self, msg: Odometry):
        with self._map_lock:
            self.pose2D = self._occupancy_map.make_pose_2D(msg)
            if self.first_pose2D is None:
                self.first_pose2D = self.pose2D


    def _update_map_callback(self, event):
        if self.scan_msg is None or self.pose2D is None:
            return

        with self._map_lock:
            self._occupancy_map.update_map(self.scan_msg, self.pose2D)

    def _plan_path_callback(self, event):
        with self._map_lock:
            self._inflated_occupancy_map.occupancy_map = deepcopy(self._occupancy_map.occupancy_map)
        self._inflated_occupancy_map.inflate_obstacles()

        if self.path == [] or path_planning.is_obstacle_on_path(self._inflated_occupancy_map.occupancy_map, self.path):
            self.path = []
            self.path, _ = self._inflated_occupancy_map.find_path(start_pose=self.pose2D, end_pose=Pose2D(2.0, 1.0, 0.0))
        self._inflated_occupancy_map.publish_map()
        print(self.path)

    
    def _publish_velocities(self, event):
        if self.path == []:
            self.set_vel(0,0)
            return
        
        if self.node_reached(self.path, self.pose2D):
            self.path.pop(0)
        
        destination = bresenham.map_room_to_position(self.path[0])
        # adjusted_start_pose = adjust_position(self.pose2D)
        v, w = calculate_velocity(self.pose2D, destination)
        self.set_vel(v, w)

    def set_vel(self, v, w):
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self._cmd_vel_pub.publish(twist)

    def node_reached(self, path, pose):
        room_pose = bresenham.map_Pose2Do_room(pose)
        if room_pose.grid_y == path[0].grid_y and room_pose.grid_x == path[0].grid_x:
            return True
        return False


if __name__ == '__main__':
    rospy.init_node('lab_7_main')
    plotter = Plotter()
    robot_controller = RobotController()
    # path, map = robot_controller._occupancy_map.find_path()
    # print(path)
    # plotter.plot_heatmap(robot_controller._occupancy_map.occupancy_map)
    # plotter.plot_heatmap(map)
    rospy.spin()
