#!/usr/bin/env python

import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from robotParams import *

_POSE_TOPIC = "/PIONIER"+ROBOT_NUMBER+"/RosAria/pose"


class PoseListener(object):
    def __init__(self):
        self._sub = rospy.Subscriber(_POSE_TOPIC, Odometry, self._callback)
        self._first_pose = None
        self._logs = []
        
    def rotate_point(self, x, y, theta):
        x_rotated = x * np.cos(theta) - y * np.sin(theta)
        y_rotated = x * np.sin(theta) + y * np.cos(theta)
        return x_rotated, y_rotated


    def _callback(self, msg: Odometry):
        if self._first_pose is None:
            self._first_pose = msg
            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            _,_,self.angle = euler_from_quaternion(orientation_list)
            
        first_position = self._first_pose.pose.pose.position
        position = msg.pose.pose.position
        
        first_x, first_y = self.rotate_point(first_position.x, first_position.y, -self.angle)
        x, y = self.rotate_point(position.x, position.y, -self.angle)
        
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _,_,angle = euler_from_quaternion(orientation_list)
        
        self._logs.append("{} {} {} {}".format("pose", round(x - first_x, 8), round(y - first_y, 8), angle - self.angle))
        