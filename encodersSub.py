#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import JointState
from robotParams import *

_ENCODER_TOPIC = "/PIONIER"+ROBOT_NUMBER+"/RosAria/wheels"

_LEFT = 0
_RIGHT = 1
_MAX_INT_32 = 2**15


class EncodersListener(object):
    def __init__(self):
        self._sub = rospy.Subscriber(_ENCODER_TOPIC, JointState, self._callback_wheel_state)
        self._encoder_msg = JointState()
        self._previous_encoder_msg = None
        self._previous_timestamp = None
        self._logs = []
        
        self._pose_from_pos = [0,0,0]
        self._pose_from_vel = [0,0,0]
        
    def _callback_wheel_state(self, msg: JointState):
        if self._previous_timestamp is None:
            self._previous_timestamp = self._get_timestamp(msg)
            self._previous_encoder_msg = msg
            return
        
        # pose from position
        pos = self._get_position(msg.position, msg.position)
        previous_pos = self._get_position(msg.position, self._previous_encoder_msg.position)
        delta_pos = []
        for a,b in zip(pos, previous_pos):
            delta_pos.append(a - b)
        self._pose_from_pos = self._wheel_pos_to_robot_pos(self._pose_from_pos, delta_pos)
        
        # pose from velocity
        time_diff = self._get_timestamp(msg) - self._previous_timestamp
        delta_pose_vel = self._delta_pos_from_vel(time_diff.to_sec(), msg.velocity)
        self._pose_from_vel = self._wheel_pos_to_robot_pos(self._pose_from_vel, delta_pose_vel)
        
        self._previous_timestamp = rospy.Duration(msg.header.stamp.secs, msg.header.stamp.nsecs)
        self._previous_encoder_msg = msg
        self._logs.append("{} {} {} {}".format("pos", self._pose_from_pos[0]/1000, self._pose_from_pos[1]/1000, self._pose_from_pos[2]))
        self._logs.append("{} {} {} {}".format("vel", self._pose_from_vel[0]/1000, self._pose_from_vel[1]/1000, self._pose_from_vel[2]))

    def _get_timestamp(self, msg):
        return rospy.Duration(msg.header.stamp.secs, msg.header.stamp.nsecs) 
    
    def _get_position(self, current_pos, prev_pos):
        true_pos = []
        for a, b in zip(current_pos, prev_pos):
            if abs(a - b) > _MAX_INT_32:
                distance = b + a
            else:
                distance = b - a

            true_pos.append(distance / ENCODER_TICKS_TICKS_MM)
        return true_pos

    def _delta_pos_from_vel(self, delta_time, velocity):
        return [vel * delta_time for vel in velocity[:]]

    def _wheel_pos_to_robot_pos(self, start_pos, delta_wheel_pos):
        linear = (delta_wheel_pos[_LEFT] + delta_wheel_pos[_RIGHT]) / 2
        angular = (delta_wheel_pos[_RIGHT] - delta_wheel_pos[_LEFT]) / WHEEL_SEPARATION

        theta = start_pos[2] + angular
        x = math.cos(theta - angular/2) * linear + start_pos[0]
        y = math.sin(theta - angular/2) * linear + start_pos[1]

        return [x, y, theta]
