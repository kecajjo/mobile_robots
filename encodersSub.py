#!/usr/bin/env python

# run: python subscriber_scan.py 5  (where 5 is robot number)
import sys
import rospy
import math
from sensor_msgs.msg import JointState
from robotParams import *

_ENCODER_TOPIC = "/PIONIER"+ROBOT_NUMBER+"/RosAria/wheels"

_LEFT = 0
_RIGHT = 1
_MAX_TICK = 2^16/4
_MAX_INT_32 = 2^15
_MIN_INT_32 = -2^15

class EncodersListener(object):
    def __init__(self):
        # Subscribe topics and bind with callback functions
        self._sub = rospy.Subscriber(_ENCODER_TOPIC, JointState, self._callback_wheel_state)
        self.encoder_msg = JointState()

    def _callback_wheel_state(self, msg):
        ## extract ranges from message
        self.encoder_msg = msg
        
    def get_velocity(self):
        return self.encoder_msg.velocity

    def _check_overflow(self, end_val, prev_val):
        if abs(end_val - prev_val) > _MAX_TICK:
                distance = _MAX_INT_32 - max(end_val,prev_val) + (min(end_val,prev_val)-_MIN_INT_32)
                if end_val > prev_val:
                    distance = -distance
        return distance


    def get_position(self, prev_pos):
        # print([position / ENCODER_TICKS_TICKS_MM for position in self.encoder_msg.position[:]])
        ret_val =  [position / ENCODER_TICKS_TICKS_MM for position in self.encoder_msg.position[:]]
        distance = 0
        for a, b in zip(ret_val, prev_pos):
            if abs(a - b) > _MAX_TICK:
                distance = _MAX_INT_32 - max(a,b) + (min(a,b)-_MIN_INT_32)
                if a > b:
                    distance = -distance


    def get_timestamp(self):
        return rospy.Duration(self.encoder_msg.header.stamp.secs, self.encoder_msg.header.stamp.nsecs)

    def delta_pos_from_vel(self, delta_time):
        return [velocity * delta_time for velocity in self.encoder_msg.velocity[:]]

    def wheel_pos_to_robot_pos(self, start_pos, delta_wheel_pos):
        linear = (delta_wheel_pos[_LEFT] + delta_wheel_pos[_RIGHT]) / 2
        angular = (delta_wheel_pos[_RIGHT] - delta_wheel_pos[_LEFT]) / WHEEL_SEPARATION

        theta = start_pos[2] + angular
        x = math.cos(theta - angular/2) * linear + start_pos[0]
        y = math.sin(theta - angular/2) * linear + start_pos[1]

        return [x, y, theta]

if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.stderr.write('Usage: sys.argv[0] \n')
        sys.exit(1)

    nr=sys.argv[1]
    listener()  