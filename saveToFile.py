#!/usr/bin/env python
import rospy
from laserSub import LaserScanListener
from sonarSub import SonarScanListener
from encodersSub import EncodersListener
from plot import plot
from time import sleep

class odometry(object):
    def __init__(self, listener):
        self.listener = listener

        self.first_timestamp = rospy.Duration(0)
        self.pose_stamp_pos = [0,0,0]
        self.pose_stamp_vel = [0,0,0]
        self.init_pos = 0
        self.isFirst = True

        self.print_timer = rospy.Timer(rospy.Duration(0, 10000), self.print_callback)

    def print_callback(self, event):
        msg_timestamp = encoder_listener.get_timestamp()
        pos = encoder_listener.get_position()
        if msg_timestamp == rospy.Duration(0):
            self.first_timestamp = msg_timestamp
            return
        if self.isFirst:
            self.init_pos = pos
            self.isFirst = False
        time_diff = msg_timestamp - self.first_timestamp
        self.first_timestamp = msg_timestamp
        delta_pose_vel = encoder_listener.delta_pos_from_vel(time_diff.to_sec())
        self.pose_stamp_vel = encoder_listener.wheel_pos_to_robot_pos(self.pose_stamp_vel, delta_pose_vel)

        delta_pos = []
        for a,b in zip(pos, self.init_pos):
            delta_pos.append(a - b)
        self.init_pos = pos
        self.pose_stamp_pos = encoder_listener.wheel_pos_to_robot_pos(self.pose_stamp_pos, delta_pos)

        # print("Pose stamp vel: {} \nPose stamp pos: {}".format(self.pose_stamp_vel, self.pose_stamp_pos))




if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    # laser_listener = LaserScanListener()
    # sonar_listener = SonarScanListener()
    encoder_listener = EncodersListener()
    odom = odometry(encoder_listener)

    rospy.spin()


