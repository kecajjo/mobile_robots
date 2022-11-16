#!/usr/bin/env python
import rospy
from laserSub import LaserScanListener
from sonarSub import SonarScanListener
import pickle
from plot import plot
from time import sleep

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    laser_listener = LaserScanListener()
    sonar_listener = SonarScanListener()

    while True:
        plot(laser_listener.get_point_list(), sonar_listener.get_point_list())
        sleep(1)

