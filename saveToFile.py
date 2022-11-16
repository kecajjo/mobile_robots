#!/usr/bin/env python
import rospy
from laserSub import LaserScanListener
from sonarSub import SonarScanListener
import pickle
from plot import plot
from time import sleep

class _pc_saver(object):
    def __init__(self, laser_scan_listener, sonar_scan_listener):
        self.laser_scan_listener = laser_scan_listener
        self.sonar_scan_listener = sonar_scan_listener

    def __call__(self, event):
        self.save_files(self.laser_scan_listener, self.sonar_scan_listener)

    def save_files(self, laser_scan_listener, sonar_scan_listener):
        with open('laser_scans.txt', 'w+') as file:
            lst = laser_scan_listener.get_point_list()
            lst_to_save = []
            for point in lst:
                lst_to_save.append([point.x, point.y])
            pickle.dump(lst_to_save, file)
        with open('sonar_scans.txt', 'w+') as file:
            lst = sonar_scan_listener.get_point_list()
            lst_to_save = []
            for point in lst:
                lst_to_save.append([point.x, point.y])
            pickle.dump(lst_to_save, file)
        rospy.signal_shutdown("Finishing")

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    laser_listener = LaserScanListener()
    sonar_listener = SonarScanListener()

    save_files = _pc_saver(laser_listener, sonar_listener)

    while True:
        plot(laser_listener.get_point_list(), sonar_listener.get_point_list())
        sleep(1)

