#!/usr/bin/env python
import rospy
from laserSub import LaserScanListener
from sonarSub import SonarScanListener

class _pc_saver(object):
    def __init__(self, laser_scan_listener, sonar_scan_listener):
        self.laser_scan_listener = laser_scan_listener
        self.sonar_scan_listener = sonar_scan_listener

    def __call__(self, event):
        self.save_files(self.laser_scan_listener, self.sonar_scan_listener)

    def save_files(self, laser_scan_listener, sonar_scan_listener):
        with open('laser_scans.txt', 'w+') as file:
            lst = laser_scan_listener.get_point_list()
            for point in lst:
                file.write("{} {}\n".format(point.x, point.y))    
        with open('sonar_scans.txt', 'w+') as file:
            lst = sonar_scan_listener.get_point_list()
            for point in lst:
                file.write("{} {}\n".format(point.x, point.y)) 
        rospy.signal_shutdown("Finishing")

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    laser_listener = LaserScanListener()
    sonar_listener = SonarScanListener()

    save_files = _pc_saver(laser_listener, sonar_listener)

    shutdown_timer = rospy.Timer(rospy.Duration(1), save_files)

    rospy.spin()
