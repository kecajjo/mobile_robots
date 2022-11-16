#!/usr/bin/env python3

# run: python subscriber_scan.py 5  (where 5 is robot number)
import sys
import rospy
import math
import pickle
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import laser_geometry
import laser_geometry.laser_geometry
import sensor_msgs.point_cloud2 as pc2


class laser_scan(object):
    def __init__(self, robot_nr):
        self.laser_projection = laser_geometry.LaserProjection()
        self.laser_scan_sub = rospy.Subscriber("/PIONIER"+robot_nr+"/scan", LaserScan, self.callback_scan)
        self.sonar_sub = rospy.Subscriber("/PIONIER"+robot_nr+"/RosAria/sonar_pointcloud2", PointCloud2, self.callback_sonar)
        
        self.shutdown_timer = rospy.Timer(rospy.Duration(5), self.timer_callback)
        
        self.laser_scans = []
        self.sonar_scans = []
        
    
    def callback_scan(self, msg: LaserScan):
        ## extract ranges from message
        pc2_msg = self.laser_projection.projectLaser(msg)
        point_generator = pc2.read_points_list(pc2_msg)
        current_scan = []
        for point in point_generator:
            current_scan.append([point.x, point.y])
        
        self.laser_scans.append(current_scan)
        
    def callback_sonar(self, msg: PointCloud2):
        ## extract ranges from message
        # print(msg)
        point_generator = pc2.read_points_list(msg)
        current_scan = []
        for point in point_generator:
            current_scan.append([point.x, point.y])
        
        self.sonar_scans.append(current_scan)
        # scan=list(msg.ranges)
        # print("  Scan min: {}  front: {}".format(min(scan),scan[256]))
        
    def timer_callback(self, event):
        with open('laser_scans', 'wb') as file:
            pickle.dump(self.laser_scans, file)
            # file.write(str(self.laser_scans))
            
        with open('sonar_scans', 'wb') as file:
            pickle.dump(self.sonar_scans, file)
            # file.write(str(self.laser_scans))

        rospy.signal_shutdown("Finishing")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.stderr.write('Usage: sys.argv[0] \n')
        sys.exit(1)

    nr=sys.argv[1]
    
    rospy.init_node('listener', anonymous=True)
    handler = laser_scan(nr)

    rospy.spin()
    