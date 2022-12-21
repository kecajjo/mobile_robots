#!/usr/bin/env python
import rospy
from encodersSub import EncodersListener
from poseSub import PoseListener


class odometry(object):
    def __init__(self, encoder_listener, pose_listener):
        self.encoder_listener = encoder_listener
        self.pose_listener = pose_listener
        
        rospy.on_shutdown(self.dump_logs)

    def dump_logs(self):
        with open('data.txt', 'w') as f:
            for log in self.encoder_listener._logs:
                f.write(log+"\n")
            for log in self.pose_listener._logs:
                f.write(log+"\n")


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    encoder_listener = EncodersListener()
    pose_listener = PoseListener()
    odom = odometry(encoder_listener, pose_listener)
    
    rospy.spin()


