#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

class Agent(object):
    def __init__(self):
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)

    def scan_callback(self, scan_msg):
        # print('got scan, now plan')
        drive = AckermannDriveStamped()
        drive.drive.speed = 0.0
        self.drive_pub.publish(drive)

if __name__ == '__main__':
    rospy.init_node('dummy_agent')
    dummy_agent = Agent()
    rospy.spin()