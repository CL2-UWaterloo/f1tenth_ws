#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

class Agent(object):
    def __init__(self):
        self.scan_sub = rospy.Subscriber('')