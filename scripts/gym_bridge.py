#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import *

import numpy as np
import msgpack
import zmq

import gym
class GymBridge(object):
    def __init__(self):
        # get params
        self.ego_scan_topic = rospy.get_param('ego_scan_topic')
        self.opp_scan_topic = rospy.get_param('opp_scan_topic')
        self.ego_odom_topic = rospy.get_param('ego_odom_topic')
        self.opp_odom_topic = rospy.get_param('opp_odom_topic')

        self.ego_drive_topic = rospy.get_param('ego_drive_topic')

        # init gym backend
        self.racecar_env = gym.make('f110_gym:f110-v0')

        # pubs
        self.ego_scan_pub = rospy.Publisher(self.ego_scan_topic, LaserScan, queue_size=1)
        self.opp_scan_pub = rospy.Publisher(self.opp_scan_topic, LaserScan, queue_size=1)
        self.ego_odom_pub = rospy.Publisher(self.ego_odom_topic, Odometry, queue_size=1)
        self.opp_odom_pub = rospy.Publisher(self.opp_odom_topic, Odometry, queue_size=1)

        # subs
        self.drive_sub = rospy.Subscriber(self.ego_drive_topic, AckermannDriveStamped, self.drive_callback, queue_size=1)


    def drive_callback(self, drive_msg):
        ego_speed = drive_msg.drive.speed
        ego_steer = drive_msg.drive.steering_angle