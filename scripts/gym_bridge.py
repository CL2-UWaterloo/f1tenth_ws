#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import transform_broadcaster
from tf.transformations import quaternion_from_euler

import numpy as np
import zmq

import gym
class GymBridge(object):
    def __init__(self):
        # get params
        self.ego_scan_topic = rospy.get_param('ego_scan_topic')
        self.ego_odom_topic = rospy.get_param('ego_odom_topic')
        self.opp_odom_topic = rospy.get_param('opp_odom_topic')

        self.ego_drive_topic = rospy.get_param('ego_drive_topic')

        # init gym backend
        self.racecar_env = gym.make('f110_gym:f110-v0')

        # init opponent agent
        # TODO: init by params.yaml
        self.opp_agent = blah
        initial_state = {'x':[0.0, 2.0], 'y': [0.0, 0.0], 'theta': [0.0, 0.0]}
        self.obs, _, self.done, _ = self.racecar_env.reset(initial_state)
        self.ego_pose = [0., 0., 0.]
        self.ego_steer = 0.0
        self.opp_pose = [2., 0., 0.]
        self.opp_steer = 0.0

        # transform broadcaster
        self.br = transform_broadcaster.TransformBroadcaster()

        # pubs
        self.ego_scan_pub = rospy.Publisher(self.ego_scan_topic, LaserScan, queue_size=1)
        self.opp_scan_pub = rospy.Publisher(self.opp_scan_topic, LaserScan, queue_size=1)
        self.ego_odom_pub = rospy.Publisher(self.ego_odom_topic, Odometry, queue_size=1)
        self.opp_odom_pub = rospy.Publisher(self.opp_odom_topic, Odometry, queue_size=1)

        # subs
        self.drive_sub = rospy.Subscriber(self.ego_drive_topic, AckermannDriveStamped, self.drive_callback, queue_size=1)


    def drive_callback(self, drive_msg):
        # TODO: trigger opp agent plan, step env, update pose and steer
        ego_speed = drive_msg.drive.speed
        self.ego_steer = drive_msg.drive.steering_angle
        opp_speed, self.opp_steer = self.opp_agent.plan(self.obs)

        action = {}
        obs, step_reward, done, info = self.racecar_env.step(action)


    def publish_transforms(self, ts):
        ego_t = Transform()
        ego_t.translation.x = self.ego_pose[0]
        ego_t.translation.y = self.ego_pose[1]
        ego_t.translation.z = 0.0
        ego_quat = quaternion_from_euler(0.0, 0.0, self.ego_pose[2])
        ego_t.rotation.x = ego_quat[0]
        ego_t.rotation.y = ego_quat[1]
        ego_t.rotation.z = ego_quat[2]
        ego_t.rotation.w = ego_quat[3]

        ego_ts = TransformStamped()
        ego_ts.transform = ego_t
        ego_ts.header.stamp = ts
        ego_ts.header.frame_id = '/map'
        # TODO: check frame names
        ego_ts.child_frame_id = 'ego_racecar/base_link'

        opp_t = Transform()
        opp_t.translation.x = self.opp_pose[0]
        opp_t.translation.y = self.opp_pose[1]
        opp_t.translation.z = 0.0
        opp_quat = quaternion_from_euler(0.0, 0.0, self.opp_pose[2])
        opp_t.rotation.x = opp_quat[0]
        opp_t.rotation.y = opp_quat[1]
        opp_t.rotation.z = opp_quat[2]
        opp_t.rotation.w = opp_quat[3]

        opp_ts = TransformStamped()
        opp_ts.transform = opp_t
        opp_ts.header.stamp = ts
        opp_ts.header.frame_id = '/map'
        # TODO: check frame names
        opp_ts.child_frame_id = 'opp_racecar/base_link'

        self.br.sendTransform(ego_ts)
        self.br.sendTransform(opp_ts)

    def publish_wheel_transforms(self, ts):
        ego_wheel_ts = TransformStamped()
        ego_wheel_quat = quaternion_from_euler(0., 0., self.ego_steer)
        ego_wheel_ts.transform.rotation.x = ego_wheel_quat[0]
        ego_wheel_ts.transform.rotation.y = ego_wheel_quat[1]
        ego_wheel_ts.transform.rotation.z = ego_wheel_quat[2]
        ego_wheel_ts.transform.rotation.w = ego_wheel_quat[3]
        ego_wheel_ts.header.stamp = ts
        ego_wheel_ts.header.frame_id = 'ego_racecar/front_left_hinge'
        ego_wheel_ts.child_frame_id = 'ego_racecar/front_left_wheel'
        self.br.sendTransform(ego_wheel_ts)
        ego_wheel_ts.header.frame_id = 'ego_racecar/front_right_hinge'
        ego_wheel_ts.child_frame_id = 'ego_racecar/front_right_wheel'
        self.br.sendTransform(ego_wheel_ts)


        opp_wheel_ts = TransformStamped()
        opp_wheel_quat = quaternion_from_euler(0., 0., self.opp_steer)
        opp_wheel_ts.transform.rotation.x = opp_wheel_quat[0]
        opp_wheel_ts.transform.rotation.y = opp_wheel_quat[1]
        opp_wheel_ts.transform.rotation.z = opp_wheel_quat[2]
        opp_wheel_ts.transform.rotation.w = opp_wheel_quat[3]
        opp_wheel_ts.header.stamp = ts
        opp_wheel_ts.header.frame_id = 'opp_racecar/front_left_hinge'
        opp_wheel_ts.child_frame_id = 'opp_racecar/front_left_wheel'
        self.br.sendTransform(opp_wheel_ts)
        opp_wheel_ts.header.frame_id = 'opp_racecar/front_right_hinge'
        opp_wheel_ts.child_frame_id = 'opp_racecar/front_right_wheel'
        self.br.sendTransform(opp_wheel_ts)

        