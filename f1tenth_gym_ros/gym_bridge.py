# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped

import gym

class GymBridge(Node):
    def __init__(self):
        super().__init__('gym_bridge')

        # env backend
        self.env = gym.make('f110_gym:f110_v0',
            map=self.get_parameter('map_path').get_parameter_value(),
            map_ext=self.get_parameter('map_img_ext').get_parameter_value(),
            num_agents=self.get_parameter('num_agent').get_parameter_value())

        # params
        ego_scan_topic = self.get_parameter('ego_scan_topic').get_parameter_value().string_value
        ego_odom_topic = self.get_parameter('ego_odom_topic').get_parameter_value().string_value
        ego_drive_topic = self.get_parameter('ego_drive_topic').get_parameter_value().string_value
        opp_scan_topic = self.get_parameter('opp_scan_topic').get_parameter_value().string_value
        opp_odom_topic = self.get_parameter('opp_odom_topic').get_parameter_value().string_value
        opp_drive_topic = self.get_parameter('opp_drive_topic').get_parameter_value().string_value

        # sim physical step timer
        self.timer = self.create_timer(0.01, self.timer_callback)

        # publishers
        self.ego_scan_pub = self.create_publisher(LaserScan, ego_scan_topic, 10)
        self.ego_odom_pub = self.create_publisher(Odometry, ego_odom_topic, 10)

        # subscribers
        self.ego_drive_sub = self.create_subscription(
            AckermannDriveStamped,
            ego_drive_topic,
            self.drive_callback,
            10)

    def drive_callback(self, drive_msg):
        pass

    def pose_callback(self, pose_msg):
        pass

    def timer_callback(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    rclpy.spin(gym_bridge)

if __name__ == '__main__':
    main()
