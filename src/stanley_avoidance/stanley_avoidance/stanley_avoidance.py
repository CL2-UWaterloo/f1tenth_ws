#!/usr/bin/env python3
import math
import copy
import numpy as np
from scipy import signal
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
from collections import deque

import rclpy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Pose
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped


class Vertex(object):
    def __init__(self, pos=None, parent=None):
        self.pos = pos
        self.parent = parent


class StanleyAvoidance(Node):
    def __init__(self):
        super().__init__("stanley_avoidance_node")

        self.declare_parameter("waypoints_path", "/sim_ws/racelines/e7_floor5.csv")
        self.declare_parameter("waypoints_path_2nd", "/sim_ws/racelines/e7_floor5.csv")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/ego_racecar/odom")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("rviz_current_waypoint_topic", "/current_waypoint")
        self.declare_parameter("rviz_lookahead_waypoint_topic", "/lookahead_waypoint")
        self.declare_parameter("stanley_avoidance_path_topic", "/stanley_avoidance_path")
        self.declare_parameter("stanley_avoidance_path_array_topic", "/stanley_avoidance_path_array")
        self.declare_parameter("occupancy_grid_topic", "/occupancy_grid")

        self.declare_parameter("grid_width_meters", 6.0)
        self.declare_parameter("K_p", 0.5)
        self.declare_parameter("K_p_obstacle", 0.8)
        self.declare_parameter("K_E", 2.0)
        self.declare_parameter("K_H", 1.5)
        self.declare_parameter("min_lookahead", 1.0)
        self.declare_parameter("max_lookahead", 3.0)
        self.declare_parameter("min_lookahead_speed", 3.0)
        self.declare_parameter("max_lookahead_speed", 6.0)
        self.declare_parameter("interpolation_distance", 0.05)
        self.declare_parameter("velocity_min", 0.5)
        self.declare_parameter("velocity_max", 2.0)
        self.declare_parameter("velocity_percentage", 1.0)
        self.declare_parameter("steering_limit", 25.0)
        self.declare_parameter("cells_per_meter", 10)

        self.declare_parameter("lane_number", 0)

        self.waypoints_world_path = str(self.get_parameter("waypoints_path").value)
        self.waypoints_world_path_2nd = str(self.get_parameter("waypoints_path_2nd").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.drive_topic = str(self.get_parameter("drive_topic").value)
        self.rviz_current_waypoint_topic = str(self.get_parameter("rviz_current_waypoint_topic").value)
        self.rviz_lookahead_waypoint_topic = str(self.get_parameter("rviz_lookahead_waypoint_topic").value)
        self.stanley_avoidance_path_topic = str(self.get_parameter("stanley_avoidance_path_topic").value)
        self.stanley_avoidance_path_array_topic = str(self.get_parameter("stanley_avoidance_path_array_topic").value)
        self.occupancy_grid_topic = str(self.get_parameter("occupancy_grid_topic").value)
        self.L = float(self.get_parameter("max_lookahead").value)
        self.grid_width_meters = float(self.get_parameter("grid_width_meters").value)
        self.K_E = float(self.get_parameter("K_E").value)
        self.K_H = float(self.get_parameter("K_H").value)
        self.K_p = float(self.get_parameter("K_p").value)
        self.K_p_obstacle = float(self.get_parameter("K_p_obstacle").value)
        self.interpolation_distance = float(self.get_parameter("interpolation_distance").value)
        self.velocity_min = float(self.get_parameter("velocity_min").value)
        self.velocity_max = float(self.get_parameter("velocity_max").value)
        self.velocity_percentage = float(self.get_parameter("velocity_percentage").value)
        self.steering_limit = float(self.get_parameter("steering_limit").value)
        self.CELLS_PER_METER = int(self.get_parameter("cells_per_meter").value)
        self.lane_number = int(self.get_parameter("lane_number").value)  # Dynamically change lanes

        min_lookahead = float(self.get_parameter("min_lookahead").value)
        max_lookahead = float(self.get_parameter("max_lookahead").value)
        min_lookahead_speed = float(self.get_parameter("min_lookahead_speed").value)
        max_lookahead_speed = float(self.get_parameter("max_lookahead_speed").value)

        # hyper-parameters
        self.waypoint_utils = WaypointUtils(
            node=self,
            L=self.L,
            interpolation_distance=self.interpolation_distance,
            filepath=self.waypoints_world_path,
            min_lookahead=min_lookahead,
            max_lookahead=max_lookahead,
            min_lookahead_speed=min_lookahead_speed,
            max_lookahead_speed=max_lookahead_speed,
            filepath_2nd=self.waypoints_world_path_2nd,
        )

        self.get_logger().info(f"Loaded {len(self.waypoint_utils.waypoints_world)} waypoints")
        self.utils = Utils()

        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 1)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 1)

        # publishers
        self.create_timer(1.0, self.timer_callback)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.current_waypoint_pub = self.create_publisher(Marker, self.rviz_current_waypoint_topic, 10)
        self.waypoint_pub = self.create_publisher(Marker, self.rviz_lookahead_waypoint_topic, 10)
        self.stanley_avoidance_path_pub = self.create_publisher(Marker, self.stanley_avoidance_path_topic, 10)
        self.stanley_avoidance_path_array_pub = self.create_publisher(MarkerArray, self.stanley_avoidance_path_array_topic, 10)
        self.occupancy_grid_pub = self.create_publisher(OccupancyGrid, self.occupancy_grid_topic, 10)

        # constants
        self.MAX_RANGE = self.L - 0.1
        self.MIN_ANGLE = np.radians(0)
        self.MAX_ANGLE = np.radians(180)
        # fov = 270. One side = fov/2 = 135. 135-45 = 90 (for occupancy grid)
        self.ANGLE_OFFSET = np.radians(45)
        self.IS_OCCUPIED = 100
        self.IS_FREE = 0

        # class variables
        self.grid_height = int(self.L * self.CELLS_PER_METER)
        self.grid_width = int(self.grid_width_meters * self.CELLS_PER_METER)
        self.CELL_Y_OFFSET = (self.grid_width // 2) - 1
        self.occupancy_grid = np.full(shape=(self.grid_height, self.grid_width), fill_value=-1, dtype=int)
        self.current_pose = None
        # from the laser frame, approx front wheelbase. Needed for stanley controller
        self.current_pose_wheelbase_front = None
        self.goal_pos = None
        self.closest_wheelbase_rear_point = None
        self.obstacle_detected = False
        self.target_velocity = 0.0

    def timer_callback(self):
        self.waypoints_world_path = str(self.get_parameter("waypoints_path").value)
        self.K_E = float(self.get_parameter("K_E").value)
        self.K_H = float(self.get_parameter("K_H").value)
        self.K_p = float(self.get_parameter("K_p").value)
        self.K_p_obstacle = float(self.get_parameter("K_p_obstacle").value)
        self.interpolation_distance = int(self.get_parameter("interpolation_distance").value)
        self.velocity_min = float(self.get_parameter("velocity_min").value)
        self.velocity_max = float(self.get_parameter("velocity_max").value)
        self.velocity_percentage = float(self.get_parameter("velocity_percentage").value)
        self.steering_limit = float(self.get_parameter("steering_limit").value)
        self.CELLS_PER_METER = int(self.get_parameter("cells_per_meter").value)

        self.waypoint_utils.min_lookahead = float(self.get_parameter("min_lookahead").value)
        self.waypoint_utils.max_lookahead = float(self.get_parameter("max_lookahead").value)
        self.waypoint_utils.min_lookahead_speed = float(self.get_parameter("min_lookahead_speed").value)
        self.waypoint_utils.max_lookahead_speed = float(self.get_parameter("max_lookahead_speed").value)

        self.waypoint_utils.lane_number = int(self.get_parameter("lane_number").value)  # Dynamically change lanes

    def local_to_grid(self, x, y):
        i = int(x * -self.CELLS_PER_METER + (self.grid_height - 1))
        j = int(y * -self.CELLS_PER_METER + self.CELL_Y_OFFSET)
        return (i, j)

    def local_to_grid_parallel(self, x, y):
        i = np.round(x * -self.CELLS_PER_METER + (self.grid_height - 1)).astype(int)
        j = np.round(y * -self.CELLS_PER_METER + self.CELL_Y_OFFSET).astype(int)
        return i, j

    def grid_to_local(self, point):
        i, j = point[0], point[1]
        x = (i - (self.grid_height - 1)) / -self.CELLS_PER_METER
        y = (j - self.CELL_Y_OFFSET) / -self.CELLS_PER_METER
        return (x, y)

    def odom_callback(self, pose_msg: Odometry):
        """
        The pose callback when subscribed to particle filter's inferred pose
        """
        # determine pose data type (sim vs. car)
        self.current_pose = pose_msg.pose.pose

        # TOO SLOW
        # to_frame_rel = "ego_racecar/base_link"
        # from_frame_rel = "ego_racecar/laser_model"

        # if self.base_link_to_laser_tf is None: # static tf, so only need to lookup once
        #     try:
        #         self.base_link_to_laser_tf = self.tf_buffer.lookup_transform(
        #                 to_frame_rel,
        #                 from_frame_rel,
        #                 rclpy.time.Time())
        #     except TransformException as ex:
        #         self.get_logger().info(
        #             f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        #         return

        # self.get_logger().info(str(self.base_link_to_laser_tf))

        current_pose_quaternion = np.array(
            [
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w,
            ]
        )

        # 33cm in front of base_link
        self.current_pose_wheelbase_front = Pose()
        current_pose_xyz = R.from_quat(current_pose_quaternion).apply((0.33, 0, 0)) + (
            self.current_pose.position.x,
            self.current_pose.position.y,
            0,
        )
        self.current_pose_wheelbase_front.position.x = current_pose_xyz[0]
        self.current_pose_wheelbase_front.position.y = current_pose_xyz[1]
        self.current_pose_wheelbase_front.position.z = current_pose_xyz[2]
        self.current_pose_wheelbase_front.orientation = self.current_pose.orientation

        # obtain pure pursuit waypoint (base_link frame)
        self.closest_wheelbase_rear_point, self.target_velocity = self.waypoint_utils.get_closest_waypoint_with_velocity(
            self.current_pose
        )

        self.utils.draw_marker(
            pose_msg.header.frame_id,
            pose_msg.header.stamp,
            self.closest_wheelbase_rear_point,
            self.current_waypoint_pub,
            color="blue",
        )

        # self.current_velocity = np.sqrt(pose_msg.twist.twist.linear.x**2 +  pose_msg.twist.twist.linear.y**2)

        self.goal_pos, goal_pos_world = self.waypoint_utils.get_waypoint(self.current_pose, self.target_velocity)

        self.utils.draw_marker(pose_msg.header.frame_id, pose_msg.header.stamp, goal_pos_world, self.waypoint_pub, color="red")

    def populate_occupancy_grid(self, ranges, angle_increment):
        """
        Populate occupancy grid using lidar scans and save
        the data in class member variable self.occupancy_grid.

        Optimization performed to improve the speed at which we generate the occupancy grid.

        Args:
            scan_msg (LaserScan): message from lidar scan topic
        """
        # reset empty occupacny grid (-1 = unknown)

        self.occupancy_grid = np.full(shape=(self.grid_height, self.grid_width), fill_value=self.IS_FREE, dtype=int)

        ranges = np.array(ranges)
        indices = np.arange(len(ranges))
        thetas = (indices * angle_increment) - self.ANGLE_OFFSET
        xs = ranges * np.sin(thetas)
        ys = ranges * np.cos(thetas) * -1

        i, j = self.local_to_grid_parallel(xs, ys)

        occupied_indices = np.where((i >= 0) & (i < self.grid_height) & (j >= 0) & (j < self.grid_width))
        self.occupancy_grid[i[occupied_indices], j[occupied_indices]] = self.IS_OCCUPIED

    # NYI
    def publish_occupancy_grid(self, frame_id, stamp):
        """
        Publish populated occupancy grid to ros2 topic
        Args:
            scan_msg (LaserScan): message from lidar scan topic
        """
        oc = OccupancyGrid()
        oc.header.frame_id = frame_id
        oc.header.stamp = stamp
        oc.info.origin.position.y -= ((self.grid_width / 2) + 1) / self.CELLS_PER_METER
        oc.info.width = self.grid_height
        oc.info.height = self.grid_width
        oc.info.resolution = 1 / self.CELLS_PER_METER
        oc.data = np.fliplr(np.rot90(self.occupancy_grid, k=1)).flatten().tolist()
        self.occupancy_grid_pub.publish(oc)

    def convolve_occupancy_grid(self):
        kernel = np.ones(shape=[2, 2])
        self.occupancy_grid = signal.convolve2d(
            self.occupancy_grid.astype("int"), kernel.astype("int"), boundary="symm", mode="same"
        )
        self.occupancy_grid = np.clip(self.occupancy_grid, -1, 100)

    def drive_to_target(self, point, K_p):
        """
        Using the pure pursuit derivation

        Improvement is that we make the point closer when the car is going at higher speeds

        """
        # calculate curvature/steering angle
        L = np.linalg.norm(point)
        y = point[1]
        angle = K_p * (2 * y) / (L**2)
        angle = np.clip(angle, -np.radians(self.steering_limit), np.radians(self.steering_limit))

        # determine velocity
        if self.obstacle_detected and self.velocity_percentage > 0.0:
            if np.degrees(angle) < 10.0:
                velocity = self.velocity_max
            elif np.degrees(angle) < 20.0:
                velocity = (self.velocity_max + self.velocity_min) / 2
            else:
                velocity = self.velocity_min

        else:
            # Set velocity to velocity of racing line
            velocity = self.target_velocity * self.velocity_percentage

        # publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = angle
        self.get_logger().info(
            f"Obstacle: {self.obstacle_detected} ... lookahead: {self.waypoint_utils.L:.2f} ... index: {self.waypoint_utils.index} ... Speed: {velocity:.2f}m/s ... Steering Angle: {np.degrees(angle):.2f} ... K_p: {self.K_p} ... K_p_obstacle: {self.K_p_obstacle} ... velocity_percentage: {self.velocity_percentage:.2f}"
        )
        self.drive_pub.publish(drive_msg)

    def drive_to_target_stanley(self):
        """
        Using the stanley method derivation: https://stevengong.co/notes/Stanley-Method

        Might get the best out of both worlds for responsiveness, and less oscillations compared to pure pursuit.
        """
        K_V = 0
        # calculate curvature/steering angle
        closest_wheelbase_front_point_car, closest_wheelbase_front_point_world = self.waypoint_utils.get_waypoint_stanley(
            self.current_pose_wheelbase_front
        )

        path_heading = math.atan2(
            closest_wheelbase_front_point_world[1] - self.closest_wheelbase_rear_point[1],
            closest_wheelbase_front_point_world[0] - self.closest_wheelbase_rear_point[0],
        )
        current_heading = math.atan2(
            self.current_pose_wheelbase_front.position.y - self.current_pose.position.y,
            self.current_pose_wheelbase_front.position.x - self.current_pose.position.x,
        )

        if current_heading < 0:
            current_heading += 2 * math.pi
        if path_heading < 0:
            path_heading += 2 * math.pi

        # calculate the errors
        crosstrack_error = math.atan2(
            self.K_E * closest_wheelbase_front_point_car[1], K_V + self.target_velocity
        )  # y value in car frame
        heading_error = path_heading - current_heading
        if heading_error > math.pi:
            heading_error -= 2 * math.pi
        elif heading_error < -math.pi:
            heading_error += 2 * math.pi

        heading_error *= self.K_H

        # Calculate the steering angle using the Stanley controller formula
        angle = heading_error + crosstrack_error

        self.get_logger().info(
            f"heading_error: {heading_error:.2f} ... crosstrack_error: {crosstrack_error:.2f} angle: {np.degrees(angle):.2f}"
        )
        self.get_logger().info(f"current_heading: {current_heading:.2f} ... path_heading: {path_heading:.2f}")

        angle = np.clip(angle, -np.radians(self.steering_limit), np.radians(self.steering_limit))

        velocity = self.target_velocity * self.velocity_percentage

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = angle
        self.drive_pub.publish(drive_msg)

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, update occupancy grid and perform local planning

        Args:
            scan_msg (LaserScan): incoming message from subscribed topic
        """
        # make sure we obtain initial pose and goal point
        if (self.current_pose is None) or (self.goal_pos is None):
            return

        # populate occupancy grid
        self.populate_occupancy_grid(scan_msg.ranges, scan_msg.angle_increment)
        self.convolve_occupancy_grid()
        self.publish_occupancy_grid(scan_msg.header.frame_id, scan_msg.header.stamp)

        # get path planed in occupancy grid space
        path_local = []

        current_pos = np.array(self.local_to_grid(0, 0))
        goal_pos = np.array(self.local_to_grid(self.goal_pos[0], self.goal_pos[1]))
        target = None
        path_local = [self.grid_to_local(current_pos)]
        MARGIN = int(self.CELLS_PER_METER * 0.15) # 0.15m margin on each side, since the car is ~0.3m wide

        if self.check_collision(current_pos, goal_pos, margin=MARGIN):
            self.obstacle_detected = True

            shifts = [i * (-1 if i % 2 else 1) for i in range(1, 21)]

            found = False
            for shift in shifts:
                # We consider various points to the left and right of the goal position
                new_goal = goal_pos + np.array([0, shift])

                # If we are currently super close to the wall, this logic doesn't work
                if not self.check_collision(current_pos, new_goal, margin=int(1.5 * MARGIN)):
                    target = self.grid_to_local(new_goal)
                    found = True
                    path_local.append(target)
                    self.get_logger().info("Found condition 1")
                    break

            if not found:
                # This means that the obstacle is very close to us, we need even steeper turns
                middle_grid_point = np.array(current_pos + (goal_pos - current_pos) / 2).astype(int)

                for shift in shifts:
                    new_goal = middle_grid_point + np.array([0, shift])
                    if not self.check_collision(current_pos, new_goal, margin=int(1.5 * MARGIN)):
                        target = self.grid_to_local(new_goal)
                        found = True
                        path_local.append(target)
                        self.get_logger().info("Found condition 2")
                        break

            if not found:
                # Try again with a looser collision checker, we are probably very close to the obstacle, so check only collision free in the second half
                middle_grid_point = np.array(current_pos + (goal_pos - current_pos) / 2).astype(int)

                for shift in shifts:
                    new_goal = middle_grid_point + np.array([0, shift])
                    if not self.check_collision_loose(current_pos, new_goal, margin=MARGIN):
                        target = self.grid_to_local(new_goal)
                        found = True
                        path_local.append(target)
                        self.get_logger().info("Found condition 3")
                        break

        else:
            self.obstacle_detected = False
            target = self.grid_to_local(goal_pos)
            path_local.append(target)

        if target:
            if self.obstacle_detected:
                self.drive_to_target(target, self.K_p_obstacle)
            else:
                self.drive_to_target_stanley()
        else:
            self.get_logger().info("Could not find a target path, halting vehicle")
            # publish drive message
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)

        # Visualization
        self.utils.draw_marker_array(scan_msg.header.frame_id, scan_msg.header.stamp, path_local, self.stanley_avoidance_path_array_pub)
        self.utils.draw_lines(scan_msg.header.frame_id, scan_msg.header.stamp, path_local, self.stanley_avoidance_path_pub)


    def check_collision(self, cell_a, cell_b, margin=0):
        """
        Checks whether the path between two cells
        in the occupancy grid is collision free.

        The margin is done by checking if adjacent cells are also free.

        One of the issues is that if the starting cell is next to a wall, then it already considers there to be a collision.
        See check_collision_loose


        Args:
            cell_a (i, j): index of cell a in occupancy grid
            cell_b (i, j): index of cell b in occupancy grid
            margin (int): margin of safety around the path
        Returns:
            collision (bool): whether path between two cells would cause collision
        """
        for i in range(-margin, margin + 1):  # for the margin, check
            cell_a_margin = (cell_a[0], cell_a[1] + i)
            cell_b_margin = (cell_b[0], cell_b[1] + i)
            for cell in self.utils.traverse_grid(cell_a_margin, cell_b_margin):
                if (cell[0] * cell[1] < 0) or (cell[0] >= self.grid_height) or (cell[1] >= self.grid_width):
                    continue
                try:
                    if self.occupancy_grid[cell] == self.IS_OCCUPIED:
                        return True
                except:
                    self.get_logger().info(f"Sampled point is out of bounds: {cell}")
                    return True
        return False

    def check_collision_loose(self, cell_a, cell_b, margin=0):
        """
        Checks whether the path between two cells
        in the occupancy grid is collision free.

        The margin is done by checking if adjacent cells are also free.

        This looser implementation only checks half way for meeting the margin requirement.


        Args:
            cell_a (i, j): index of cell a in occupancy grid
            cell_b (i, j): index of cell b in occupancy grid
            margin (int): margin of safety around the path
        Returns:
            collision (bool): whether path between two cells would cause collision
        """
        for i in range(-margin, margin + 1):  # for the margin, check
            cell_a_margin = (int((cell_a[0] + cell_b[0]) / 2), int((cell_a[1] + cell_b[1]) / 2) + i)
            cell_b_margin = (cell_b[0], cell_b[1] + i)
            for cell in self.utils.traverse_grid(cell_a_margin, cell_b_margin):
                if (cell[0] * cell[1] < 0) or (cell[0] >= self.grid_height) or (cell[1] >= self.grid_width):
                    continue
                try:
                    if self.occupancy_grid[cell] == self.IS_OCCUPIED:
                        return True
                except:
                    self.get_logger().info(f"Sampled point is out of bounds: {cell}")
                    return True
        return False

    def find_path(self, T_start, T_goal, pruning=True):
        """
        Returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        # traverse up T_start to obtain path to sampled point
        node = T_start[-1]
        path_start = [node.pos]
        while node.parent is not None:
            node = T_start[node.parent]
            path_start.append(node.pos)

        # traverse up T_goal to obtain path to sampled point
        node = T_goal[-1]
        path_goal = [node.pos]
        while node.parent is not None:
            node = T_goal[node.parent]
            path_goal.append(node.pos)

        # return path
        path = np.array(path_start[::-1] + path_goal[1:])

        # pruning if enabled
        if pruning:
            sub_paths = []
            for i in range(len(path) - 2):
                sub_path = path
                for j in range(i + 2, len(path)):
                    if not self.check_collision(path[i], path[j]):
                        sub_path = np.vstack((path[: i + 1], path[j:]))
                sub_paths.append(sub_path)

            costs = np.array([np.linalg.norm(p[1:] - p[:-1]).sum() for p in sub_paths])
            path = sub_paths[np.argmin(costs)]
        return path


class Utils:
    def __init__(self):
        pass

    def draw_marker(self, frame_id, stamp, position, publisher, color="red", id=0):
        if position is None:
            return
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.id = id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        if color == "red":
            marker.color.r = 1.0
        elif color == "green":
            marker.color.g = 1.0
        elif color == "blue":
            marker.color.b = 1.0
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.0
        publisher.publish(marker)

    def draw_marker_array(self, frame_id, stamp, positions, publisher):
        marker_array = MarkerArray()
        for i, position in enumerate(positions):
            if position is None:
                continue
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = 0.0
            marker.lifetime = Duration(seconds=0.1).to_msg()
            marker_array.markers.append(marker)
        publisher.publish(marker_array)

    def draw_lines(self, frame_id, stamp, path, publisher):
        points = []
        for i in range(len(path) - 1):
            a = path[i]
            b = path[i + 1]
            point = Point()
            point.x = a[0]
            point.y = a[1]
            points.append(copy.deepcopy(point))
            point.x = b[0]
            point.y = b[1]
            points.append(copy.deepcopy(point))

        line_list = Marker()
        line_list.header.frame_id = frame_id
        line_list.header.stamp = stamp
        line_list.id = 0
        line_list.type = line_list.LINE_LIST
        line_list.action = line_list.ADD
        line_list.scale.x = 0.1
        line_list.color.a = 1.0
        line_list.color.r = 0.0
        line_list.color.g = 1.0
        line_list.color.b = 0.0
        line_list.points = points
        publisher.publish(line_list)

    def traverse_grid(self, start, end):
        """
        Bresenham's line algorithm for fast voxel traversal

        CREDIT TO: Rogue Basin
        CODE TAKEN FROM: http://www.roguebasin.com/index.php/Bresenham%27s_Line_Algorithm
        """
        # Setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1

        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)

        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        # Swap start and end points if necessary and store swap state
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1

        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1

        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1

        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
        return points


class WaypointUtils:
    def __init__(
        self,
        node,
        L=1.7,
        interpolation_distance=None,
        filepath="/f1tenth_ws/racelines/e7_floor5.csv",
        min_lookahead=0.5,
        max_lookahead=3.0,
        min_lookahead_speed=3.0,
        max_lookahead_speed=6.0,
        filepath_2nd="/f1tenth_ws/racelines/e7_floor5.csv",
        lane_number=0,
    ):

        self.node = node
        self.L = L  # dynamic lookahead distance
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.min_lookahead_speed = min_lookahead_speed
        self.max_lookahead_speed = max_lookahead_speed

        self.waypoints_world, self.velocities = self.load_and_interpolate_waypoints(
            file_path=filepath, interpolation_distance=interpolation_distance
        )

        # For competition, where I want to customize the lanes that I am using
        self.lane_number = lane_number
        self.waypoints_world_2nd, self.velocities_2nd = self.load_and_interpolate_waypoints(
            file_path=filepath_2nd, interpolation_distance=interpolation_distance
        )

        self.index = 0
        self.velocity_index = 0
        print(f"Loaded {len(self.waypoints_world)} waypoints")

    def transform_waypoints(self, waypoints, car_position, pose):
        # translation
        waypoints = waypoints - car_position

        # rotation
        quaternion = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        waypoints = R.inv(R.from_quat(quaternion)).apply(waypoints)

        return waypoints

    def load_and_interpolate_waypoints(self, file_path, interpolation_distance=0.05):
        # Read waypoints from csv, first two columns are x and y, third column is velocity
        # Exclude last row, because that closes the loop
        points = np.genfromtxt(file_path, delimiter=",")[:, :2]
        velocities = np.genfromtxt(file_path, delimiter=",")[:, 2]

        # Add first point as last point to complete loop
        self.node.get_logger().info(str(velocities))

        # interpolate, not generally needed because interpolation can be done with the solver, where you feed in target distance between points
        if interpolation_distance != 0 and interpolation_distance is not None:
            # Calculate the cumulative distances between points
            distances = np.sqrt(np.sum(np.diff(points, axis=0) ** 2, axis=1))
            cumulative_distances = np.insert(np.cumsum(distances), 0, 0)

            # Calculate the number of segments based on the desired distance threshold
            total_distance = cumulative_distances[-1]
            segments = int(total_distance / interpolation_distance)

            # Linear length along the line
            distance = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=0) ** 2, axis=1)))
            # Normalize distance between 0 and 1
            distance = np.insert(distance, 0, 0) / distance[-1]

            # Interpolate
            alpha = np.linspace(0, 1, segments)
            interpolator = interp1d(distance, points, kind="slinear", axis=0)
            interpolated_points = interpolator(alpha)

            # Interpolate velocities
            velocity_interpolator = interp1d(distance, velocities, kind="slinear")
            interpolated_velocities = velocity_interpolator(alpha)

            # Add z-coordinate to be 0
            interpolated_points = np.hstack((interpolated_points, np.zeros((interpolated_points.shape[0], 1))))
            assert len(interpolated_points) == len(interpolated_velocities)
            return interpolated_points, interpolated_velocities

        else:
            # Add z-coordinate to be 0
            points = np.hstack((points, np.zeros((points.shape[0], 1))))
            return points, velocities

    def get_closest_waypoint_with_velocity(self, pose):
        # get current position of car
        if pose is None:
            return

        position = (pose.position.x, pose.position.y, 0)

        # transform way-points from world to vehicle frame of reference
        if self.lane_number == 0:
            waypoints_car = self.transform_waypoints(self.waypoints_world, position, pose)
        else:
            waypoints_car = self.transform_waypoints(self.waypoints_world_2nd, position, pose)

        # get distance from car to all waypoints
        distances = np.linalg.norm(waypoints_car, axis=1)

        # get indices of waypoints sorted by ascending distance
        self.velocity_index = np.argmin(distances)

        if self.lane_number == 0:
            return self.waypoints_world[self.velocity_index], self.velocities[self.velocity_index]
        else:
            return self.waypoints_world_2nd[self.velocity_index], self.velocities_2nd[self.velocity_index]

    def get_waypoint_stanley(self, pose):
        # get current position of car
        if pose is None:
            return
        position = (pose.position.x, pose.position.y, 0)

        # transform way-points from world to vehicle frame of reference
        if self.lane_number == 0:
            waypoints_car = self.transform_waypoints(self.waypoints_world, position, pose)
        else:
            waypoints_car = self.transform_waypoints(self.waypoints_world_2nd, position, pose)

        # get distance from car to all waypoints
        distances = np.linalg.norm(waypoints_car, axis=1)

        # get indices of waypoints sorted by ascending distance
        index = np.argmin(distances)

        if self.lane_number == 0:
            return waypoints_car[index], self.waypoints_world[index]
        else:
            return waypoints_car[index], self.waypoints_world_2nd[index]

    def get_waypoint(self, pose, target_velocity, fixed_lookahead=None):
        # get current position of car
        if pose is None:
            return
        position = (pose.position.x, pose.position.y, 0)

        # transform way-points from world to vehicle frame of reference
        if self.lane_number == 0:
            waypoints_car = self.transform_waypoints(self.waypoints_world, position, pose)
        else:
            waypoints_car = self.transform_waypoints(self.waypoints_world_2nd, position, pose)

        # get distance from car to all waypoints
        distances = np.linalg.norm(waypoints_car, axis=1)

        # get indices of waypoints that are within L, sorted by descending distance
        # Use dynamic lookahead for this part

        if fixed_lookahead:
            self.L = fixed_lookahead
        else:
            # Lookahead is proportional to velocity
            self.L = min(
                max(
                    self.min_lookahead,
                    self.min_lookahead
                    + (self.max_lookahead - self.min_lookahead)
                    * (target_velocity - self.min_lookahead_speed)
                    / (self.max_lookahead_speed - self.min_lookahead_speed),
                ),
                self.max_lookahead,
            )

        indices_L = np.argsort(np.where(distances < self.L, distances, -1))[::-1]

        # set goal point to be the farthest valid waypoint within distance L
        for i in indices_L:
            # check waypoint is in front of car
            x = waypoints_car[i][0]
            if x > 0:
                self.index = i
                if self.lane_number == 0:
                    return waypoints_car[self.index], self.waypoints_world[self.index]
                else:
                    return waypoints_car[self.index], self.waypoints_world_2nd[self.index]
        return None, None


def main(args=None):
    rclpy.init(args=args)
    print("Stanley Avoidance Initialized")
    stanley_avoidance_node = StanleyAvoidance()
    rclpy.spin(stanley_avoidance_node)

    stanley_avoidance_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
