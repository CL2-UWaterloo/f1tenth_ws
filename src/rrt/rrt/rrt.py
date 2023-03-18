#!/usr/bin/env python3
"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""

import math
import copy
import numpy as np
from scipy import signal
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R

import rclpy
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


class RRT(Node):
    def __init__(self):
        super().__init__('rrt')

        self.declare_parameter('waypoints_path', '/f1tenth_ws/src/pure_pursuit/racelines/e7_floor5.csv')
        self.declare_parameter('odom_topic', '/pf/pose/odom')
        self.declare_parameter('lookahead_distance', 3.0)
        self.declare_parameter('K_p', 0.5)
        self.declare_parameter('segments', 1024)
        self.declare_parameter('velocity_min', 0.5)
        self.declare_parameter('velocity_max', 2.0)
        self.declare_parameter('steering_limit', 25.0)
        self.declare_parameter('cells_per_meter', 10)
        self.declare_parameter('is_sim', False)
        
        self.waypoints_path = str(self.get_parameter('waypoints_path').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.L = float(self.get_parameter('lookahead_distance').value)
        self.K_p = float(self.get_parameter('K_p').value)
        self.segments = int(self.get_parameter('segments').value)
        self.velocity_min = float(self.get_parameter('velocity_min').value)
        self.velocity_max = float(self.get_parameter('velocity_max').value)
        self.steering_limit = float(self.get_parameter('steering_limit').value)
        self.CELLS_PER_METER = int(self.get_parameter('cells_per_meter').value)
        self.is_sim = bool(self.get_parameter('is_sim').value)
        

        # hyper-parameters
        self.populate_free = True

        self.pure_pursuit = PurePursuit(L=self.L, segments=self.segments, filepath=self.waypoints_path)
        self.get_logger().info(f"Loaded {len(self.pure_pursuit.waypoints)} waypoints")
        self.utils = Utils()


        self.pose_sub = self.create_subscription(Odometry, self.odom_topic, self.pose_callback, 1)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 1)

        # publishers
        self.waypoint_pub = self.create_publisher(Marker, "/waypoint_marker", 10)
        self.rrt_path_pub = self.create_publisher(Marker, "/rrt_path", 10)
        self.rrt_node_pub = self.create_publisher(MarkerArray, "/rrt_node_array", 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.occupancy_grid_pub = self.create_publisher(OccupancyGrid, "/occupancy_grid", 10)


        # constants
        self.MAX_RANGE = self.L - 0.1
        self.MIN_ANGLE = np.radians(0) #the ANGLE_OFFSET is used to map stuff from -pi,pi to 0 to 180 with ANGLE_OFFSET (look at oc grid formation below)
        self.MAX_ANGLE = np.radians(180)
        self.ANGLE_OFFSET = np.radians(45) #fov = 270. One side = fov/2 = 135. 135-45 = 90 (for occupancy grid)
        self.IS_OCCUPIED = 100
        self.IS_FREE = 0
        self.MAX_RRT_ITER = 100

        # class variables
        self.grid_height = int(self.L * self.CELLS_PER_METER) # in number of cells
        self.grid_width = int(60)
        self.CELL_Y_OFFSET = (self.grid_width // 2) - 1 #cartesian frame orientation same as that of car local ref frame.
        self.occupancy_grid = np.full(shape=(self.grid_height, self.grid_width), fill_value=-1, dtype=int)
        self.current_pose = None
        self.goal_pos = None
        # rrt variables
        self.path_world = []


    def local_to_grid(self, x, y):
        i = int(x * -self.CELLS_PER_METER + (self.grid_height -1))
        j = int(y * -self.CELLS_PER_METER + self.CELL_Y_OFFSET)
        return (i, j)

    def grid_to_local(self, point):
        i, j = point[0], point[1]
        x = (i - (self.grid_height - 1))  / -self.CELLS_PER_METER
        y = (j - self.CELL_Y_OFFSET) / -self.CELLS_PER_METER
        return (x, y)

    def pose_callback(self, pose_msg: Odometry):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens
        """
        # determine pose data type (sim vs. car)
        pose = pose_msg.pose.pose
        self.current_velocity = np.sqrt(pose_msg.twist.twist.linear.x**2 +  pose_msg.twist.twist.linear.y**2)
        # TODO: Implement dynamic lookahead

        # save current car pose
        self.current_pose = copy.deepcopy(pose)
        self.get_logger().info(f"{self.current_pose}")

        # obtain pure pursuit waypoint
        self.goal_pos, goal_pos_world = self.pure_pursuit.get_waypoint(pose)
        self.utils.draw_marker(
            pose_msg.header.frame_id,
            pose_msg.header.stamp,
            goal_pos_world,
            self.waypoint_pub
        )

    def populate_occupancy_grid(self, ranges, angle_increment):
        """
        Populate occupancy grid using lidar scans and save
        the data in class member variable self.occupancy_grid

        Args:
            scan_msg (LaserScan): message from lidar scan topic
        """
        # reset empty occupacny grid (-1 = unknown)
        self.occupancy_grid = np.full(shape=(self.grid_height, self.grid_width), fill_value=-1, dtype=int)

        # enumerate over lidar scans
        for idx, dist in enumerate(ranges):
            # skip scans behind the car
            theta = (idx * angle_increment) - self.ANGLE_OFFSET
            if theta < self.MIN_ANGLE or theta > self.MAX_ANGLE:
                continue

            # obtain local coordinate of scan
            dist_clipped = np.clip(dist, 0, self.MAX_RANGE)
            x = dist_clipped * np.sin(theta)
            y = dist_clipped * np.cos(theta) * -1
            if abs(x) > (self.grid_height / self.CELLS_PER_METER) or abs(y) > (self.grid_width / self.CELLS_PER_METER):
                print(f"Invalid coordinates: ({x}, {y})")
                continue

            # obtain grid indices from local coordinates of scan point
            i, j = self.local_to_grid(x, y)

            # set occupied space
            if dist < self.MAX_RANGE:
                self.occupancy_grid[i, j] = self.IS_OCCUPIED

            #TODO: read up on fast voxel traverse (as compared to flood fill)
            # set free space by fast voxel traverse
            if self.populate_free:
                free_cells = self.utils.traverse_grid(self.local_to_grid(0, 0), (i, j))
                for cell in free_cells:
                    if self.occupancy_grid[cell] != self.IS_OCCUPIED:
                        self.occupancy_grid[cell] = self.IS_FREE

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
        oc.info.origin.position.y -= ((self.grid_width / 2) + 1) / 10
        oc.info.width = self.grid_height
        oc.info.height = self.grid_width
        oc.info.resolution = 0.1
        oc.data = np.fliplr(np.rot90(self.occupancy_grid, k=1)).flatten().tolist()
        self.occupancy_grid_pub.publish(oc)

    def convolve_occupancy_grid(self):
        kernel = np.ones(shape=[2, 2])
        self.occupancy_grid = signal.convolve2d(
            self.occupancy_grid.astype('int'),
            kernel.astype('int'),
            boundary='symm',
            mode='same'
        )
        self.occupancy_grid = np.clip(self.occupancy_grid, -1, 100)

    def drive_to_target(self, point):
        # calculate curvature/steering angle
        L = np.linalg.norm(point)
        y = point[1]
        angle = self.K_p * (2 * y) / (L**2)
        angle = np.clip(angle, -np.radians(self.steering_limit), np.radians(self.steering_limit))

        # determine velocity
        logit = np.clip(
            (2.0 / (1.0 + np.exp(-5 * (np.abs(np.degrees(angle)) / self.steering_limit)))) - 1.0,
            0.0,
            1.0
        )
        velocity = self.velocity_max - (logit * (self.velocity_max - self.velocity_min))

        # publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed          = velocity
        drive_msg.drive.steering_angle = angle
        self.drive_pub.publish(drive_msg)

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, update occupancy grid and perform RRT

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
        path_grid = self.rrt()

        # convert path from grid to local coordinates
        path_local = [self.grid_to_local(point) for point in path_grid]
        
        self.get_logger().info(f"Path: {path_local}")
        if len(path_local) < 2:
            return

        # navigate to first node in tree
        self.drive_to_target(path_local[1])

        # rrt visualization
        self.utils.draw_marker_array(
            scan_msg.header.frame_id,
            scan_msg.header.stamp,
            path_local,
            self.rrt_node_pub
        )

        self.utils.draw_lines(
            scan_msg.header.frame_id,
            scan_msg.header.stamp,
            path_local,
            self.rrt_path_pub
        )

    def rrt(self):
        # convert position to occupancy grid indices
        current_pos = self.local_to_grid(0, 0)
        goal_pos = self.local_to_grid(self.goal_pos[0], self.goal_pos[1])

        # resample a close point if our goal point is occupied
        if self.occupancy_grid[goal_pos] == self.IS_OCCUPIED:
            i, j = self.sample()
            while np.linalg.norm(np.array([i, j]) - np.array(goal_pos)) > 5:
                i, j = self.sample()
            goal_pos = (i, j)

        # initialize start and goal trees
        T_start = [Vertex(current_pos)]
        T_goal = [Vertex(goal_pos)]

        # start rrt algorithm
        for itn in range(self.MAX_RRT_ITER):
            # sample from free space
            pos_sampled = self.sample()

            # attempt to expand tree using sampled point
            T_start, success_start = self.expand_tree(T_start, pos_sampled, check_closer=True)
            T_goal, success_goal = self.expand_tree(T_goal, pos_sampled)

            # if sampled point can reach both T_start and T_goal
            # get path from start to goal and return
            if success_start and success_goal:
                path = self.find_path(T_start, T_goal, pruning=True)
                return path
        return []

    def sample(self):
        """
        Randomly sample the free space in occupancy grid, and returns its index.
        If free space has already been populated then just check if sampled
        cell is free, else do fast voxel traversal for each sampling.

        Returns:
            (i, j) (int, int): index of free cell in occupancy grid
        """
        if self.populate_free:
            i, j = np.random.randint(self.grid_height), np.random.randint(self.grid_width)
            while self.occupancy_grid[i, j] != self.IS_FREE:
                i, j = np.random.randint(self.grid_height), np.random.randint(self.grid_width)
        else:
            free = False
            while not free:
                i, j = np.random.randint(self.grid_height), np.random.randint(self.grid_width)
                free = True
                for cell in self.utils.traverse_grid(self.local_to_grid(0, 0), (i, j)):
                    if self.occupancy_grid[cell] == self.IS_OCCUPIED:
                        free = False
                        break
        return (i, j)

    def expand_tree(self, tree, sampled_point, check_closer=False):
        """
        Attempts to expand tree using the sampled point by
        checking if it causes collision and if the new node
        brings the car closer to the goal.

        Args:
            tree ([]): current RRT tree
            sampled_point: cell sampled in occupancy grid free space
            check_closer: check if sampled point brings car closer
        Returns:
            tree ([]): expanded RRT tree
            success (bool): whether tree was successfully expanded
        """
        # get closest node to sampled point in tree
        idx_nearest = self.nearest(tree, sampled_point)
        pos_nearest = tree[idx_nearest].pos

        # check if nearest node -> sampled node causes collision
        collision = self.check_collision(sampled_point, pos_nearest)

        # check if sampeld point bring car closer to goal compared to the nearest node
        is_closer = self.is_closer(sampled_point, pos_nearest) if check_closer else True

        # if p_free -> p_nearest causes no collision
        # then add p_free as child of p_nearest in T_start
        if is_closer and (not collision):
            tree.append(Vertex(sampled_point, idx_nearest))

        return tree, (is_closer and (not collision))

    def nearest(self, tree, sampled_cell):
        """
        Return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_cell (i,j): cell sampled in occupancy grid free space
        Returns:
            nearest_indx (int): index of neareset node on the tree
        """
        nearest_indx = -1
        nearest_dist = np.Inf
        for idx, node in enumerate(tree):
            dist = np.linalg.norm(np.array(sampled_cell) - np.array(node.pos))
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_indx = idx
        return nearest_indx

    def is_closer(self, sampled_pos, nearest_pos):
        """
        Checks if the new sampled node brings the car closer
        to the goal point then the nearest existing node on the tree

        Args:
            sampled_pos (i, j): index of sampled pos in occupancy grid
            nearest_pos (i, j): index of nearest pos in occupancy grid
        Returns:
            is_closer (bool): whether the sampled pos brings the car closer
        """
        a = self.grid_to_local(sampled_pos)
        b = self.grid_to_local(nearest_pos)
        return np.linalg.norm(a - self.goal_pos[:2]) < np.linalg.norm(b - self.goal_pos[:2])

    def check_collision(self, cell_a, cell_b):
        """
        Checks whether the path between two cells
        in the occupancy grid is collision free.

        Args:
            cell_a (i, j): index of cell a in occupancy grid
            cell_b (i, j): index of cell b in occupancy grid
        Returns:
            collision (bool): whether path between two cells would cause collision
        """
        for cell in self.utils.traverse_grid(cell_a, cell_b):
            if (cell[0] * cell[1] < 0) or (cell[0] >= self.grid_height) or (cell[1] >= self.grid_width):
                continue
            if self.occupancy_grid[cell] == self.IS_OCCUPIED:
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
                        sub_path = np.vstack((path[:i+1], path[j:]))
                sub_paths.append(sub_path)

            costs = np.array([np.linalg.norm(p[1:] - p[:-1]).sum() for p in sub_paths])
            path = sub_paths[np.argmin(costs)]
        return path


class Utils:
    def __init__(self):
        pass

    def draw_marker(self, frame_id, stamp, position, publisher, id=0):
        if position is None:
            return
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.id = id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
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
            marker.lifetime = Duration(seconds=0.05).to_msg()
            marker_array.markers.append(marker)
        publisher.publish(marker_array)

    def draw_lines(self, frame_id, stamp, path, publisher):
        points = []
        for i in range(len(path) - 1):
            a = path[i]
            b = path[i+1]
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


class PurePursuit:
    def __init__(self, L=1.7, segments=1024, filepath="/f1tenth_ws/src/rrt/racelines/e7_floor5.csv", K_p = 0.5):
        # TODO: Make self.L a function of the current velocity, so we have more intelligent RRT
        self.L = L
        self.waypoints = self.interpolate_waypoints(
            file_path=filepath,
            segments=segments
        )
        self.grid_width = 0.5
        print(f"Loaded {len(self.waypoints)} waypoints")

    def transform_waypoints(self, waypoints, car_position, pose):
        # translation
        waypoints = waypoints - car_position

        # rotation
        quaternion = np.array([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        waypoints = R.inv(R.from_quat(quaternion)).apply(waypoints)

        return waypoints

    def interpolate_waypoints(self, file_path, segments):
        # Read waypoints from csv
        points = np.genfromtxt(file_path, delimiter=",")[:-2, :2] # Exclude last row, because that closes the loop

        # Add first point as last point to complete loop
        points = np.vstack((points, points[0]))

        # Linear length along the line
        distance = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1)))
        distance = np.insert(distance, 0, 0) / distance[-1]

        # Interpolate
        alpha = np.linspace(0, 1, segments)
        interpolator = interp1d(distance, points, kind='slinear', axis=0)
        interpolated_points = interpolator(alpha)

        # Add z-coordinate to be 0
        interpolated_points = np.hstack(
            (interpolated_points, np.zeros((interpolated_points.shape[0], 1)))
        )
        return interpolated_points

    def get_waypoint(self, pose):
        # get current position of car
        position = (pose.position.x, pose.position.y, 0)

        # transform way-points from world to vehicle frame of reference
        waypoints = self.transform_waypoints(self.waypoints, position, pose)

        # get distance from car to all waypoints
        distances = np.linalg.norm(waypoints, axis=1)

        # get indices of waypoints that are within L, sorted by descending distance
        indices_L = np.argsort(np.where(distances < self.L, distances, -1))[::-1]
        self.CELL_Y_OFFSET = (self.grid_width // 2) - 1

        # set goal point to be the farthest valid waypoint within distance L
        for i in indices_L:
            # check waypoint is in front of car
            x = waypoints[i][0]
            if x > 0:
                return waypoints[i], self.waypoints[i]
        return None, None


def main(args=None):
    rclpy.init(args=args)
    print("RRT Initialized")
    rrt_node = RRT()
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()