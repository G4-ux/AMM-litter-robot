#!/usr/bin/env python3
"""Coordinate mapping, frontier exploration, approach, and litter collection."""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node
from std_msgs.msg import String
import tf2_ros


def yaw_to_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class MissionManager(Node):
    TARGETS = {'soda_can_red', 'soda_can_blue', 'litter_1', 'litter_2', 'litter_3'}

    def __init__(self):
        super().__init__('mission_manager')
        self.state = 'INITIALIZING'
        self.collected_items = set()
        self.latest_map = None
        self.target_name = None
        self.target_pose_base = None
        self.last_detection_time = None
        self.navigation_active = False
        self.last_frontier_goal = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.navigator = BasicNavigator()

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pick_publisher = self.create_publisher(String, '/trigger_pick', 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(PoseStamped, '/detected_litter_relative', self.detection_callback, 10)
        self.create_subscription(String, '/detected_litter_name', self.detection_name_callback, 10)
        self.create_subscription(String, '/manipulator_status', self.manipulator_status_callback, 10)
        self.create_timer(0.5, self.state_machine_tick)
        self.get_logger().info('Mission manager is waiting for SLAM and Nav2 data.')

    def map_callback(self, message):
        self.latest_map = message

    def detection_name_callback(self, message):
        if message.data in self.TARGETS and message.data not in self.collected_items:
            self.target_name = message.data

    def detection_callback(self, message):
        if self.target_name is None or self.target_name in self.collected_items:
            return
        self.target_pose_base = message
        self.last_detection_time = self.get_clock().now()

    def manipulator_status_callback(self, message):
        if not message.data.startswith('COMPLETED:'):
            return
        completed_target = message.data.split(':', 1)[1]
        self.collected_items.add(completed_target)
        self.target_name = None
        self.target_pose_base = None
        self.navigation_active = False
        self.last_frontier_goal = None
        self.state = 'EXPLORING'
        self.get_logger().info(
            f'Collected {completed_target}; {len(self.collected_items)}/{len(self.TARGETS)} targets complete.'
        )

    def robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return None
        position = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return position.x, position.y, yaw

    def find_frontier_goal(self):
        if self.latest_map is None:
            return None
        robot_pose = self.robot_pose()
        if robot_pose is None:
            return None

        info = self.latest_map.info
        grid = np.asarray(self.latest_map.data, dtype=np.int8).reshape((info.height, info.width))
        free = grid == 0
        unknown = grid == -1
        unknown_neighbour = np.zeros_like(unknown)
        for row_offset, column_offset in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            unknown_neighbour |= np.roll(np.roll(unknown, row_offset, axis=0), column_offset, axis=1)
        frontier = free & unknown_neighbour
        frontier[[0, -1], :] = False
        frontier[:, [0, -1]] = False
        rows, columns = np.where(frontier)
        if len(columns) == 0:
            return None

        robot_x, robot_y, _ = robot_pose
        best_goal = None
        best_distance = float('inf')
        for index in range(0, len(columns), max(1, len(columns) // 80)):
            goal_x = info.origin.position.x + (columns[index] + 0.5) * info.resolution
            goal_y = info.origin.position.y + (rows[index] + 0.5) * info.resolution
            distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
            if 0.8 < distance < 4.5 and distance < best_distance:
                best_goal = (goal_x, goal_y)
                best_distance = distance
        return best_goal

    def send_frontier_goal(self):
        if not self.navigator.nav_to_pose_client.server_is_ready():
            self.get_logger().info('NavigateToPose is not active yet; waiting for Nav2 lifecycle startup.')
            return
        frontier = self.find_frontier_goal()
        if frontier is None:
            return
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = frontier
        goal.pose.orientation.w = 1.0
        self.navigator.goToPose(goal)
        self.navigation_active = True
        self.last_frontier_goal = frontier
        self.get_logger().info(f'Exploring frontier at ({frontier[0]:.2f}, {frontier[1]:.2f}).')

    def target_is_fresh(self):
        if self.target_pose_base is None or self.last_detection_time is None:
            return False
        age = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        return age < 1.5

    def approach_target(self):
        if not self.target_is_fresh():
            self.cmd_vel_publisher.publish(Twist())
            self.state = 'EXPLORING'
            return
        position = self.target_pose_base.pose.position
        distance = math.hypot(position.x, position.y)
        heading = math.atan2(position.y, position.x)
        distance_error = distance - 0.35
        command = Twist()
        if abs(heading) > 0.12:
            command.angular.z = max(-0.35, min(0.35, heading))
        elif abs(distance_error) > 0.05:
            command.linear.x = max(-0.10, min(0.12, 0.5 * distance_error))
        else:
            self.cmd_vel_publisher.publish(command)
            self.pick_publisher.publish(String(data=self.target_name))
            self.state = 'PICKING'
            self.get_logger().info(f'Picking {self.target_name}.')
            return
        self.cmd_vel_publisher.publish(command)

    def state_machine_tick(self):
        if self.state == 'INITIALIZING':
            if self.latest_map is not None and self.robot_pose() is not None:
                self.state = 'EXPLORING'
                self.get_logger().info('SLAM map and robot transform received; starting exploration.')
            return

        if self.state == 'EXPLORING':
            if self.collected_items == self.TARGETS:
                self.cmd_vel_publisher.publish(Twist())
                self.state = 'COMPLETED'
                self.get_logger().info('All litter has been deposited in the onboard bin.')
                return
            if self.target_is_fresh() and self.target_name not in self.collected_items:
                if self.navigation_active:
                    self.navigator.cancelTask()
                    self.navigation_active = False
                self.state = 'APPROACHING'
                return
            if not self.navigation_active or self.navigator.isTaskComplete():
                self.navigation_active = False
                self.send_frontier_goal()
            return

        if self.state == 'APPROACHING':
            self.approach_target()
            return

        if self.state == 'PICKING':
            self.cmd_vel_publisher.publish(Twist())
            return

        if self.state == 'COMPLETED':
            self.cmd_vel_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
