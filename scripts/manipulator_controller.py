#!/usr/bin/env python3
"""Execute a non-blocking pick-and-deposit sequence for the simulated arm."""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float64, String


class ManipulatorController(Node):
    def __init__(self):
        super().__init__('manipulator_controller')
        self.command_publishers = {
            'waist': self.create_publisher(Float64, '/arm_waist/cmd_pos', 10),
            'shoulder': self.create_publisher(Float64, '/arm_shoulder/cmd_pos', 10),
            'elbow': self.create_publisher(Float64, '/arm_elbow/cmd_pos', 10),
            'wrist': self.create_publisher(Float64, '/arm_wrist/cmd_pos', 10),
            'gripper': self.create_publisher(Float64, '/gripper/cmd_pos', 10),
        }
        self.status_publisher = self.create_publisher(String, '/manipulator_status', 10)
        self.attach_publishers = {}
        self.detach_publishers = {}
        for target in ('soda_can_red', 'soda_can_blue', 'litter_1', 'litter_2', 'litter_3'):
            self.attach_publishers[target] = self.create_publisher(Empty, f'/{target}/attach', 10)
            self.detach_publishers[target] = self.create_publisher(Empty, f'/{target}/detach', 10)

        self.poses = {
            'STOW': {'waist': 0.0, 'shoulder': -1.2, 'elbow': 1.8, 'wrist': 0.8, 'gripper': 0.0},
            'PRE_GRASP': {'waist': 0.0, 'shoulder': 0.6, 'elbow': 0.8, 'wrist': -0.4, 'gripper': 0.018},
            'GRASP': {'waist': 0.0, 'shoulder': 0.95, 'elbow': 1.15, 'wrist': -0.75, 'gripper': 0.0},
            'LIFT': {'waist': 0.0, 'shoulder': 0.2, 'elbow': 1.0, 'wrist': -0.2, 'gripper': 0.0},
            'OVER_BIN': {'waist': math.pi, 'shoulder': 0.45, 'elbow': 0.9, 'wrist': 0.3, 'gripper': 0.0},
            'RELEASE': {'waist': math.pi, 'shoulder': 0.45, 'elbow': 0.9, 'wrist': 0.3, 'gripper': 0.018},
        }
        self.current_pose = dict(self.poses['STOW'])
        self.sequence = []
        self.sequence_index = 0
        self.sequence_start = None
        self.target_name = None

        self.create_subscription(String, '/trigger_pick', self.pick_callback, 10)
        self.create_timer(0.05, self.sequence_tick)
        self.publish_pose(self.current_pose)
        self.publish_status('READY_STOW')

    def publish_status(self, status):
        self.status_publisher.publish(String(data=status))

    def publish_pose(self, pose):
        for joint_name, value in pose.items():
            self.command_publishers[joint_name].publish(Float64(data=float(value)))

    def pick_callback(self, message):
        target_name = message.data.strip()
        if self.sequence:
            self.get_logger().warn('Ignoring pick request while the manipulator is busy.')
            return
        if target_name not in self.attach_publishers:
            self.get_logger().error(f'Unsupported litter model: {target_name}')
            self.publish_status(f'FAILED:{target_name}')
            return

        self.target_name = target_name
        self.sequence = [
            ('PRE_GRASP', self.poses['PRE_GRASP'], 1.8, None),
            ('GRASP', self.poses['GRASP'], 1.5, 'attach'),
            ('LIFT', self.poses['LIFT'], 1.8, None),
            ('OVER_BIN', self.poses['OVER_BIN'], 2.5, None),
            ('RELEASE', self.poses['RELEASE'], 0.8, 'detach'),
            ('STOW', self.poses['STOW'], 2.2, 'complete'),
        ]
        self.sequence_index = 0
        self.sequence_start = self.get_clock().now()
        self.segment_start_pose = dict(self.current_pose)
        self.publish_status(f'PRE_GRASP:{target_name}')
        self.get_logger().info(f'Starting pick-and-deposit cycle for {target_name}.')

    def sequence_tick(self):
        if not self.sequence:
            return

        phase, target_pose, duration, event = self.sequence[self.sequence_index]
        elapsed = (self.get_clock().now() - self.sequence_start).nanoseconds / 1e9
        alpha = min(1.0, elapsed / duration)
        smooth_alpha = (1.0 - math.cos(math.pi * alpha)) / 2.0
        interpolated_pose = {
            joint: self.segment_start_pose[joint] + smooth_alpha * (target_pose[joint] - self.segment_start_pose[joint])
            for joint in target_pose
        }
        self.publish_pose(interpolated_pose)
        if alpha < 1.0:
            return

        self.current_pose = dict(target_pose)
        if event == 'attach':
            self.attach_publishers[self.target_name].publish(Empty())
            self.publish_status(f'LIFTING:{self.target_name}')
        elif event == 'detach':
            self.detach_publishers[self.target_name].publish(Empty())
            self.publish_status(f'RELEASING_INTO_BIN:{self.target_name}')
        elif event == 'complete':
            self.publish_status(f'COMPLETED:{self.target_name}')
            self.get_logger().info(f'Collected {self.target_name} into the onboard bin.')
            self.sequence = []
            self.target_name = None
            return

        self.sequence_index += 1
        next_phase = self.sequence[self.sequence_index][0]
        self.segment_start_pose = dict(self.current_pose)
        self.sequence_start = self.get_clock().now()
        self.publish_status(f'{next_phase}:{self.target_name}')


def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
