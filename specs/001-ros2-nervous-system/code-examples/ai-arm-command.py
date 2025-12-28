#!/usr/bin/env python3
"""
AI Arm Command Example
Demonstrates AI agent publishing joint commands to raise left arm
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ArmCommandPublisher(Node):
    def __init__(self):
        super().__init__('arm_command_publisher')

        # Create publisher for arm commands
        self.publisher = self.create_publisher(
            JointState,
            '/left_arm/command',
            10
        )

        self.get_logger().info('Arm command publisher initialized')

    def publish_raise_arm_command(self):
        """Publish command to raise left arm to 90 degrees"""
        msg = JointState()

        # Define joint names
        msg.name = ['shoulder_pitch', 'shoulder_roll', 'elbow']

        # Set target positions (in radians: 90° = 1.57 rad)
        msg.position = [1.57, 0.0, 1.57]

        # Publish command
        self.publisher.publish(msg)
        self.get_logger().info('Published command: Raise left arm to 90°')


def main(args=None):
    rclpy.init(args=args)
    node = ArmCommandPublisher()

    # Publish command once
    node.publish_raise_arm_command()

    # Spin briefly to ensure message is sent
    rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
