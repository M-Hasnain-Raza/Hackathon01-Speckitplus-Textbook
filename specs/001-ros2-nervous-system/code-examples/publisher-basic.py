#!/usr/bin/env python3
"""
Basic Publisher Example
Demonstrates creating a publisher and sending velocity commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')

        # Create publisher: message type, topic name, queue size
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create timer to publish at 1 Hz (every 1.0 seconds)
        self.timer = self.create_timer(1.0, self.publish_velocity)

        self.get_logger().info('Velocity publisher initialized')

    def publish_velocity(self):
        """Publish velocity command to move robot forward"""
        msg = Twist()
        msg.linear.x = 0.5   # Move forward at 0.5 m/s
        msg.angular.z = 0.1  # Turn left at 0.1 rad/s

        self.publisher.publish(msg)
        self.get_logger().info(f'Published: linear.x={msg.linear.x}, angular.z={msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
