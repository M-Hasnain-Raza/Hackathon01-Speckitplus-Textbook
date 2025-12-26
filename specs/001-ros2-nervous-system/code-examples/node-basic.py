#!/usr/bin/env python3
"""
Basic ROS 2 Node Example
Demonstrates minimal node initialization, spinning, and shutdown
"""

import rclpy
from rclpy.node import Node


def main(args=None):
    # Initialize ROS 2 communications for this context
    rclpy.init(args=args)

    # Create a node with name 'basic_node'
    node = Node('basic_node')

    # Log a message to confirm node started
    node.get_logger().info('Basic node started successfully')

    # Spin - process callbacks until shutdown signal
    rclpy.spin(node)

    # Clean up node resources
    node.destroy_node()

    # Shutdown ROS 2 communications
    rclpy.shutdown()


if __name__ == '__main__':
    main()
