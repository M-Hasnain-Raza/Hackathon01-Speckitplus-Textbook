#!/usr/bin/env python3
"""
Complete Publisher Example
Full example with initialization, publishing, and cleanup
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        # Create publisher: message type, topic name, queue size
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Create timer to publish at 2 Hz (every 0.5 seconds)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.i = 0  # Message counter

        self.get_logger().info('Publisher initialized')

    def timer_callback(self):
        """Callback invoked by timer - publishes message"""
        msg = String()
        msg.data = f'Hello World: {self.i}'

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.i += 1


def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create node
    minimal_publisher = MinimalPublisher()

    # Spin - process callbacks until shutdown
    rclpy.spin(minimal_publisher)

    # Clean up
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
