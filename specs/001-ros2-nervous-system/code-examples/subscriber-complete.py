#!/usr/bin/env python3
"""
Complete Subscriber Example
Demonstrates receiving and processing messages from a topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        # Create subscription: message type, topic name, callback, queue size
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )
        # Prevent unused variable warning
        self.subscription

        self.get_logger().info('Subscriber initialized, listening on "topic"')

    def listener_callback(self, msg):
        """Callback function invoked when message received"""
        self.get_logger().info(f'I heard: "{msg.data}"')
        # Process message here (e.g., update internal state, trigger actions)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)  # Process callbacks until shutdown
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
