#!/usr/bin/env python3
"""
Service Call Example
Demonstrates calling a ROS 2 service and waiting for response
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args)
    node = Node('service_client')

    # Create service client: service type, service name
    client = node.create_client(AddTwoInts, 'add_two_ints')

    # Wait for service to be available
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting...')

    # Create request
    request = AddTwoInts.Request()
    request.a = 5
    request.b = 3

    # Call service asynchronously
    future = client.call_async(request)

    # Wait for response (blocking)
    rclpy.spin_until_future_complete(node, future)

    # Process response
    response = future.result()
    node.get_logger().info(f'Result: {request.a} + {request.b} = {response.sum}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
