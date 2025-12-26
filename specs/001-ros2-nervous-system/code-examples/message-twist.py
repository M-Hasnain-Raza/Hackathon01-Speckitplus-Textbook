#!/usr/bin/env python3
"""
Twist Message Example
Demonstrates creating and populating a geometry_msgs/Twist message
"""

from geometry_msgs.msg import Twist


def create_twist_message():
    """
    Create a Twist message for robot movement commands
    Twist contains linear and angular velocity vectors
    """
    msg = Twist()

    # Set linear velocity (meters per second)
    msg.linear.x = 0.5  # Move forward at 0.5 m/s
    msg.linear.y = 0.0  # No sideways movement
    msg.linear.z = 0.0  # No vertical movement

    # Set angular velocity (radians per second)
    msg.angular.x = 0.0  # No roll
    msg.angular.y = 0.0  # No pitch
    msg.angular.z = 0.1  # Turn left at 0.1 rad/s

    return msg


if __name__ == '__main__':
    # Example: Create and inspect Twist message
    twist = create_twist_message()
    print(f"Linear velocity: x={twist.linear.x}, y={twist.linear.y}, z={twist.linear.z}")
    print(f"Angular velocity: x={twist.angular.x}, y={twist.angular.y}, z={twist.angular.z}")

    # In real code, this message would be published:
    # publisher.publish(twist)
