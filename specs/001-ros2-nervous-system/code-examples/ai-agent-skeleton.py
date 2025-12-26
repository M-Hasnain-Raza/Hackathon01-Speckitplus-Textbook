#!/usr/bin/env python3
"""
AI Agent Skeleton Example
Demonstrates integrating an AI agent as a ROS 2 node
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AIAgent(Node):
    def __init__(self):
        super().__init__('ai_agent')

        # Create publisher for robot commands
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create timer for AI decision loop (10 Hz)
        self.timer = self.create_timer(0.1, self.ai_decision_loop)

        self.get_logger().info('AI Agent initialized')

    def ai_decision_loop(self):
        """Main AI loop: sense, decide, act"""
        # AI decision logic here (machine learning, planning, etc.)
        # This is outside Module 1 scope - placeholder only
        decision = self.make_decision()

        # Publish command via ROS 2
        cmd = Twist()
        cmd.linear.x = decision['speed']
        cmd.angular.z = decision['turn_rate']
        self.cmd_publisher.publish(cmd)

    def make_decision(self):
        """
        Placeholder for AI decision-making logic
        In real agent: process sensors, run ML model, plan actions
        """
        # Simplified example: constant forward movement
        return {'speed': 0.5, 'turn_rate': 0.1}


def main(args=None):
    rclpy.init(args=args)
    agent = AIAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
