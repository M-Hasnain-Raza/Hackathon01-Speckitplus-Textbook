/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 01: ROS 2 Nervous System',
      link: {
        type: 'doc',
        id: 'module-01-ros2-nervous-system/index',
      },
      items: [
        'module-01-ros2-nervous-system/introduction',
        'module-01-ros2-nervous-system/ros2-architecture',
        'module-01-ros2-nervous-system/nodes',
        'module-01-ros2-nervous-system/topics',
        'module-01-ros2-nervous-system/services',
        'module-01-ros2-nervous-system/messages',
        'module-01-ros2-nervous-system/ai-agent-integration',
        'module-01-ros2-nervous-system/rclpy-basics',
        'module-01-ros2-nervous-system/message-flow',
        'module-01-ros2-nervous-system/urdf',
        'module-01-ros2-nervous-system/summary',
      ],
    },
    {
      type: 'category',
      label: 'Module 02: The Digital Twin',
      items: [
        'module-02-digital-twin/introduction-to-digital-twins',
        'module-02-digital-twin/gazebo-fundamentals',
        'module-02-digital-twin/physics-simulation',
        'module-02-digital-twin/sensor-simulation-overview',
        'module-02-digital-twin/lidar-sensors',
        'module-02-digital-twin/depth-cameras',
        'module-02-digital-twin/imu-sensors',
        'module-02-digital-twin/custom-worlds',
        'module-02-digital-twin/unity-visualization',
        'module-02-digital-twin/gazebo-unity-integration',
        'module-02-digital-twin/sim2real-transfer',
        'module-02-digital-twin/programmatic-control',
        'module-02-digital-twin/performance-optimization',
        'module-02-digital-twin/conclusion',
      ],
    },
  ],
};

export default sidebars;
