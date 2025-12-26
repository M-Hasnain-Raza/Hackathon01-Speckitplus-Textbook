# Section Structure: ROS 2 — The Robotic Nervous System Documentation Module

**Date**: 2025-12-26
**Purpose**: Detailed outline for all 11 documentation sections
**Input**: plan.md, research.md, spec.md

---

## Section 01: Introduction — The Robotic Nervous System

**File**: `docs/module-01-ros2-nervous-system/01-introduction.md`
**Primary Concept**: ROS 2's role as middleware connecting AI brains to robot bodies
**Learning Outcome**: Reader can explain why physical AI systems need ROS 2
**Word Count Target**: 400-600 words
**Reading Time**: ~3-5 minutes

### Content Outline

#### H1: Introduction

#### H2: The Physical AI Challenge
- Problem statement: How does an AI decision become physical robot motion?
- Gap between software intelligence and physical actuation
- Need for communication infrastructure

#### H2: The Nervous System Analogy
- Brain = AI decision-making
- Nerves = ROS 2 communication layer
- Muscles = Robot actuators
- Visual: Mermaid diagram showing analogy mapping

#### H2: What is ROS 2?
- Definition: Middleware framework for robot software
- Purpose: Connecting independent software components
- Key benefit: Modularity and reusability

#### H2: Module 1 Preview
- What you'll learn (brief bullet list)
- What's NOT covered (brief note on scope)

### Diagram Specifications
- **nervous-system-analogy.mmd**: Flowchart showing brain→nerves→muscles mapping to AI→ROS 2→Robot

### Code Examples
- None (introductory section)

### Comprehension Questions
1. In the nervous system analogy, what does ROS 2 represent?
2. What problem does ROS 2 solve for physical AI systems?
3. Name one key benefit of using ROS 2 for robot software.

### Frontmatter Template
```yaml
---
title: "Introduction: The Robotic Nervous System"
description: "Understand ROS 2's role as the communication layer connecting AI decision-making to robot actuation"
keywords: [ros2, middleware, physical ai, nervous system, robotics]
learning_outcome: "Explain why physical AI systems need ROS 2"
---
```

---

## Section 02: ROS 2 Architecture Overview

**File**: `docs/module-01-ros2-nervous-system/02-ros2-architecture.md`
**Primary Concept**: High-level ROS 2 distributed system architecture
**Learning Outcome**: Reader understands distributed node architecture vs. monolithic systems
**Word Count Target**: 600-800 words
**Reading Time**: ~5-7 minutes

### Content Outline

#### H1: ROS 2 Architecture Overview

#### H2: Distributed vs. Monolithic Architecture
- Monolithic: Single program controlling entire robot
- Distributed: Multiple independent programs (nodes) collaborating
- Comparison table

#### H2: The ROS 2 Graph
- Nodes as computational processes
- Communication via topics and services
- Diagram: Multi-node system with communication channels

#### H2: Why Distributed Architecture?
- **Modularity**: Components can be developed independently
- **Fault Isolation**: One node failure doesn't crash entire system
- **Reusability**: Nodes can be reused across robots
- **Scalability**: Add nodes without rewriting existing code

#### H2: DDS Middleware Layer
- Brief mention of Data Distribution Service (DDS)
- ROS 2 uses DDS for inter-process communication
- Note: Implementation details in advanced modules

### Diagram Specifications
- **node-architecture.mmd**: Architecture diagram showing nodes communicating via topics/services

### Code Examples
- None (architectural overview)

### Comprehension Questions
1. What are the advantages of ROS 2's distributed architecture over monolithic systems?
2. What is a "node" in ROS 2?
3. How do nodes communicate with each other in ROS 2?

### Frontmatter Template
```yaml
---
title: "ROS 2 Architecture Overview"
description: "Learn about ROS 2's distributed node architecture and why it matters for robotics"
keywords: [ros2, architecture, distributed systems, nodes, dds]
learning_outcome: "Contrast distributed ROS 2 architecture with monolithic robot control"
---
```

---

## Section 03: ROS 2 Nodes

**File**: `docs/module-01-ros2-nervous-system/03-nodes.md`
**Primary Concept**: Nodes as computational processes
**Learning Outcome**: Reader can describe nodes and provide humanoid robotics examples
**Word Count Target**: 600-800 words
**Reading Time**: ~5-7 minutes

### Content Outline

#### H1: ROS 2 Nodes

#### H2: What is a Node?
- Definition: A node is a process performing one computational task
- Each node runs independently
- Multiple nodes work together to form a robot system

#### H2: Node Examples in Humanoid Robotics
- `joint_controller_node`: Controls joint positions
- `vision_processor_node`: Processes camera data
- `ai_decision_node`: Makes high-level decisions
- `imu_sensor_node`: Reads inertial measurement unit data

#### H2: Node Lifecycle
- **Initialize**: Set up resources and connections
- **Spin**: Process incoming messages and callbacks
- **Shutdown**: Clean up resources

#### H2: Decomposition Principle
- One node, one responsibility
- Benefits: easier testing, debugging, reuse
- Example: Separate nodes for left arm vs. right arm control

### Diagram Specifications
- None specific to this section (can reuse node-architecture.mmd from Section 02)

### Code Examples
- **node-basic.py**: Basic rclpy node initialization (10-15 lines)
  ```python
  import rclpy
  from rclpy.node import Node

  def main():
      rclpy.init()  # Initialize ROS communications
      node = Node('basic_node')  # Create node
      node.get_logger().info('Node started')
      rclpy.spin(node)  # Process callbacks
      node.destroy_node()
      rclpy.shutdown()  # Clean shutdown
  ```

### Comprehension Questions
1. Give three examples of nodes in a humanoid robot system.
2. What is the "one node, one responsibility" principle?
3. What happens during the "spin" phase of a node's lifecycle?

### Frontmatter Template
```yaml
---
title: "ROS 2 Nodes: The Building Blocks"
description: "Understand ROS 2 nodes as independent computational processes in robot systems"
keywords: [ros2, nodes, rclpy, processes, decomposition]
learning_outcome: "Describe nodes and provide humanoid robotics examples"
---
```

---

## Section 04: ROS 2 Topics

**File**: `docs/module-01-ros2-nervous-system/04-topics.md`
**Primary Concept**: Publish/subscribe communication for continuous data streams
**Learning Outcome**: Reader can explain pub/sub pattern and identify topic use cases
**Word Count Target**: 700-900 words
**Reading Time**: ~6-8 minutes

### Content Outline

#### H1: ROS 2 Topics

#### H2: What is a Topic?
- Definition: Named channel for asynchronous message passing
- Publisher/subscriber pattern
- Many-to-many communication

#### H2: The Publish/Subscribe Pattern
- Publishers send messages to topics
- Subscribers receive messages from topics
- Decoupled: Publishers don't know who's subscribing
- Diagram: Publisher → Topic → Subscriber flow

#### H2: When to Use Topics
- **Continuous data streams**: Sensor readings, position updates
- **Asynchronous communication**: Don't need immediate response
- **Broadcast data**: Multiple nodes need same information
- Examples: Camera images, joint positions, movement commands

#### H2: Topic Naming Conventions
- Use namespaces: `/robot1/camera/image`
- Descriptive names: `/left_arm/joint_states`
- Avoid generic names like `/data` or `/output`

#### H2: Topics vs. Services (Preview)
- Topics: Continuous, asynchronous, many-to-many
- Services: One-time, synchronous, one-to-one (detailed in Section 05)

### Diagram Specifications
- **topic-flow.mmd**: Sequence diagram showing publisher→topic→subscriber message flow

### Code Examples
- **publisher-basic.py**: Publisher sending movement commands (15-20 lines)
  ```python
  import rclpy
  from rclpy.node import Node
  from geometry_msgs.msg import Twist

  class VelocityPublisher(Node):
      def __init__(self):
          super().__init__('velocity_publisher')
          self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
          self.timer = self.create_timer(1.0, self.publish_velocity)

      def publish_velocity(self):
          msg = Twist()
          msg.linear.x = 0.5  # Move forward
          msg.angular.z = 0.1  # Turn slightly
          self.publisher.publish(msg)
          self.get_logger().info('Published velocity command')
  ```

### Comprehension Questions
1. When should you use a topic vs. other communication methods?
2. What is the pub/sub pattern and why is it useful?
3. Can multiple nodes subscribe to the same topic? Why is this useful?

### Frontmatter Template
```yaml
---
title: "ROS 2 Topics: Publish/Subscribe Communication"
description: "Learn about ROS 2 topics for continuous, asynchronous data streams"
keywords: [ros2, topics, publish, subscribe, messaging, pub/sub]
learning_outcome: "Explain pub/sub pattern and identify topic use cases"
---
```

---

## Section 05: ROS 2 Services

**File**: `docs/module-01-ros2-nervous-system/05-services.md`
**Primary Concept**: Request/response communication for synchronous operations
**Learning Outcome**: Reader can contrast topics vs. services
**Word Count Target**: 600-800 words
**Reading Time**: ~5-7 minutes

### Content Outline

#### H1: ROS 2 Services

#### H2: What is a Service?
- Definition: Synchronous request/response pattern
- Client sends request, server processes, sends response
- One-to-one communication

#### H2: Service Lifecycle
- Client sends request
- Server processes (blocking operation)
- Server returns response
- Diagram: Request/response flow

#### H2: When to Use Services
- **Configuration changes**: Set parameters, update settings
- **State queries**: Get current robot state
- **Infrequent operations**: Not continuous like topics
- **Synchronous operations**: Need immediate response
- Examples: Query battery level, trigger calibration, request arm position

#### H2: When NOT to Use Services
- Continuous data streams → Use topics
- Long-running operations → Use actions (Module 2+)
- Broadcast to multiple nodes → Use topics

#### H2: Topics vs. Services Comparison
| Aspect | Topics | Services |
|--------|--------|----------|
| Pattern | Pub/Sub | Request/Response |
| Timing | Asynchronous | Synchronous (blocking) |
| Cardinality | Many-to-many | One-to-one |
| Use Case | Continuous data | Infrequent operations |
| Examples | Sensor data, commands | State queries, config |

### Diagram Specifications
- **service-flow.mmd**: Sequence diagram showing client→service→server request/response

### Code Examples
- **service-call.py**: Service call to query robot state (12-18 lines)
  ```python
  import rclpy
  from rclpy.node import Node
  from example_interfaces.srv import AddTwoInts

  def main():
      rclpy.init()
      node = Node('service_client')
      client = node.create_client(AddTwoInts, 'add_two_ints')

      request = AddTwoInts.Request()
      request.a = 5
      request.b = 3

      future = client.call_async(request)
      rclpy.spin_until_future_complete(node, future)

      response = future.result()
      node.get_logger().info(f'Result: {response.sum}')

      node.destroy_node()
      rclpy.shutdown()
  ```

### Comprehension Questions
1. Provide two scenarios where services are more appropriate than topics.
2. What is the key difference between topics and services in terms of timing?
3. Why should you avoid using services for continuous data streams?

### Frontmatter Template
```yaml
---
title: "ROS 2 Services: Request/Response Communication"
description: "Understand ROS 2 services for synchronous, one-to-one operations"
keywords: [ros2, services, request, response, synchronous, rpc]
learning_outcome: "Contrast topics vs. services and identify appropriate use cases"
---
```

---

## Section 06: ROS 2 Messages

**File**: `docs/module-01-ros2-nervous-system/06-messages.md`
**Primary Concept**: Structured data packets with defined schemas
**Learning Outcome**: Reader understands message types and can identify common types
**Word Count Target**: 600-800 words
**Reading Time**: ~5-7 minutes

### Content Outline

#### H1: ROS 2 Messages

#### H2: What is a Message?
- Definition: Typed data structure for topics/services
- Schema defines fields and types
- Ensures interoperability between nodes

#### H2: Why Typed Messages?
- **Type Safety**: Catch errors at compile/runtime
- **Interoperability**: Nodes agree on data format
- **Documentation**: Schema documents expected data
- **Validation**: Automatic type checking

#### H2: Standard Message Types
- **std_msgs**: Basic types (String, Int32, Float64, Header)
  - Note: Deprecated for production, use for prototyping only
- **geometry_msgs**: Geometric primitives (Pose, Twist, Transform)
  - `Twist`: Linear and angular velocity
  - `Pose`: Position and orientation
- **sensor_msgs**: Sensor data (Image, Imu, LaserScan, JointState)

#### H2: Message Structure Example
- Breakdown of `geometry_msgs/Twist`
  - `linear.x, linear.y, linear.z`: Linear velocity (m/s)
  - `angular.x, angular.y, angular.z`: Angular velocity (rad/s)
- Diagram: Visual representation of Twist message fields

#### H2: Custom vs. Standard Messages
- Module 1 uses standard messages only
- Custom message creation covered in advanced modules
- Best practice: Use standard messages when possible

### Diagram Specifications
- **message-structure.mmd**: Diagram showing Twist message structure with fields

### Code Examples
- **message-twist.py**: Creating and publishing a Twist message (10-15 lines)
  ```python
  from geometry_msgs.msg import Twist

  # Create message
  msg = Twist()

  # Set linear velocity (move forward at 0.5 m/s)
  msg.linear.x = 0.5
  msg.linear.y = 0.0
  msg.linear.z = 0.0

  # Set angular velocity (turn at 0.1 rad/s)
  msg.angular.x = 0.0
  msg.angular.y = 0.0
  msg.angular.z = 0.1

  # Publish (assumes publisher already created)
  publisher.publish(msg)
  ```

### Comprehension Questions
1. Why does ROS 2 use typed messages rather than generic data?
2. What is the `geometry_msgs/Twist` message used for?
3. Name three standard message packages in ROS 2.

### Frontmatter Template
```yaml
---
title: "ROS 2 Messages: Typed Data Structures"
description: "Learn about ROS 2 message types and schemas for robot communication"
keywords: [ros2, messages, geometry_msgs, sensor_msgs, data types]
learning_outcome: "Understand message types and identify common standard messages"
---
```

---

## Section 07: Connecting AI Agents to ROS 2

**File**: `docs/module-01-ros2-nervous-system/07-ai-agent-integration.md`
**Primary Concept**: Bridging Python AI logic to ROS 2 via rclpy
**Learning Outcome**: Reader understands AI-ROS 2 integration point
**Word Count Target**: 600-800 words
**Reading Time**: ~5-7 minutes

### Content Outline

#### H1: Connecting AI Agents to ROS 2

#### H2: AI Agents as ROS 2 Nodes
- AI agent = Software entity making high-level decisions
- Integration: AI agent becomes a ROS 2 node
- Diagram: AI agent node in multi-node system

#### H2: The Integration Point
- AI logic: Decision-making, planning, learning
- ROS 2 interface: Publishing commands, subscribing to sensor data
- Separation of concerns: AI vs. communication

#### H2: rclpy as the Bridge
- rclpy = Python client library for ROS 2
- Allows Python AI code to interact with ROS 2 graph
- AI agent uses rclpy to create publishers/subscribers

#### H2: High-Level Integration Flow
1. AI agent initializes as ROS 2 node
2. AI subscribes to sensor topics (camera, IMU, etc.)
3. AI processes data and makes decisions
4. AI publishes commands to actuator topics
5. Robot executes commands

### Diagram Specifications
- **ai-agent-integration.mmd**: Architecture diagram showing AI agent node + robot control nodes

### Code Examples
- **ai-agent-skeleton.py**: Minimal AI agent node skeleton (12-16 lines)
  ```python
  import rclpy
  from rclpy.node import Node
  from geometry_msgs.msg import Twist

  class AIAgent(Node):
      def __init__(self):
          super().__init__('ai_agent')
          self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
          self.timer = self.create_timer(1.0, self.ai_decision_loop)

      def ai_decision_loop(self):
          # AI decision logic here (outside Module 1 scope)
          decision = self.make_decision()

          # Publish command via ROS 2
          cmd = Twist()
          cmd.linear.x = decision['speed']
          cmd.angular.z = decision['turn_rate']
          self.cmd_publisher.publish(cmd)

      def make_decision(self):
          # Placeholder for AI logic
          return {'speed': 0.5, 'turn_rate': 0.1}
  ```

### Comprehension Questions
1. How does an AI agent become part of a ROS 2 system?
2. What role does rclpy play in AI-ROS 2 integration?
3. What types of topics would an AI agent typically subscribe to?

### Frontmatter Template
```yaml
---
title: "Connecting AI Agents to ROS 2"
description: "Learn how Python AI agents integrate with ROS 2 robot systems"
keywords: [ros2, ai, rclpy, integration, python, agents]
learning_outcome: "Understand how AI agents connect to ROS 2 via rclpy"
---
```

---

## Section 08: rclpy Basics

**File**: `docs/module-01-ros2-nervous-system/08-rclpy-basics.md`
**Primary Concept**: Python client library fundamentals
**Learning Outcome**: Reader can write basic rclpy publisher/subscriber code
**Word Count Target**: 800-1000 words
**Reading Time**: ~7-9 minutes

### Content Outline

#### H1: rclpy Basics

#### H2: What is rclpy?
- Python client library for ROS 2
- Enables Python programs to interact with ROS 2 graph
- Core library for Python-based robot applications

#### H2: rclpy Initialization
- `rclpy.init()`: Initialize ROS communications
- Must be called before creating nodes
- Sets up underlying DDS infrastructure

#### H2: Creating Nodes
- `Node('node_name')`: Create node instance
- Nodes are the entry point to ROS 2 graph
- Multiple nodes can exist in one Python process

#### H2: Publishers and Subscribers
- **create_publisher()**: Create publisher for a topic
  - Parameters: message type, topic name, queue size
- **create_subscription()**: Create subscriber for a topic
  - Parameters: message type, topic name, callback function, queue size

#### H2: Spinning (Processing Callbacks)
- `rclpy.spin(node)`: Process incoming messages and timers
- Blocking call that runs until interrupted
- Enables callback functions to be invoked
- Alternative: `spin_once()` for single iteration

#### H2: Shutdown and Cleanup
- `node.destroy_node()`: Clean up node resources
- `rclpy.shutdown()`: Shutdown ROS communications
- Always call shutdown to prevent resource leaks

#### H2: rclpy Lifecycle Diagram
- Diagram: init → create_node → create_entities → spin → destroy → shutdown

### Diagram Specifications
- **rclpy-lifecycle.mmd**: Flowchart showing rclpy initialization → shutdown lifecycle

### Code Examples
- **publisher-complete.py**: Complete publisher example (18-20 lines)
  ```python
  import rclpy
  from rclpy.node import Node
  from std_msgs.msg import String

  class MinimalPublisher(Node):
      def __init__(self):
          super().__init__('minimal_publisher')
          self.publisher_ = self.create_publisher(String, 'topic', 10)
          self.timer = self.create_timer(0.5, self.timer_callback)
          self.i = 0

      def timer_callback(self):
          msg = String()
          msg.data = f'Hello World: {self.i}'
          self.publisher_.publish(msg)
          self.get_logger().info(f'Publishing: "{msg.data}"')
          self.i += 1

  def main(args=None):
      rclpy.init(args=args)
      minimal_publisher = MinimalPublisher()
      rclpy.spin(minimal_publisher)
      minimal_publisher.destroy_node()
      rclpy.shutdown()

  if __name__ == '__main__':
      main()
  ```

- **subscriber-complete.py**: Complete subscriber example (18-20 lines)
  ```python
  import rclpy
  from rclpy.node import Node
  from std_msgs.msg import String

  class MinimalSubscriber(Node):
      def __init__(self):
          super().__init__('minimal_subscriber')
          self.subscription = self.create_subscription(
              String,
              'topic',
              self.listener_callback,
              10)
          self.subscription  # prevent unused variable warning

      def listener_callback(self, msg):
          self.get_logger().info(f'I heard: "{msg.data}"')

  def main(args=None):
      rclpy.init(args=args)
      minimal_subscriber = MinimalSubscriber()
      rclpy.spin(minimal_subscriber)
      minimal_subscriber.destroy_node()
      rclpy.shutdown()

  if __name__ == '__main__':
      main()
  ```

### Comprehension Questions
1. What does `rclpy.spin()` do and why is it necessary?
2. What are the five main lifecycle steps when using rclpy?
3. Why must you call `rclpy.init()` before creating nodes?

### Frontmatter Template
```yaml
---
title: "rclpy Basics: Python Client Library"
description: "Learn fundamental rclpy patterns for creating ROS 2 nodes in Python"
keywords: [ros2, rclpy, python, publisher, subscriber, client library]
learning_outcome: "Write basic rclpy publisher and subscriber code"
---
```

---

## Section 09: Tracing Message Flow — AI to Actuation

**File**: `docs/module-01-ros2-nervous-system/09-message-flow.md`
**Primary Concept**: End-to-end flow from AI decision to physical robot action
**Learning Outcome**: Reader can trace AI → ROS 2 → robot flow
**Word Count Target**: 800-1000 words
**Reading Time**: ~7-9 minutes

### Content Outline

#### H1: Tracing Message Flow — AI to Actuation

#### H2: Concrete Example: "Raise Left Arm"
- Scenario: AI agent decides to raise humanoid's left arm
- Step-by-step message flow trace

#### H2: Step 1: AI Decision
- AI node processes sensor data
- Decides target action: raise left arm to 90 degrees
- Prepares command message

#### H2: Step 2: Publish Command
- AI node publishes `JointState` message to `/left_arm/command` topic
- Message contains joint positions for arm joints
- Code example: AI publishing arm command

#### H2: Step 3: Joint Controller Subscribes
- `joint_controller_node` subscribes to `/left_arm/command`
- Receives message, processes target positions
- Calculates motor commands

#### H2: Step 4: Actuator Execution
- Joint controller sends motor commands to actuators
- Physical motors move arm to target position
- Feedback sensors confirm movement

#### H2: Identifying Components
- **Nodes**: AI agent, joint controller
- **Topics**: `/left_arm/command`
- **Messages**: `sensor_msgs/JointState`
- **Flow**: AI → topic → controller → actuator

#### H2: Debugging Message Flow
- `ros2 topic echo /topic_name`: Monitor messages
- `ros2 node list`: See active nodes
- `ros2 topic list`: See all topics
- `ros2 topic info /topic_name`: Topic details

#### H2: Diagram: End-to-End Flow
- Visual trace of message from AI to robot

### Diagram Specifications
- **end-to-end-flow.mmd**: Sequence diagram showing AI → topic → controller → actuator

### Code Examples
- **ai-arm-command.py**: AI agent publishing arm command (15-18 lines)
  ```python
  import rclpy
  from rclpy.node import Node
  from sensor_msgs.msg import JointState

  class ArmCommandPublisher(Node):
      def __init__(self):
          super().__init__('arm_command_publisher')
          self.publisher = self.create_publisher(JointState, '/left_arm/command', 10)

      def publish_raise_arm_command(self):
          msg = JointState()
          msg.name = ['shoulder_pitch', 'shoulder_roll', 'elbow']
          msg.position = [1.57, 0.0, 1.57]  # 90 degrees in radians

          self.publisher.publish(msg)
          self.get_logger().info('Published: Raise left arm')

  def main():
      rclpy.init()
      node = ArmCommandPublisher()
      node.publish_raise_arm_command()
      rclpy.spin_once(node)
      node.destroy_node()
      rclpy.shutdown()
  ```

### Comprehension Questions
1. Trace the message flow for the command "walk forward" through nodes, topics, and messages.
2. What debugging command would you use to monitor messages on a specific topic?
3. Identify the nodes, topics, and message types in the "raise left arm" example.

### Frontmatter Template
```yaml
---
title: "Tracing Message Flow: AI to Actuation"
description: "Follow the complete path from AI decision to robot movement in ROS 2"
keywords: [ros2, message flow, debugging, ai, actuation, tracing]
learning_outcome: "Trace message flow from AI decision through ROS 2 to robot actuation"
---
```

---

## Section 10: URDF — Describing Robot Structure

**File**: `docs/module-01-ros2-nervous-system/10-urdf-intro.md`
**Primary Concept**: Formal description of robot physical structure
**Learning Outcome**: Reader can identify URDF elements (links, joints)
**Word Count Target**: 600-800 words
**Reading Time**: ~5-7 minutes

### Content Outline

#### H1: URDF — Describing Robot Structure

#### H2: Why Describe Robot Structure?
- Need formal description of robot's physical layout
- Used for: Kinematics, visualization, planning, simulation
- URDF provides standard format

#### H2: URDF Basics
- XML-based format
- Describes robot as tree of links and joints
- Used by ROS 2 tools (RViz, robot_state_publisher, MoveIt)

#### H2: Links: Rigid Bodies
- Definition: Physical components of robot
- Examples: Base, arm segments, gripper, sensors
- Properties: Visual, collision, inertial

#### H2: Joints: Connections
- Definition: How links move relative to each other
- Parent-child relationship

#### H2: Joint Types
- **Revolute**: Rotational with limits (elbow, shoulder)
- **Prismatic**: Linear sliding with limits (telescoping arm)
- **Fixed**: No movement (sensor mount)
- **Continuous**: Rotational without limits (wheel)

#### H2: Joint Limits
- **Position limits**: Min/max angle or distance
- **Velocity limits**: Max speed
- **Effort limits**: Max torque/force

#### H2: How ROS 2 Uses URDF
- `robot_state_publisher`: Publishes robot transforms
- Visualization in RViz
- Motion planning with MoveIt
- Note: Full URDF tutorial in advanced modules

### Diagram Specifications
- **urdf-arm-example.mmd**: Diagram showing links and joints in humanoid arm

### Code Examples
- **urdf-arm-snippet.xml**: Simple URDF snippet for humanoid arm (15-20 lines XML)
  ```xml
  <?xml version="1.0"?>
  <robot name="humanoid_arm">
    <!-- Base Link -->
    <link name="base_link"/>

    <!-- Upper Arm Link -->
    <link name="upper_arm_link"/>

    <!-- Shoulder Joint (Revolute) -->
    <joint name="shoulder_joint" type="revolute">
      <parent link="base_link"/>
      <child link="upper_arm_link"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
    </joint>

    <!-- Forearm Link -->
    <link name="forearm_link"/>

    <!-- Elbow Joint (Revolute) -->
    <joint name="elbow_joint" type="revolute">
      <parent link="upper_arm_link"/>
      <child link="forearm_link"/>
      <origin xyz="0.3 0 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="2.35" effort="100" velocity="1.0"/>
    </joint>
  </robot>
  ```

### Comprehension Questions
1. What are the two main elements in a URDF file?
2. What is the difference between a revolute and a prismatic joint?
3. What are joint limits and why are they important?

### Frontmatter Template
```yaml
---
title: "URDF: Describing Robot Structure"
description: "Introduction to URDF format for defining robot links and joints"
keywords: [ros2, urdf, robot description, links, joints, kinematics]
learning_outcome: "Identify URDF links and joints with 100% accuracy"
---
```

---

## Section 11: Module Summary and Next Steps

**File**: `docs/module-01-ros2-nervous-system/11-summary.md`
**Primary Concept**: Consolidation of Module 1 learning outcomes
**Learning Outcome**: Reader confirms understanding and knows module boundaries
**Word Count Target**: 600-800 words
**Reading Time**: ~5-7 minutes

### Content Outline

#### H1: Module Summary and Next Steps

#### H2: Module 1 Recap
- Key Concepts Covered:
  - ROS 2 as the "robotic nervous system"
  - Nodes: Independent computational processes
  - Topics: Asynchronous pub/sub communication
  - Services: Synchronous request/response
  - Messages: Typed data structures
  - rclpy: Python client library
  - Message flow: AI → ROS 2 → Robot
  - URDF: Robot structure description

#### H2: Key Takeaways
1. ROS 2 provides modular, distributed architecture for robot software
2. Nodes communicate via topics (streaming) and services (request/response)
3. AI agents integrate as ROS 2 nodes using rclpy
4. Message flow can be traced from decision to actuation
5. URDF describes robot's physical structure

#### H2: What's NOT in Module 1
- Installation and setup (assumes ROS 2 environment available)
- Hardware-specific code (vendor SDKs, drivers)
- Simulation environments (Gazebo, Isaac Sim)
- Advanced topics:
  - Navigation and path planning
  - Computer vision and perception
  - SLAM (Simultaneous Localization and Mapping)
  - Multi-robot coordination
  - Real-time control loops
  - Custom message creation

#### H2: Preview: Future Modules
- **Module 2**: Sensors and Perception (coming soon)
- **Module 3**: Navigation and Path Planning (coming soon)
- **Module 4**: Manipulation and Grasping (coming soon)
- **Module 5**: Multi-Robot Systems (coming soon)

#### H2: Self-Assessment Questions
1. Explain ROS 2's role in a physical AI system using the nervous system analogy.
2. Describe the difference between topics and services. When would you use each?
3. Trace the message flow for an AI agent commanding a robot to "turn right."
4. What is rclpy and how does it connect AI agents to ROS 2?
5. Identify the links and joints in a simple URDF description of a robotic arm.
6. Give three examples of nodes in a humanoid robot system.
7. Why does ROS 2 use typed messages instead of generic data?

### Diagram Specifications
- None (summary only)

### Code Examples
- None (summary only)

### Comprehension Questions
- See Self-Assessment Questions above (embedded in content)

### Frontmatter Template
```yaml
---
title: "Module Summary and Next Steps"
description: "Consolidate learning from Module 1 and preview future modules"
keywords: [ros2, summary, review, next steps, modules]
learning_outcome: "Confirm Module 1 understanding and identify scope boundaries"
---
```

---

## Module Landing Page

**File**: `docs/module-01-ros2-nervous-system/index.md`
**Purpose**: Entry point for Module 1, provides overview and navigation
**Word Count Target**: 300-400 words

### Content Outline

#### H1: Module 01: ROS 2 — The Robotic Nervous System

#### Module Overview
- Brief description of module purpose
- Learning outcomes
- Estimated reading time: 45-60 minutes
- Prerequisites: Python background, interest in robotics

#### What You'll Learn
- Bulleted list of key concepts (nodes, topics, services, messages, rclpy, URDF)

#### Module Structure
- Navigation to sections 01-11
- Brief description of each section

#### How to Use This Module
- Sequential reading recommended for beginners
- Sections can be accessed independently for reference
- Code examples are illustrative, not production-ready
- Comprehension questions at end of each section

### Frontmatter Template
```yaml
---
title: "Module 01: ROS 2 — The Robotic Nervous System"
description: "Introduction to ROS 2 concepts for physical AI and humanoid robotics"
keywords: [ros2, module, introduction, robotics, physical ai]
---
```

---

## Summary

**Total Sections**: 11 main sections + 1 landing page = 12 files
**Total Word Count**: ~7,200-10,000 words
**Total Reading Time**: ~50-70 minutes
**Total Code Examples**: 9 Python files + 1 XML file = 10 examples
**Total Diagrams**: 9 Mermaid diagrams

### Implementation Order (Recommended)
1. Create diagrams (T008-T016)
2. Create code examples (T017-T025)
3. Write sections in order (01 → 11)
4. Create landing page last

### Validation Checklist
- [ ] All sections include frontmatter metadata
- [ ] Word counts within target ranges
- [ ] All diagrams referenced in appropriate sections
- [ ] All code examples embedded in appropriate sections
- [ ] Comprehension questions in each section
- [ ] Citations for all ROS 2 factual claims
- [ ] No scope violations (installation, hardware, simulation, advanced topics)
