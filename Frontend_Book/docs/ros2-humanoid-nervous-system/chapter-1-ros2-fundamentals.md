---
sidebar_position: 1
title: "ROS 2 Fundamentals for Humanoid Robotics"
description: "Understanding ROS 2 as the robotic nervous system for humanoid robots"
---

# ROS 2 Fundamentals for Humanoid Robotics

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain how ROS 2 serves as a robotic nervous system for humanoid robots
- Identify the core components of ROS 2 architecture (nodes, topics, services, actions)
- Understand the high-level ROS 2 communication model
- Recognize how different robot components communicate using ROS 2

## ROS 2 as a Robotic Nervous System

ROS 2 (Robot Operating System 2) serves as the communication backbone for humanoid robots, much like a nervous system in biological organisms. Just as the nervous system enables different parts of the body to communicate and coordinate actions, ROS 2 enables different components of a humanoid robot to exchange information and work together.

### Key Characteristics of ROS 2 as a Nervous System

- **Distributed**: Components can run on different computers or processors
- **Real-time**: Supports time-critical communication between robot components
- **Modular**: Components can be developed and tested independently
- **Scalable**: Can handle simple robots to complex humanoid systems with many components

## Core Architecture Components

### Nodes

Nodes are the fundamental building blocks of ROS 2. Each node represents a process that performs a specific function within the robot system.

```python
# Example of a simple ROS 2 node
import rclpy
from rclpy.node import Node

class JointControllerNode(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.get_logger().info('Joint Controller Node Started')

def main(args=None):
    rclpy.init(args=args)
    node = JointControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topics and Message Passing

Topics enable asynchronous communication between nodes using a publish-subscribe pattern. This is ideal for continuous data streams like sensor readings or motor commands.

```python
# Publisher example
import rclpy
from std_msgs.msg import Float64MultiArray

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, 'joint_positions', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [1.0, 2.0, 3.0]  # Example joint positions
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
```

```python
# Subscriber example
import rclpy
from std_msgs.msg import Float64MultiArray

class MotorSubscriber(Node):
    def __init__(self):
        super().__init__('motor_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'joint_positions',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint positions: {msg.data}')
```

### Services

Services provide synchronous request-response communication, suitable for operations that require confirmation or return specific results.

```python
# Service server example
from example_interfaces.srv import AddTwoInts

class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__('add_two_ints_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response
```

```python
# Service client example
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

### Actions

Actions provide goal-oriented communication with feedback, perfect for long-running tasks like navigation or manipulation.

```python
# Action server example
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_controllers.action import MoveToPose  # Custom action definition

class MoveToPoseActionServer(Node):
    def __init__(self):
        super().__init__('move_to_pose_action_server')
        self._action_server = ActionServer(
            self,
            MoveToPose,
            'move_to_pose',
            self.execute_callback)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Simulate moving to pose
        feedback_msg = MoveToPose.Feedback()
        for i in range(0, 100, 5):
            feedback_msg.current_pose = i  # Simplified example
            goal_handle.publish_feedback(feedback_msg)

        result = MoveToPose.Result()
        result.success = True
        return result
```

## High-Level ROS 2 Architecture

### Communication Layer

ROS 2 uses Data Distribution Service (DDS) as its communication middleware, providing:

- **Discovery**: Automatic discovery of nodes on the network
- **Reliability**: Configurable quality of service settings
- **Security**: Built-in security features for safe robot operation
- **Real-time**: Support for time-critical applications

### Execution Layer

The execution layer manages how nodes are organized and run:

- **Single-threaded executor**: Processes callbacks sequentially
- **Multi-threaded executor**: Processes callbacks in parallel
- **Static single-threaded executor**: Optimized for single-threaded execution

### Client Libraries

ROS 2 provides client libraries for multiple languages:

- **rclcpp**: C++ client library
- **rclpy**: Python client library (primary focus for AI agents)
- **rcl**: Common client library infrastructure

## Practical Example: Humanoid Robot Communication

Let's consider a simple humanoid robot with the following components:
- Joint controllers
- Sensor systems (IMU, cameras, force sensors)
- High-level motion planners
- AI decision-making modules

```python
# Example: Head tracking node that subscribes to camera data and controls neck joints
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np

class HeadTrackerNode(Node):
    def __init__(self):
        super().__init__('head_tracker')

        # Subscribe to camera feed
        self.subscription = self.create_subscription(
            Image,
            'camera_front/image_raw',
            self.image_callback,
            10)

        # Publisher for neck joint commands
        self.joint_publisher = self.create_publisher(
            Float64MultiArray,
            'neck_controller/commands',
            10)

        # Timer for processing
        self.timer = self.create_timer(0.05, self.process_callback)  # 20 Hz

        self.target_detected = False
        self.target_position = [0.0, 0.0]

    def image_callback(self, msg):
        # Process image to detect target
        # Simplified example - in practice would use CV algorithms
        self.target_detected = True
        self.target_position = [0.5, 0.5]  # Example normalized coordinates

    def process_callback(self):
        if self.target_detected:
            # Calculate neck joint angles to look at target
            neck_commands = Float64MultiArray()
            neck_commands.data = [
                self.target_position[0] * 0.5,  # Pan
                self.target_position[1] * 0.3   # Tilt
            ]
            self.joint_publisher.publish(neck_commands)

def main(args=None):
    rclpy.init(args=args)
    node = HeadTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

In this chapter, you've learned about the fundamental components of ROS 2 that make it function as a robotic nervous system:

1. **Nodes** as the basic computational units
2. **Topics** for asynchronous communication
3. **Services** for synchronous request-response
4. **Actions** for goal-oriented tasks
5. The high-level architecture that enables complex humanoid robot coordination

These concepts form the foundation for connecting AI agents to ROS 2, which we'll explore in the next chapter.

## Next Steps

In the next chapter, we'll explore how Python-based AI agents can connect to ROS 2 using the rclpy library, enabling AI logic to interface with robot controllers.