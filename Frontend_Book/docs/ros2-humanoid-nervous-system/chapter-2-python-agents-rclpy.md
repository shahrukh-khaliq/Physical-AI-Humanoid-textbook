---
sidebar_position: 2
title: "Python Agents with rclpy"
description: "Connecting Python AI agents to ROS 2 using rclpy for publishers, subscribers, and services"
---

# Python Agents with rclpy

## Learning Objectives

By the end of this chapter, you will be able to:
- Connect Python-based AI agents to ROS 2 using rclpy
- Implement publishers, subscribers, and services in Python
- Understand agent-to-controller interaction patterns
- Create practical examples of AI logic interfacing with robot controllers

## Connecting Python AI Agents to ROS 2

Python is a popular choice for AI development due to its rich ecosystem of machine learning libraries. The rclpy library enables Python AI agents to communicate with ROS 2, bridging the gap between AI logic and robot control systems.

### Installing and Setting Up rclpy

rclpy is typically installed as part of a ROS 2 installation. To use it in your Python projects:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash  # Replace 'humble' with your ROS 2 distribution

# Or in your Python script, ensure ROS 2 is properly sourced in your environment
```

### Basic rclpy Node Structure

Every Python agent that connects to ROS 2 must be structured as a node:

```python
import rclpy
from rclpy.node import Node

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent')
        self.get_logger().info('AI Agent initialized')

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AIAgentNode()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publishers: Sending Commands from AI Agents

Publishers allow AI agents to send commands and data to other nodes in the ROS 2 system.

### Basic Publisher Implementation

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist

class DecisionMakerNode(Node):
    def __init__(self):
        super().__init__('decision_maker')

        # Publisher for movement commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Publisher for robot status
        self.status_publisher = self.create_publisher(
            String,
            'ai_agent/status',
            10
        )

        # Timer to make decisions periodically
        self.timer = self.create_timer(0.1, self.make_decision)

        self.get_logger().info('Decision Maker Node initialized')

    def make_decision(self):
        # Example AI decision logic
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.5  # Move forward at 0.5 m/s
        cmd_msg.angular.z = 0.2  # Turn slightly right

        self.cmd_vel_publisher.publish(cmd_msg)

        status_msg = String()
        status_msg.data = 'Moving forward with slight turn'
        self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DecisionMakerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced Publisher Patterns

For humanoid robots, you often need to publish joint commands:

```python
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__('motion_planner')

        # Publisher for joint trajectory commands
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            'joint_trajectory_controller/joint_trajectory',
            10
        )

        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]

    def send_walk_trajectory(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Create trajectory points
        point = JointTrajectoryPoint()
        point.positions = [0.1, -0.2, 0.1, 0.1, -0.2, 0.1]  # Example positions
        point.velocities = [0.0] * 6  # Zero velocities
        point.time_from_start = Duration(sec=1, nanosec=0)

        trajectory_msg.points.append(point)

        self.joint_trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info('Published walk trajectory')

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerNode()

    # Send trajectory once for demonstration
    node.send_walk_trajectory()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Subscribers: Receiving Data for AI Processing

Subscribers allow AI agents to receive sensor data and other information from the robot system.

### Basic Subscriber Implementation

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Subscribe to laser scan data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Subscribe to odometry data
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # Store robot state
        self.robot_pose = None
        self.laser_data = None

    def scan_callback(self, msg):
        # Process laser scan data for obstacle detection
        self.laser_data = np.array(msg.ranges)

        # Detect obstacles within 1 meter
        min_distance = min(self.laser_data)
        if min_distance < 1.0:
            self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m')

        # Store for AI processing
        self.process_laser_data()

    def odom_callback(self, msg):
        # Extract robot pose
        self.robot_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': self.quaternion_to_yaw(msg.pose.pose.orientation)
        }

        self.get_logger().info(f'Robot pose: ({self.robot_pose["x"]:.2f}, {self.robot_pose["y"]:.2f})')

    def quaternion_to_yaw(self, orientation):
        # Convert quaternion to yaw angle
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.z + orientation.z * orientation.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def process_laser_data(self):
        # AI processing of laser data
        if self.laser_data is not None:
            # Example: Calculate safe navigation direction
            front_distances = self.laser_data[300:420]  # Front 120 degrees
            min_front_distance = min(front_distances) if len(front_distances) > 0 else float('inf')

            if min_front_distance < 0.5:
                self.get_logger().info('Turning to avoid obstacle')
            else:
                self.get_logger().info('Path is clear')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Image Processing with Subscribers

For humanoid robots with cameras:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Create CV bridge to convert ROS images to OpenCV
        self.cv_bridge = CvBridge()

        # Subscribe to camera feed
        self.image_subscription = self.create_subscription(
            Image,
            'camera_front/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Vision Node initialized')

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform AI-based image processing
            processed_image = self.process_image(cv_image)

            # Optionally publish processed image
            # self.publish_processed_image(processed_image)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_image(self, image):
        # Example: Object detection using OpenCV
        # In practice, you might use a deep learning model here

        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Detect edges using Canny
        edges = cv2.Canny(blurred, 50, 150)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contours on original image
        result = image.copy()
        cv2.drawContours(result, contours, -1, (0, 255, 0), 2)

        # AI decision based on contours
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > 1000:  # Threshold for significant object
                self.get_logger().info(f'Significant object detected with area: {area:.2f}')

        return result

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services: Synchronous AI-Controller Communication

Services provide synchronous communication for operations that require confirmation or return specific results.

### Service Client Implementation

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

from example_interfaces.srv import SetBool
from std_srvs.srv import Trigger

class AIAgentWithServices(Node):
    def __init__(self):
        super().__init__('ai_agent_with_services')

        # Create service clients
        self.emergency_stop_client = self.create_client(
            SetBool,
            'emergency_stop'
        )

        self.calibrate_sensors_client = self.create_client(
            Trigger,
            'calibrate_sensors'
        )

        # Wait for services to be available
        while not self.emergency_stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Emergency stop service not available, waiting again...')

        while not self.calibrate_sensors_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Sensor calibration service not available, waiting again...')

        # Timer to demonstrate service calls
        self.timer = self.create_timer(5.0, self.periodic_service_calls)

    def periodic_service_calls(self):
        # Example: Request sensor calibration
        future = self.calibrate_sensors_client.call_async(Trigger.Request())
        future.add_done_callback(self.calibration_callback)

    def calibration_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Sensors calibrated successfully')
            else:
                self.get_logger().warn(f'Calibration failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def request_emergency_stop(self):
        request = SetBool.Request()
        request.data = True  # Activate emergency stop

        future = self.emergency_stop_client.call_async(request)
        future.add_done_callback(self.emergency_stop_callback)

    def emergency_stop_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Emergency stop: {response.message}')
            else:
                self.get_logger().warn(f'Emergency stop failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Emergency stop service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AIAgentWithServices()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Agent-to-Controller Interaction Patterns

### State Machine Pattern

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum

class RobotState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    MANIPULATING = "manipulating"
    EMERGENCY_STOP = "emergency_stop"

class StateMachineAIAgent(Node):
    def __init__(self):
        super().__init__('state_machine_agent')

        self.current_state = RobotState.IDLE
        self.state_publisher = self.create_publisher(String, 'robot_state', 10)

        # Timer for state transitions
        self.state_timer = self.create_timer(0.1, self.state_machine)

        self.get_logger().info(f'Initial state: {self.current_state.value}')

    def state_machine(self):
        # Publish current state
        state_msg = String()
        state_msg.data = self.current_state.value
        self.state_publisher.publish(state_msg)

        # Example state transitions based on some condition
        if self.current_state == RobotState.IDLE:
            # Check if navigation goal is available
            if self.should_start_navigation():
                self.current_state = RobotState.NAVIGATING
                self.get_logger().info('Transitioning to NAVIGATING state')

        elif self.current_state == RobotState.NAVIGATING:
            # Check if navigation is complete
            if self.navigation_complete():
                self.current_state = RobotState.IDLE
                self.get_logger().info('Navigation complete, returning to IDLE')

        elif self.current_state == RobotState.EMERGENCY_STOP:
            # Wait for manual override or automatic recovery
            if self.emergency_resolved():
                self.current_state = RobotState.IDLE
                self.get_logger().info('Emergency resolved, returning to IDLE')

    def should_start_navigation(self):
        # Example condition - in practice this would check for goals
        import random
        return random.random() < 0.01  # 1% chance per cycle

    def navigation_complete(self):
        # Example condition
        import random
        return random.random() < 0.05  # 5% chance per cycle

    def emergency_resolved(self):
        # Example condition
        return True  # Simplified

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineAIAgent()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Behavior Tree Pattern

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class BehaviorTreeNode:
    def __init__(self, name):
        self.name = name
        self.children = []

    def add_child(self, child):
        self.children.append(child)

    def execute(self):
        # To be implemented by subclasses
        pass

class SelectorNode(BehaviorTreeNode):
    def execute(self):
        for child in self.children:
            if child.execute():
                return True
        return False

class SequenceNode(BehaviorTreeNode):
    def execute(self):
        for child in self.children:
            if not child.execute():
                return False
        return True

class CheckBatteryNode(BehaviorTreeNode):
    def __init__(self, node, name):
        super().__init__(name)
        self.node = node
        self.battery_level = 0.8  # Example battery level

    def execute(self):
        # Check if battery is above threshold
        if self.battery_level > 0.2:
            self.node.get_logger().info('Battery level is sufficient')
            return True
        else:
            self.node.get_logger().warn('Battery level is low')
            return False

class CheckObstaclesNode(BehaviorTreeNode):
    def __init__(self, node, name):
        super().__init__(name)
        self.node = node
        self.obstacles_detected = False

    def execute(self):
        # Check for obstacles
        if not self.obstacles_detected:
            self.node.get_logger().info('No obstacles detected')
            return True
        else:
            self.node.get_logger().warn('Obstacles detected')
            return False

class MoveToGoalNode(BehaviorTreeNode):
    def __init__(self, node, name):
        super().__init__(name)
        self.node = node

    def execute(self):
        # Execute movement to goal
        self.node.get_logger().info('Moving to goal')
        # In practice, this would send movement commands
        return True

class BehaviorTreeAIAgent(Node):
    def __init__(self):
        super().__init__('behavior_tree_agent')

        # Create behavior tree
        self.root = SelectorNode("root")

        # Emergency sequence: check battery, then move to charging station if needed
        emergency_sequence = SequenceNode("emergency_sequence")
        emergency_sequence.add_child(CheckBatteryNode(self, "check_battery"))

        # Normal sequence: check for obstacles, then move to goal
        normal_sequence = SequenceNode("normal_sequence")
        normal_sequence.add_child(CheckObstaclesNode(self, "check_obstacles"))
        normal_sequence.add_child(MoveToGoalNode(self, "move_to_goal"))

        # Add sequences to root selector
        self.root.add_child(emergency_sequence)  # Will execute if battery is low
        self.root.add_child(normal_sequence)    # Will execute otherwise

        # Timer to run behavior tree
        self.tree_timer = self.create_timer(0.5, self.run_behavior_tree)

        self.get_logger().info('Behavior Tree Agent initialized')

    def run_behavior_tree(self):
        self.get_logger().info('Running behavior tree...')
        success = self.root.execute()
        if success:
            self.get_logger().info('Behavior tree execution successful')
        else:
            self.get_logger().info('Behavior tree execution failed')

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeAIAgent()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Example: Complete AI Agent

Here's a complete example combining publishers, subscribers, and services:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

class CompleteAIAgent(Node):
    def __init__(self):
        super().__init__('complete_ai_agent')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, 'ai_agent/status', 10)

        # Subscribers
        self.scan_subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

        # Services
        self.pause_service = self.create_service(
            Trigger, 'ai_agent/pause', self.pause_callback
        )
        self.resume_service = self.create_service(
            Trigger, 'ai_agent/resume', self.resume_callback
        )

        # Internal state
        self.is_paused = False
        self.obstacle_distance = float('inf')

        # Timer for AI decision making
        self.ai_timer = self.create_timer(0.1, self.ai_decision_loop)

        self.get_logger().info('Complete AI Agent initialized')

    def scan_callback(self, msg):
        # Process laser scan to detect obstacles
        if len(msg.ranges) > 0:
            # Consider front-facing range (simplified)
            front_ranges = msg.ranges[len(msg.ranges)//2-30:len(msg.ranges)//2+30]
            self.obstacle_distance = min([r for r in front_ranges if r > 0], default=float('inf'))

    def ai_decision_loop(self):
        if self.is_paused:
            # Publish zero velocity when paused
            cmd_msg = Twist()
            self.cmd_vel_publisher.publish(cmd_msg)
            return

        # Simple navigation AI
        cmd_msg = Twist()

        if self.obstacle_distance < 0.5:  # Obstacle within 0.5m
            # Stop and rotate to find clear path
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.5  # Rotate in place
            status = f'Obstacle detected at {self.obstacle_distance:.2f}m, rotating'
        else:
            # Move forward
            cmd_msg.linear.x = 0.3  # 0.3 m/s forward
            cmd_msg.angular.z = 0.0
            status = f'Moving forward, obstacle distance: {self.obstacle_distance:.2f}m'

        # Publish commands and status
        self.cmd_vel_publisher.publish(cmd_msg)
        status_msg = String()
        status_msg.data = status
        self.status_publisher.publish(status_msg)

    def pause_callback(self, request, response):
        self.is_paused = True
        response.success = True
        response.message = 'AI agent paused'
        self.get_logger().info('AI agent paused')
        return response

    def resume_callback(self, request, response):
        self.is_paused = False
        response.success = True
        response.message = 'AI agent resumed'
        self.get_logger().info('AI agent resumed')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CompleteAIAgent()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

In this chapter, you've learned how to connect Python AI agents to ROS 2:

1. **rclpy basics**: Setting up Python nodes that connect to ROS 2
2. **Publishers**: Sending commands and data from AI agents
3. **Subscribers**: Receiving sensor data for AI processing
4. **Services**: Implementing synchronous communication patterns
5. **Interaction patterns**: State machines and behavior trees for AI logic

These concepts enable AI agents to effectively interface with robot controllers and form the bridge between high-level AI decision-making and low-level robot control.

## Next Steps

In the next chapter, we'll explore URDF (Unified Robot Description Format) and how to interpret humanoid robot descriptions, which is essential for controlling and simulating humanoid robots.