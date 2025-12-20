---
sidebar_position: 3
title: "Humanoid Description with URDF"
description: "Understanding URDF for humanoid robots including links, joints, frames, and kinematics"
---

# Humanoid Description with URDF

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the purpose and structure of URDF files
- Identify links, joints, and frames in humanoid robot descriptions
- Explain kinematics concepts in URDF
- Understand the role of URDF in robot control and simulation

## Purpose of URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. For humanoid robots, URDF serves as the digital blueprint that defines the robot's physical structure, including:

- **Physical properties**: Mass, inertia, visual appearance
- **Kinematic structure**: How different parts are connected
- **Geometric relationships**: Positions and orientations of components
- **Collision properties**: Shapes for physics simulation

### Why URDF is Critical for Humanoid Robots

Humanoid robots are complex systems with many degrees of freedom. URDF provides:

1. **Standardization**: A common format for robot description across ROS ecosystem
2. **Simulation compatibility**: Enables physics simulation in Gazebo and other simulators
3. **Visualization**: Tools like RViz can visualize the robot based on URDF
4. **Kinematic computation**: Robot state publishers can compute forward kinematics
5. **Controller integration**: Joint controllers can understand robot structure

## URDF Structure and Components

### Basic URDF Document Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_head" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.7" rpy="0 0 0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>
</robot>
```

### Links: The Building Blocks

Links represent rigid bodies in the robot. Each link can have:

- **Visual properties**: How the link appears in simulation/visualization
- **Collision properties**: Shapes used for collision detection
- **Inertial properties**: Mass, center of mass, and inertia tensor for physics simulation

#### Link Components

```xml
<link name="example_link">
  <!-- Visual representation -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Can be box, cylinder, sphere, or mesh -->
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>

  <!-- Collision geometry -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>

  <!-- Physical properties for simulation -->
  <inertial>
    <mass value="0.5"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

### Joints: Connecting the Structure

Joints define how links are connected and how they can move relative to each other.

#### Joint Types

```xml
<!-- Fixed joint (no movement) -->
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<!-- Revolute joint (single axis rotation) -->
<joint name="revolute_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <!-- Rotation axis -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>

<!-- Continuous joint (unlimited rotation) -->
<joint name="continuous_joint" type="continuous">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>

<!-- Prismatic joint (linear motion) -->
<joint name="prismatic_joint" type="prismatic">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.1" effort="100" velocity="1"/>
</joint>

<!-- Floating joint (6 DOF) -->
<joint name="floating_joint" type="floating">
  <parent link="parent_link"/>
  <child link="child_link"/>
</joint>
```

### Materials and Colors

Materials define the visual appearance of links:

```xml
<material name="blue">
  <color rgba="0 0 1 1"/>
</material>

<material name="green">
  <color rgba="0 1 0 1"/>
</material>

<material name="red">
  <color rgba="1 0 0 1"/>
</material>
```

## Links, Joints, and Frames in Humanoid Robots

### Humanoid Robot Structure Example

Let's examine a simplified humanoid robot structure:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.006" ixy="0" ixz="0" iyy="0.006" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="15" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.6"/>
      <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
</robot>
```

### Frame Conventions

In URDF, each link has its own coordinate frame:

- **Origin**: Defines the position and orientation of a joint or link relative to its parent
- **Axes**: Standard ROS convention uses right-handed coordinate system (X forward, Y left, Z up)
- **Joint axes**: Define the direction of motion for revolute and prismatic joints

## Kinematics in URDF

### Forward Kinematics

Forward kinematics computes the position and orientation of end-effectors given joint angles. The URDF structure provides the kinematic chain needed for these calculations.

```python
# Example using Python with ROS 2 and TF2 for forward kinematics
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')

        # Transform broadcaster to publish link poses
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to update transforms (in a real system, this would use robot state publisher)
        self.timer = self.create_timer(0.1, self.publish_transforms)

    def publish_transforms(self):
        # Example: Publish transforms for a simple arm
        transforms = []

        # Base to shoulder
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'shoulder'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.5
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        transforms.append(t)

        # Shoulder to elbow
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'shoulder'
        t.child_frame_id = 'elbow'
        t.transform.translation.x = 0.3  # Arm length
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        transforms.append(t)

        # Publish all transforms
        for transform in transforms:
            self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()

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

### Inverse Kinematics

While URDF defines the forward kinematic structure, inverse kinematics (finding joint angles for desired end-effector positions) typically requires additional libraries:

```python
# Example of conceptual inverse kinematics setup
import numpy as np

class InverseKinematics:
    def __init__(self):
        # Robot parameters (from URDF)
        self.upper_arm_length = 0.3
        self.lower_arm_length = 0.25

    def two_link_arm_ik(self, x, y):
        """
        Solve inverse kinematics for a 2-link planar arm
        """
        # Distance from shoulder to target
        r = np.sqrt(x**2 + y**2)

        # Check if target is reachable
        if r > (self.upper_arm_length + self.lower_arm_length):
            # Target is out of reach
            scale = (self.upper_arm_length + self.lower_arm_length) / r
            x *= scale
            y *= scale
            r = np.sqrt(x**2 + y**2)
        elif r < abs(self.upper_arm_length - self.lower_arm_length):
            # Target is too close
            return None  # Cannot reach

        # Calculate elbow angle using law of cosines
        cos_elbow = (self.upper_arm_length**2 + self.lower_arm_length**2 - r**2) / \
                    (2 * self.upper_arm_length * self.lower_arm_length)
        elbow_angle = np.arccos(np.clip(cos_elbow, -1, 1))

        # Calculate shoulder angle
        alpha = np.arctan2(y, x)
        beta = np.arccos((self.upper_arm_length**2 + r**2 - self.lower_arm_length**2) / \
                        (2 * self.upper_arm_length * r))

        shoulder_angle = alpha - beta

        # Alternative solution (elbow down)
        # shoulder_angle_alt = alpha + beta
        # elbow_angle_alt = -elbow_angle

        return shoulder_angle, elbow_angle

# Usage example
ik_solver = InverseKinematics()
joint_angles = ik_solver.two_link_arm_ik(0.4, 0.2)  # Target at x=0.4, y=0.2
if joint_angles:
    shoulder, elbow = joint_angles
    print(f"Shoulder: {shoulder:.3f} rad, Elbow: {elbow:.3f} rad")
else:
    print("Target not reachable")
```

## Role of URDF in Control and Simulation

### Robot State Publisher

The robot_state_publisher node uses URDF to compute and publish transforms between robot links based on joint states:

```python
# Example of how robot state publisher works conceptually
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class RobotStatePublisherNode(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Store joint positions
        self.joint_positions = {}

    def joint_state_callback(self, msg):
        # Update joint positions
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = position

        # Publish transforms based on current joint positions
        self.publish_transforms()

    def publish_transforms(self):
        # Example: Publish transform for a revolute joint
        if 'shoulder_joint' in self.joint_positions:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'torso'
            t.child_frame_id = 'upper_arm'

            # Convert joint angle to quaternion
            angle = self.joint_positions['shoulder_joint']
            t.transform.translation.x = 0.2  # Offset from parent
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.1
            # Rotate around Y axis
            cy = math.cos(angle * 0.5)
            sy = math.sin(angle * 0.5)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = sy
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = cy

            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisherNode()

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

### Integration with Controllers

URDF is essential for joint controllers to understand the robot structure:

```python
# Example controller that uses URDF information
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Publisher for joint trajectories
        self.joint_traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Timer to send control commands
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # Joint names from URDF (these would typically be loaded from parameter server)
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]

        self.get_logger().info('Humanoid controller initialized')

    def control_loop(self):
        # Create a simple walking gait pattern
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Create trajectory point
        point = JointTrajectoryPoint()

        # Example joint positions for walking stance
        point.positions = [
            0.1,   # left_hip_joint
            -0.2,  # left_knee_joint
            0.1,   # left_ankle_joint
            0.1,   # right_hip_joint
            -0.2,  # right_knee_joint
            0.1    # right_ankle_joint
        ]

        # Set velocities to zero
        point.velocities = [0.0] * len(self.joint_names)

        # Set acceleration
        point.accelerations = [0.0] * len(self.joint_names)

        # Set time from start (50ms for this point)
        point.time_from_start = Duration(sec=0, nanosec=50000000)

        trajectory_msg.points.append(point)

        # Publish the trajectory
        self.joint_traj_pub.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidController()

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

## Practical URDF Example: Complete Humanoid

Here's a more complete example of a humanoid robot URDF that demonstrates all the concepts:

```xml
<?xml version="1.0"?>
<robot name="complete_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Include gazebo plugins -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/</robotNamespace>
    </plugin>
  </gazebo>

  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.42352941176 0.03921568627 1.0"/>
  </material>
  <material name="brown">
    <color rgba="0.87058823529 0.81176470588 0.76470588235 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.2"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.4"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.4"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <inertia ixx="0.3" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.2 0.6"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.2 0.6"/>
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="5" velocity="1"/>
  </joint>

  <link name="head">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.006" ixy="0.0" ixz="0.0" iyy="0.006" iyz="0.0" izz="0.0002"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="15" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <inertial>
      <mass value="0.6"/>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.0001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
  </link>

  <!-- Right Arm (symmetric to left) -->
  <joint name="right_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <link name="right_upper_arm">
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.006" ixy="0.0" ixz="0.0" iyy="0.006" iyz="0.0" izz="0.0002"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="15" velocity="1"/>
  </joint>

  <link name="right_lower_arm">
    <inertial>
      <mass value="0.6"/>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.0001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Leg -->
  <joint name="left_hip" type="revolute">
    <parent link="base_link"/>
    <child link="left_thigh"/>
    <origin xyz="0.08 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="30" velocity="1"/>
  </joint>

  <link name="left_thigh">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_knee" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="30" velocity="1"/>
  </joint>

  <link name="left_shin">
    <inertial>
      <mass value="1.2"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_ankle" type="revolute">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="1"/>
  </joint>

  <link name="left_foot">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
    <visual>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.1 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.1 0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Right Leg (symmetric to left) -->
  <joint name="right_hip" type="revolute">
    <parent link="base_link"/>
    <child link="right_thigh"/>
    <origin xyz="-0.08 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="30" velocity="1"/>
  </joint>

  <link name="right_thigh">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_knee" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="30" velocity="1"/>
  </joint>

  <link name="right_shin">
    <inertial>
      <mass value="1.2"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_ankle" type="revolute">
    <parent link="right_shin"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="1"/>
  </joint>

  <link name="right_foot">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
    <visual>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.1 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.1 0.05"/>
      </geometry>
    </collision>
  </link>
</robot>
```

## Working with URDF Files

### Validating URDF

You can validate your URDF files using ROS tools:

```bash
# Check URDF syntax
check_urdf /path/to/your/robot.urdf

# Display URDF information
urdf_to_graphiz /path/to/your/robot.urdf
```

### Loading URDF in Python

```python
# Example of loading and working with URDF in Python
import xml.etree.ElementTree as ET

def parse_urdf(urdf_path):
    """Parse URDF file and extract information"""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    robot_name = root.get('name')
    print(f"Robot name: {robot_name}")

    # Extract links
    links = root.findall('link')
    print(f"Number of links: {len(links)}")

    for link in links:
        link_name = link.get('name')
        print(f"  - Link: {link_name}")

        # Check for visual, collision, and inertial elements
        if link.find('visual') is not None:
            print(f"    Visual: Present")
        if link.find('collision') is not None:
            print(f"    Collision: Present")
        if link.find('inertial') is not None:
            print(f"    Inertial: Present")

    # Extract joints
    joints = root.findall('joint')
    print(f"Number of joints: {len(joints)}")

    for joint in joints:
        joint_name = joint.get('name')
        joint_type = joint.get('type')
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        print(f"  - Joint: {joint_name} ({joint_type}) from {parent} to {child}")

# Usage
# parse_urdf('path/to/humanoid.urdf')
```

## Summary

In this chapter, you've learned about URDF (Unified Robot Description Format) and its critical role in humanoid robotics:

1. **URDF purpose**: Digital blueprints that define robot physical structure
2. **Links and joints**: The fundamental components that make up robot structure
3. **Frames and kinematics**: How URDF enables spatial relationships and movement calculations
4. **Control and simulation**: How URDF integrates with controllers and simulators

URDF is essential for humanoid robots as it provides the foundation for visualization, simulation, control, and understanding of the robot's physical structure.

## Next Steps

With all three chapters complete, your understanding of ROS 2 for humanoid robotics is now comprehensive:
- ROS 2 fundamentals as a robotic nervous system
- Connecting Python AI agents using rclpy
- Understanding and working with humanoid robot descriptions in URDF

This knowledge provides a solid foundation for developing and controlling humanoid robots using ROS 2.