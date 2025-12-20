---
sidebar_position: 4
title: "Nav2 for Humanoid Robot Navigation"
description: "Configure path planning concepts and navigation stacks for bipedal robots using Nav2"
---

# Nav2 for Humanoid Robot Navigation

## Learning Objectives

By the end of this chapter, you will be able to:
- Adapt Nav2 for humanoid robot navigation with specialized path planning
- Configure costmap parameters for bipedal locomotion requirements
- Implement dynamic obstacle avoidance for walking robots
- Address terrain considerations specific to humanoid navigation
- Integrate humanoid-specific navigation behaviors with Nav2

## Introduction to Nav2 for Humanoid Robots

Navigation2 (Nav2) is the standard navigation framework for ROS 2, providing a comprehensive set of tools for robot path planning, obstacle avoidance, and autonomous navigation. When applied to humanoid robots, Nav2 requires specialized configuration to account for the unique challenges of bipedal locomotion.

### Key Challenges for Humanoid Navigation

Humanoid robot navigation presents unique challenges compared to wheeled robots:

- **Stability Requirements**: Bipedal locomotion requires maintaining balance during movement
- **Terrain Constraints**: Humanoid robots have specific requirements for traversable terrain
- **Dynamic Obstacle Interaction**: Humanoid robots may need to navigate around other humans
- **Footstep Planning**: Path planning must account for foot placement and balance
- **Multi-Modal Navigation**: Potential transitions between walking, climbing stairs, etc.

## Path Planning Concepts for Humanoid Robots

Path planning for humanoid robots must account for the physical constraints of bipedal locomotion and the need for stable, safe navigation.

### Humanoid-Specific Path Planning Considerations

Unlike wheeled robots, humanoid robots have specific requirements for path planning:

- **Stability**: Paths must maintain the robot's center of mass within stable regions
- **Footstep Planning**: The path must be executable with discrete foot placements
- **Terrain Analysis**: Different terrain types require different walking strategies
- **Dynamic Balance**: Continuous balance maintenance during locomotion
- **Step Height**: Limitations on step height and ground clearance

### Nav2 Planner Configuration for Humanoids

```python
# Example: Configuring Nav2 for humanoid-specific path planning
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

class HumanoidNav2Client(Node):
    def __init__(self):
        super().__init__('humanoid_nav2_client')

        # Create action client for navigation
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # Humanoid-specific navigation parameters
        self.step_height_limit = 0.15  # meters
        self.stability_margin = 0.3    # meters from obstacles
        self.max_slope_angle = 15.0    # degrees

    def send_navigate_goal(self, x, y, theta):
        """Send navigation goal with humanoid-specific constraints"""
        goal_msg = NavigateToPose.Goal()

        # Set the target pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Set orientation
        from math import sin, cos
        from math import pi
        quat = self.euler_to_quaternion(0, 0, theta)
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]

        # Send the goal
        self.nav_to_pose_client.wait_for_server()
        return self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Current navigation progress: {feedback.current_pose}'
        )

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        import math
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [x, y, z, w]

def main(args=None):
    rclpy.init(args=args)
    client = HumanoidNav2Client()

    # Example: Navigate to a position
    future = client.send_navigate_goal(5.0, 5.0, 0.0)

    rclpy.spin_until_future_complete(client, future)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Nav2 Adaptation for Bipedal Navigation

Adapting Nav2 for bipedal navigation requires modifications to the standard navigation stack to account for humanoid-specific constraints.

### Humanoid Navigation Stack Components

The humanoid navigation stack includes:

1. **Footstep Planner**: Plans discrete foot placements along the path
2. **Balance Controller**: Maintains stability during navigation
3. **Terrain Analyzer**: Assesses terrain traversability for bipedal locomotion
4. **Step Height Limiter**: Ensures path respects step height constraints
5. **Stability Checker**: Verifies path stability for bipedal movement

### Configuration Parameters

```yaml
# Example: Nav2 configuration for humanoid robots (typically in nav2_params.yaml)
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.2
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: "map"
    robot_base_frame: "base_footprint"
    odom_topic: "odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    # Humanoid-specific behavior tree
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
```

## Costmap Configurations for Bipedal Robots

Costmaps in Nav2 need to be configured specifically for humanoid robots to account for their unique locomotion requirements.

### Humanoid-Specific Costmap Parameters

```yaml
# Costmap configuration for humanoid robots
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: map
      robot_base_frame: base_footprint
      use_sim_time: False
      rolling_window: false
      width: 20
      height: 20
      resolution: 0.05  # Higher resolution for precise footstep planning
      origin_x: -10.0
      origin_y: -10.0

      # Humanoid-specific inflation
      inflation_radius: 0.5  # Account for robot's balance requirements
      cost_scaling_factor: 5.0

      # Layer configurations
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: "map"
        transform_tolerance: 0.5
        max_publish_frequency: 0.5

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 10.0
          raytrace_min_range: 0.0
          obstacle_max_range: 5.0
          obstacle_min_range: 0.0
          footprint_clearing_enabled: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: True
        cost_scaling_factor: 5.0
        inflation_radius: 0.5  # Wider safety margin for bipedal stability

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_footprint
      use_sim_time: False
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.025  # Even higher resolution for local planning
      origin_x: -3.0
      origin_y: -3.0

      # Humanoid-specific local costmap
      inflation_radius: 0.3  # Closer to robot for detailed planning
      cost_scaling_factor: 8.0  # Higher cost scaling for safety

      plugins: ["obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 3.0
          obstacle_min_range: 0.0
          footprint_clearing_enabled: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: True
        cost_scaling_factor: 8.0
        inflation_radius: 0.3
```

## Dynamic Obstacle Avoidance for Walking Robots

Humanoid robots require specialized dynamic obstacle avoidance that considers both their bipedal nature and the fact that they often navigate in human-populated environments.

### Humanoid Collision Avoidance Strategies

1. **Predictive Avoidance**: Anticipate human movement patterns
2. **Social Navigation**: Follow human social conventions (right-side passing, etc.)
3. **Stability Preservation**: Avoid maneuvers that could compromise balance
4. **Multi-Step Planning**: Plan several steps ahead for smooth avoidance

### Example Dynamic Avoidance Implementation

```python
# Example: Dynamic obstacle avoidance for humanoid robots
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class HumanoidDynamicAvoidance(Node):
    def __init__(self):
        super().__init__('humanoid_dynamic_avoidance')

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Humanoid-specific parameters
        self.min_distance = 0.8  # Minimum safe distance (wider for stability)
        self.turn_threshold = 1.0  # Distance to start turning
        self.stability_factor = 0.7  # Factor to maintain balance

        self.current_velocity = Twist()

    def scan_callback(self, msg):
        """Process laser scan data for obstacle detection"""
        # Find closest obstacle in front of robot
        front_range = msg.ranges[len(msg.ranges)//2]  # Front reading

        # Check for obstacles in humanoid's path
        min_distance = min(msg.ranges)
        min_index = msg.ranges.index(min_distance)

        # Calculate angle of closest obstacle
        angle_increment = msg.angle_increment
        obstacle_angle = msg.angle_min + min_index * angle_increment

        # Determine avoidance action
        cmd_vel = Twist()

        if min_distance < self.min_distance:
            # Emergency stop or avoidance maneuver
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = self.calculate_avoidance_turn(obstacle_angle)
        elif min_distance < self.turn_threshold:
            # Gentle avoidance turn
            cmd_vel.linear.x = 0.2  # Slow down
            cmd_vel.angular.z = self.calculate_avoidance_turn(obstacle_angle) * 0.5
        else:
            # Safe to proceed
            cmd_vel.linear.x = 0.5  # Normal walking speed
            cmd_vel.angular.z = 0.0

        # Apply stability constraints
        cmd_vel.linear.x *= self.stability_factor
        cmd_vel.angular.z *= self.stability_factor

        self.cmd_vel_pub.publish(cmd_vel)

    def calculate_avoidance_turn(self, obstacle_angle):
        """Calculate appropriate turning direction to avoid obstacle"""
        # Turn away from obstacle
        if obstacle_angle > 0:
            return -0.5  # Turn left to avoid right-side obstacle
        else:
            return 0.5   # Turn right to avoid left-side obstacle

    def odom_callback(self, msg):
        """Update current velocity for dynamic calculations"""
        self.current_velocity = msg.twist.twist

def main(args=None):
    rclpy.init(args=args)
    avoidance_node = HumanoidDynamicAvoidance()
    rclpy.spin(avoidance_node)
    avoidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Terrain Considerations for Bipedal Locomotion

Humanoid robots have specific requirements for terrain traversability that differ significantly from wheeled robots.

### Terrain Analysis for Humanoid Navigation

Key terrain factors for humanoid robots:

- **Step Height**: Maximum height difference between consecutive steps
- **Surface Stability**: Firmness of ground for stable foot placement
- **Slope Angle**: Maximum incline/decline angles the robot can handle
- **Surface Roughness**: Impact on foot placement and balance
- **Obstacle Size**: Objects that require stepping over vs. going around

### Implementing Terrain-Aware Navigation

```python
# Example: Terrain-aware navigation for humanoid robots
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
import numpy as np

class TerrainAnalyzer(Node):
    def __init__(self):
        super().__init__('terrain_analyzer')

        # Subscriber for point cloud data (from depth camera or LIDAR)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            10
        )

        # Parameters for humanoid terrain analysis
        self.step_height_threshold = 0.15  # Max step height (0.15m)
        self.slope_threshold = 20.0        # Max slope in degrees
        self.traversable_height_min = 0.05 # Min height for obstacles
        self.traversable_height_max = 0.8  # Max height for obstacles

        self.terrain_map = None

    def pointcloud_callback(self, msg):
        """Analyze terrain traversability from point cloud data"""
        # Convert point cloud to numpy array for analysis
        points = self.pointcloud_to_array(msg)

        # Analyze terrain characteristics
        traversability_map = self.analyze_terrain(points)

        # Update global costmap with terrain information
        self.update_costmap_with_terrain(traversability_map)

    def analyze_terrain(self, points):
        """Analyze terrain for humanoid traversability"""
        # Calculate height variations (step heights)
        height_diffs = self.calculate_height_differences(points)

        # Calculate slope angles
        slopes = self.calculate_surface_slopes(points)

        # Determine traversability based on humanoid constraints
        traversability = np.ones(len(points))

        # Mark areas with excessive step height as non-traversable
        step_height_violations = height_diffs > self.step_height_threshold
        traversability[step_height_violations] = 0.1  # Very low traversability

        # Mark areas with excessive slope as difficult
        slope_violations = slopes > self.slope_threshold
        traversability[slope_violations] *= 0.3  # Reduced traversability

        return traversability

    def calculate_height_differences(self, points):
        """Calculate height differences between adjacent points"""
        # Simplified implementation - in practice would use more sophisticated methods
        if len(points) < 2:
            return np.array([])

        height_diffs = np.abs(points[1:, 2] - points[:-1, 2])
        return np.append(height_diffs, 0)  # Pad to match original length

    def calculate_surface_slopes(self, points):
        """Calculate surface slopes from point cloud"""
        # Simplified implementation
        if len(points) < 3:
            return np.array([0.0])

        slopes = np.zeros(len(points))
        for i in range(1, len(points) - 1):
            # Calculate slope between three consecutive points
            dx = points[i+1, 0] - points[i-1, 0]
            dz = points[i+1, 2] - points[i-1, 2]

            if dx != 0:
                slope_rad = np.arctan(abs(dz/dx))
                slopes[i] = np.degrees(slope_rad)

        return slopes

    def update_costmap_with_terrain(self, traversability_map):
        """Update navigation costmap with terrain information"""
        # This would interface with the Nav2 costmap to update terrain costs
        # Implementation would depend on specific Nav2 integration approach
        pass

def main(args=None):
    rclpy.init(args=args)
    terrain_node = TerrainAnalyzer()
    rclpy.spin(terrain_node)
    terrain_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Examples of Nav2 for Humanoid Navigation

Let's look at practical examples of implementing Nav2 for humanoid robot navigation.

### Example 1: Indoor Navigation for Humanoid Robot

Setting up Nav2 for indoor humanoid navigation with obstacle avoidance:

1. **Map Creation**: Create a detailed map of the indoor environment
2. **Costmap Configuration**: Configure costmaps with humanoid-specific parameters
3. **Path Planning**: Use footstep planning algorithms for stable path execution
4. **Dynamic Avoidance**: Implement human-aware obstacle avoidance

### Example 2: Multi-Floor Navigation

For humanoid robots navigating multi-floor environments:

1. **Elevator Integration**: Handle elevator usage for floor transitions
2. **Stair Navigation**: Specialized path planning for stairs (if robot capable)
3. **Localization Across Floors**: Maintain accurate localization during floor changes

## Summary

In this chapter, you've learned about configuring Nav2 for humanoid robot navigation:

1. **Path Planning Concepts**: Understanding the unique requirements for bipedal path planning
2. **Nav2 Adaptation**: Modifying standard Nav2 for humanoid-specific needs
3. **Costmap Configuration**: Setting up costmaps with humanoid constraints
4. **Dynamic Obstacle Avoidance**: Implementing avoidance strategies for walking robots
5. **Terrain Considerations**: Addressing ground traversability for bipedal locomotion

These capabilities enable humanoid robots to navigate complex environments safely and effectively, accounting for the unique challenges of bipedal locomotion.

## Next Steps

With the completion of this module, you now understand the complete AI-Robot Brain system using the NVIDIA Isaac ecosystem:
- Using Isaac Sim for perception training with photorealistic simulation
- Implementing Isaac ROS for hardware-accelerated perception pipelines
- Configuring Nav2 for humanoid robot navigation with specialized path planning

This comprehensive system enables the development of sophisticated humanoid robot autonomy systems.

← [Previous Chapter: Isaac ROS for Real-Time Perception](./chapter-2-isaac-ros.md)