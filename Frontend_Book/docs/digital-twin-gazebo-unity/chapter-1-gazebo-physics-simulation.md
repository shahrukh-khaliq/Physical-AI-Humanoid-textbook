---
sidebar_position: 2
title: "Physics Simulation with Gazebo"
description: "Simulating gravity, collisions, and dynamics for humanoid robots using Gazebo"
---

# Physics Simulation with Gazebo

## Learning Objectives

By the end of this chapter, you will be able to:
- Set up Gazebo for humanoid robot physics simulation
- Configure gravity and dynamics parameters for realistic simulation
- Implement collision detection and response for humanoid robots
- Build custom worlds for robot simulation and testing
- Integrate robot models with Gazebo's physics engine

## Introduction to Gazebo Physics Simulation

Gazebo is a powerful physics simulation engine that provides realistic simulation of robots in complex environments. For humanoid robotics, Gazebo offers the essential capabilities needed to simulate the complex dynamics of bipedal locomotion and interaction with the environment.

### Key Physics Concepts in Gazebo

Gazebo's physics engine handles several fundamental concepts that are crucial for humanoid robot simulation:

- **Gravity**: Simulates realistic gravitational forces affecting all objects
- **Collisions**: Detects and responds to physical contact between objects
- **Dynamics**: Calculates motion based on forces, torques, and constraints
- **Joints**: Models the connections between different parts of a robot
- **Friction**: Simulates surface interactions and grip characteristics

### Gazebo vs. Real-World Physics

Understanding the relationship between Gazebo simulation and real-world physics is essential:

- Simulation parameters should match real-world values as closely as possible
- Some simplifications are necessary for computational efficiency
- Validation against real-world data helps ensure simulation accuracy
- Tuning parameters may be needed to achieve desired behavior

## Simulating Gravity in Gazebo

Gravity is a fundamental force that affects all objects in the simulation. For humanoid robots, accurate gravity simulation is crucial for realistic movement and balance.

### Configuring Global Gravity

Gazebo allows you to configure global gravity parameters in the world file:

```xml
<!-- Example: Setting gravity in a Gazebo world file -->
<sdf version='1.7'>
  <world name='humanoid_world'>
    <!-- Set global gravity (negative Z for downward force) -->
    <gravity>0 0 -9.8</gravity>

    <!-- World properties -->
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Your robot and environment models go here -->
  </world>
</sdf>
```

### Gravity Considerations for Humanoid Robots

When simulating gravity for humanoid robots, consider:

- The robot's center of mass and how it affects balance
- Appropriate joint stiffness to maintain stability
- Ground contact properties for realistic foot interaction
- Balance control algorithms that work with simulated gravity

## Collisions and Dynamics in Gazebo

Collision detection and dynamics simulation are critical for realistic humanoid robot behavior in Gazebo.

### Collision Detection Models

Gazebo offers several collision detection models:

- **Bullet**: Fast and suitable for most applications
- **ODE**: Good balance of speed and accuracy
- **Simbody**: More accurate but slower
- **DART**: Advanced dynamics with constraint solving

### Implementing Collision Models

```xml
<!-- Example: Collision model for a humanoid robot link -->
<link name="lower_leg">
  <!-- Visual properties for rendering -->
  <visual name="visual">
    <geometry>
      <cylinder>
        <radius>0.05</radius>
        <length>0.35</length>
      </cylinder>
    </geometry>
    <material>
      <ambient>0.1 0.1 0.8 1</ambient>
      <diffuse>0.2 0.2 0.9 1</diffuse>
    </material>
  </visual>

  <!-- Collision properties for physics -->
  <collision name="collision">
    <geometry>
      <cylinder>
        <radius>0.05</radius>
        <length>0.35</length>
      </cylinder>
    </geometry>
    <!-- Surface properties affecting collisions -->
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>
          <mu2>1.0</mu2>
        </ode>
      </friction>
      <bounce>
        <restitution_coefficient>0.01</restitution_coefficient>
        <threshold>100000</threshold>
      </bounce>
    </surface>
  </collision>
</link>
```

### Dynamics Configuration

Dynamics parameters affect how objects move and respond to forces:

```xml
<!-- Physics engine configuration -->
<physics type='ode'>
  <max_step_size>0.001</max_step_size>  <!-- Smaller steps for accuracy -->
  <real_time_factor>1.0</real_time_factor>  <!-- Real-time simulation -->
  <real_time_update_rate>1000</real_time_update_rate>  <!-- Update rate -->

  <!-- Solver parameters -->
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>  <!-- Iterations for constraint solving -->
      <sor>1.3</sor>     <!-- Successive over-relaxation parameter -->
    </solver>
    <constraints>
      <cfm>0.000001</cfm>  <!-- Constraint force mixing -->
      <erp>0.2</erp>       <!-- Error reduction parameter -->
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## World Building Techniques

Creating effective simulation environments is crucial for humanoid robot testing and development.

### Basic World Structure

A Gazebo world file typically includes:

1. **Environment geometry**: Floors, walls, obstacles
2. **Lighting**: Sun position, ambient lighting, directional lights
3. **Models**: Robots, objects, furniture
4. **Physics parameters**: Gravity, solver settings

### Creating a Humanoid-Friendly Environment

```xml
<!-- Example: Humanoid-friendly world -->
<sdf version='1.7'>
  <world name='humanoid_test_world'>
    <!-- Gravity -->
    <gravity>0 0 -9.8</gravity>

    <!-- Physics engine -->
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Custom floor with appropriate friction for walking -->
    <model name='walking_surface'>
      <pose>0 0 0 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>
                <mu2>0.8</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add obstacles for navigation testing -->
    <model name='obstacle_block'>
      <pose>2 0 0.2 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.4 0.4 0.4</size>
            </box>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.4 0.4 0.4</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.2 0.2 1</ambient>
            <diffuse>0.7 0.3 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Robot-Environment Interaction

Humanoid robots interact with their environment in complex ways that require careful simulation setup.

### Contact Sensors

Contact sensors help detect when robot parts touch the environment:

```xml
<!-- Example: Contact sensor for foot -->
<joint name="left_foot_joint" type="fixed">
  <parent>left_ankle</parent>
  <child>left_foot</child>
</joint>

<link name="left_foot">
  <collision name="foot_collision">
    <geometry>
      <box>
        <size>0.15 0.1 0.02</size>  <!-- Size of the foot -->
      </box>
    </geometry>
    <!-- Contact sensor plugin -->
    <sensor name="foot_contact" type="contact">
      <always_on>1</always_on>
      <update_rate>30</update_rate>
      <contact>
        <collision>left_foot::foot_collision</collision>
      </contact>
      <plugin name="contact_plugin" filename="libgazebo_ros_bumper.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>30.0</updateRate>
        <bumperTopicName>left_foot_bumper</bumperTopicName>
        <frameName>left_foot</frameName>
      </plugin>
    </sensor>
  </collision>
</link>
```

### Force/Torque Sensors

Force/torque sensors can be used to measure interaction forces:

```xml
<!-- Example: Force/torque sensor in the ankle joint -->
<joint name="left_ankle_joint" type="revolute">
  <parent>left_shin</parent>
  <child>left_foot</child>
  <axis>
    <xyz>0 1 0</xyz>
    <limit>
      <lower>-0.5</lower>
      <upper>0.5</upper>
      <effort>100</effort>
      <velocity>1.0</velocity>
    </limit>
  </axis>
  <!-- Force/torque sensor plugin -->
  <sensor name="left_ankle_ft_sensor" type="force_torque">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <force_torque>
      <frame>child</frame>
      <measure_direction>child_to_parent</measure_direction>
    </force_torque>
  </sensor>
</joint>
```

## Practical Examples of Gazebo Physics with Humanoid Robots

Let's look at practical examples of implementing Gazebo physics for humanoid robots.

### Example 1: Basic Humanoid Model with Physics

A minimal humanoid model with proper physics configuration:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="simple_humanoid">
    <!-- Torso -->
    <link name="torso">
      <pose>0 0 0.9 0 0 0</pose>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.2</iyy>
          <iyz>0</iyz>
          <izz>0.15</izz>
        </inertia>
      </inertial>
      <collision name="torso_collision">
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
      </collision>
      <visual name="torso_visual">
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.2 1</ambient>
          <diffuse>0.9 0.9 0.3 1</diffuse>
        </material>
      </visual>
    </link>

    <!-- Head -->
    <link name="head">
      <pose>0 0 0.3 0 0 0</pose>
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
    </link>

    <!-- Joint connecting head to torso -->
    <joint name="neck_joint" type="revolute">
      <parent>torso</parent>
      <child>head</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-0.5</lower>
          <upper>0.5</upper>
          <effort>10</effort>
          <velocity>1.0</velocity>
        </limit>
      </axis>
    </joint>
  </model>
</sdf>
```

### Example 2: Walking Pattern Simulation

Creating a simple walking pattern in simulation requires careful physics tuning:

1. Proper mass distribution in robot model
2. Appropriate joint limits and stiffness
3. Correct friction values for feet
4. Stable control algorithms

## Summary

In this chapter, you've learned about Gazebo physics simulation for humanoid robots:

1. **Gravity Simulation**: Configuring realistic gravitational forces
2. **Collision Detection**: Setting up proper collision models and properties
3. **Dynamics Configuration**: Tuning physics parameters for realistic movement
4. **World Building**: Creating appropriate environments for humanoid testing
5. **Robot-Environment Interaction**: Implementing sensors for interaction detection

These concepts form the foundation for realistic humanoid robot simulation in Gazebo. The physics simulation you've learned about here works in conjunction with the visual rendering you'll explore in [Chapter 2: High-Fidelity Environments with Unity](./chapter-2-unity-digital-twins.md).

## Next Steps

[Next Chapter: High-Fidelity Environments with Unity](./chapter-2-unity-digital-twins.md) →