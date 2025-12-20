---
sidebar_position: 5
title: "Quickstart Guide"
description: "Getting started with Digital Twin simulation using Gazebo and Unity"
---

# Quickstart Guide: Digital Twin Simulation

## Overview

This quickstart guide will help you get up and running with digital twin simulation using Gazebo and Unity. By the end of this guide, you'll have a basic understanding of how to set up and use both tools for humanoid robot simulation.

## Prerequisites

Before starting with digital twin simulation, ensure you have:

- Basic knowledge of ROS 2 (Robot Operating System)
- Understanding of 3D concepts and simulation principles
- Access to appropriate hardware (recommended: NVIDIA GPU for Unity rendering)

## Setting Up Gazebo for Physics Simulation

### Basic Gazebo Installation

First, install Gazebo for physics simulation:

```bash
# For Ubuntu with ROS 2 Humble
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev
```

### Running Your First Simulation

1. Launch Gazebo:
```bash
# Start Gazebo with an empty world
gz sim -r empty.sdf
```

2. Create a simple world file with physics properties:
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="digital_twin_world">
    <!-- Gravity -->
    <gravity>0 0 -9.8</gravity>

    <!-- Physics engine -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

## Setting Up Unity for Visual Rendering

### Unity Installation

1. Download and install Unity Hub from the Unity website
2. Install Unity version 2021.3 LTS or newer
3. Install the Unity Robotics Hub package through the Package Manager

### Basic Unity Scene Setup

1. Create a new 3D project
2. Import the Unity Robotics packages:
   - Go to Window → Package Manager
   - Install "ROS TCP Connector" and related packages

3. Set up a basic scene with lighting:
```csharp
// Example: Basic lighting setup
using UnityEngine;

public class DigitalTwinLighting : MonoBehaviour
{
    void Start()
    {
        // Create directional light
        GameObject lightObj = new GameObject("Main Light");
        Light mainLight = lightObj.AddComponent<Light>();
        mainLight.type = LightType.Directional;
        mainLight.color = Color.white;
        mainLight.intensity = 1.0f;

        // Position the light
        lightObj.transform.position = new Vector3(0, 10, 0);
        lightObj.transform.LookAt(Vector3.zero);
    }
}
```

## Connecting Gazebo and Unity via ROS

### Basic Communication Setup

1. Start ROS 2 daemon:
```bash
# In terminal 1
source /opt/ros/humble/setup.bash
ros2 daemon start
```

2. Set up ROS bridge for communication:
```bash
# Install ROS bridge
sudo apt install ros-humble-rosbridge-suite
```

3. In Unity, configure the ROS connection:
```csharp
using Unity.Robotics.ROSTCPConnector;

public class RobotBridge : MonoBehaviour
{
    void Start()
    {
        // Connect to ROS
        ROSConnection.GetOrCreateInstance().Initialize("127.0.0.1", 10000);
    }
}
```

## Creating Your First Digital Twin

### Simple Robot Model

1. Create a basic robot model in Gazebo format (SDF):
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="simple_robot">
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      </inertial>
      <collision name="collision">
        <geometry>
          <box><size>0.5 0.3 0.2</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>0.5 0.3 0.2</size></box>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.8 1</ambient>
          <diffuse>0.2 0.2 0.9 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

2. Visualize the robot in Unity by subscribing to its state:
```csharp
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std_msgs;

public class RobotVisualizer : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<Float64MultiArrayMsg>(
            "robot_state", UpdateRobot);
    }

    void UpdateRobot(Float64MultiArrayMsg robotState)
    {
        // Update robot visualization based on state
        // This is a simplified example
    }
}
```

## Running a Complete Digital Twin

### Launch Sequence

1. Start ROS:
```bash
source /opt/ros/humble/setup.bash
```

2. Launch Gazebo simulation:
```bash
gz sim -r your_world.sdf
```

3. Start Unity scene (with ROS connection configured)

4. Monitor and control through ROS topics:
```bash
# View available topics
ros2 topic list

# Monitor robot state
ros2 topic echo /robot_state sensor_msgs/JointState
```

## Next Steps

After completing this quickstart guide, you can:

1. Explore [Chapter 1: Physics Simulation with Gazebo](./chapter-1-gazebo-physics-simulation.md) for in-depth physics simulation concepts
2. Continue with [Chapter 2: High-Fidelity Environments with Unity](./chapter-2-unity-digital-twins.md) for advanced visualization techniques
3. Learn about [Chapter 3: Sensor Simulation](./chapter-3-sensor-simulation.md) for realistic perception systems

This quickstart provides the foundation for building comprehensive digital twin systems for humanoid robotics applications.