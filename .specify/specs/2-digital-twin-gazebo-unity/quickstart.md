# Quickstart: Digital Twin for Humanoid Robotics (Gazebo & Unity)

## Overview
Quick setup guide for the digital twin implementation using Gazebo and Unity for humanoid robotics.

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- At least 8GB RAM (16GB recommended for Unity)
- Modern CPU with support for virtualization
- GPU with decent graphics capability (for Unity rendering)

### Software Dependencies
1. **ROS 2 Humble Hawksbill**
   - Installation: Follow official ROS 2 Humble installation guide
   - Environment setup: Source ROS environment in your shell

2. **Gazebo Garden**
   - Installation: `sudo apt install ros-humble-gazebo-*` (on Ubuntu)
   - Or follow Gazebo Garden installation guide

3. **Unity Hub and Unity 2022.3 LTS or later**
   - Download from Unity website
   - Install Unity Robotics Hub package

4. **Node.js and npm** (for Docusaurus documentation)
   - Version 18 or later
   - Installation: `sudo apt install nodejs npm` (on Ubuntu)

## Setting Up the Digital Twin Environment

### 1. ROS 2 and Gazebo Setup
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Verify installation
ros2 topic list
gazebo --version
```

### 2. Create a Workspace for Simulation
```bash
mkdir -p ~/digital_twin_ws/src
cd ~/digital_twin_ws
colcon build
source install/setup.bash
```

### 3. Install Unity Robotics Packages
1. Open Unity Hub
2. Create a new 3D project
3. Go to Package Manager
4. Install "Unity Robotics Hub" from the Package Manager
5. This includes ROS-TCP-Connector and other robotics packages

### 4. Basic Gazebo Simulation
```bash
# Launch a simple Gazebo world
gazebo --verbose worlds/empty.world
```

### 5. Verify ROS-Gazebo Integration
```bash
# Start Gazebo with ROS plugins
ros2 launch gazebo_ros empty_world.launch.py

# In another terminal, check available topics
ros2 topic list | grep gazebo
```

## Running Your First Digital Twin Simulation

### 1. Physics Simulation with Gazebo
```bash
# Launch a humanoid robot in Gazebo
ros2 launch gazebo_ros spawn_entity.launch.py entity:=my_robot x:=0 y:=0 z:=1.0
```

### 2. Visual Simulation with Unity
1. In Unity, create a new scene
2. Add the ROS-TCP-Connector component to your scene
3. Configure connection settings to match ROS 2 bridge
4. Create visual representations of your robot

### 3. Sensor Simulation
```bash
# Launch sensor simulation with realistic noise models
ros2 launch my_robot_gazebo sensor_launch.py
```

## Documentation Development

### 1. Setting Up Docusaurus
```bash
# Navigate to your documentation directory
cd ~/path/to/documentation

# Install dependencies
npm install

# Start development server
npm start
```

### 2. Adding New Content
- Create new markdown files in the `docs/digital-twin-gazebo-unity/` directory
- Add entries to sidebar configuration
- Use proper frontmatter with sidebar_position, title, and description

### 3. Testing Examples
All code examples in the documentation should be tested in the simulation environment before publication.

## Common Issues and Troubleshooting

1. **Gazebo fails to start**
   - Check if GPU drivers are properly installed
   - Ensure X11 forwarding is enabled if using remote systems

2. **Unity-ROS connection fails**
   - Verify both systems are on the same network
   - Check firewall settings for the configured ports

3. **Performance issues**
   - Reduce simulation complexity for initial testing
   - Adjust rendering quality in Unity settings

## Next Steps

1. Complete the detailed chapters on Gazebo physics simulation
2. Implement Unity integration examples
3. Test sensor simulation pipelines
4. Document integration patterns between all components