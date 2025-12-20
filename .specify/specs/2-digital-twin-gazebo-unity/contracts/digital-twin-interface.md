# Digital Twin Interface Contract

## Overview
This document defines the conceptual interfaces and communication patterns for the digital twin system combining Gazebo physics simulation and Unity visual rendering.

## ROS 2 Message Interfaces

### Robot State Communication
- **Topic**: `/digital_twin/robot_state`
- **Type**: `sensor_msgs/JointState`
- **Purpose**: Synchronize robot joint positions between physics simulation and visual rendering
- **Frequency**: 50 Hz minimum
- **QoS**: Reliable delivery with transient local durability

### Sensor Data Output
- **Topic Pattern**: `/digital_twin/sensors/*`
- **Types**:
  - `sensor_msgs/LaserScan` (for LiDAR)
  - `sensor_msgs/Image` (for cameras)
  - `sensor_msgs/Imu` (for IMU)
- **Purpose**: Output simulated sensor data for AI training and testing
- **Frequency**: Variable based on sensor type (10-30 Hz for cameras, 5-20 Hz for LiDAR, 100-200 Hz for IMU)

### Environment State
- **Topic**: `/digital_twin/environment_state`
- **Type**: `std_msgs/String` or custom message
- **Purpose**: Communicate environmental conditions between physics and visual layers
- **Frequency**: 10 Hz
- **Content**: Lighting conditions, weather parameters, object positions

### Control Commands
- **Topic**: `/digital_twin/control_commands`
- **Type**: `std_msgs/Float64MultiArray`
- **Purpose**: Send control commands from AI agents to the simulated robot
- **Frequency**: 50 Hz
- **QoS**: Reliable delivery

## Service Interfaces

### Simulation Control
- **Service**: `/digital_twin/reset_simulation`
- **Type**: `std_srvs/Empty`
- **Purpose**: Reset the simulation to initial state

### World Management
- **Service**: `/digital_twin/load_world`
- **Type**: `std_srvs/SetBool`
- **Purpose**: Load different simulation environments
- **Request**: World name/path as string parameter

## Action Interfaces

### Navigation Tasks
- **Action**: `/digital_twin/navigation`
- **Type**: `nav_msgs/ExecuteTrajectory`
- **Purpose**: Execute complex navigation tasks in the digital twin environment
- **Feedback**: Progress updates during execution
- **Result**: Success/failure of navigation task

## Performance Requirements

### Timing Constraints
- Physics simulation: 1000 Hz internal, 50-100 Hz state output
- Visual rendering: 30-60 FPS (when active)
- Sensor simulation: Match real sensor frequencies
- Communication delay: <10ms between components

### Accuracy Requirements
- Physics simulation: <1% deviation from expected dynamics
- Sensor simulation: Match real sensor noise characteristics
- Visual-Physics sync: <50ms delay between physics state and visual update

## Error Handling

### Failure Modes
1. **Physics-Visual desync**: Automatic resynchronization mechanism
2. **Communication failure**: Graceful degradation with error reporting
3. **Performance degradation**: Quality reduction strategies
4. **Invalid inputs**: Validation and rejection of invalid commands

## Validation Criteria

### Interface Compliance
- All topics follow ROS 2 naming conventions
- Message types match standard ROS 2 interfaces where possible
- Services and actions have appropriate feedback mechanisms
- Error conditions are properly handled and reported