# AI-Robot Brain Interface Contract

## Overview
This document defines the conceptual interfaces and communication patterns for the AI-Robot Brain system combining Isaac Sim, Isaac ROS, and Nav2 for humanoid robotics.

## ROS 2 Message Interfaces

### Perception Data Output
- **Topic Pattern**: `/ai_robot_brain/perception/*`
- **Types**:
  - `sensor_msgs/Image` (for camera data processed by Isaac ROS)
  - `sensor_msgs/PointCloud2` (for LIDAR processed by Isaac ROS)
  - `geometry_msgs/PoseStamped` (for VSLAM pose estimates)
  - `vision_msgs/Detection2DArray` (for object detections)
- **Purpose**: Output processed perception data from Isaac ROS for navigation and decision making
- **Frequency**: Variable based on sensor type (10-30 Hz for cameras, 5-20 Hz for LIDAR, 30 Hz for VSLAM)
- **QoS**: Reliable delivery for critical navigation data, best effort for auxiliary perception

### Navigation Commands
- **Topic**: `/ai_robot_brain/navigation/commands`
- **Type**: `geometry_msgs/Twist` or `nav_msgs/Path`
- **Purpose**: Send navigation commands from Nav2 to robot controllers
- **Frequency**: 50 Hz
- **QoS**: Reliable delivery

### Simulation State
- **Topic**: `/ai_robot_brain/simulation/state`
- **Type**: `std_msgs/String` or custom message
- **Purpose**: Communicate simulation state between Isaac Sim and other components
- **Frequency**: 10 Hz
- **Content**: Simulation status, environment parameters, sensor simulation status

### Training Data Output
- **Topic Pattern**: `/ai_robot_brain/training_data/*`
- **Type**: Custom messages for synthetic dataset
- **Purpose**: Output synthetic training data from Isaac Sim for perception model training
- **Frequency**: As needed during data generation
- **QoS**: Transient local for data persistence

## Service Interfaces

### Navigation Control
- **Service**: `/ai_robot_brain/navigation/set_goal`
- **Type**: `geometry_msgs/PoseStamped` request, `std_msgs/Bool` response
- **Purpose**: Set navigation goal for the humanoid robot

### Simulation Control
- **Service**: `/ai_robot_brain/simulation/control`
- **Type**: Custom service for simulation commands
- **Purpose**: Control Isaac Sim environment (reset, change scene, etc.)

### Perception Pipeline Control
- **Service**: `/ai_robot_brain/perception/control`
- **Type**: `std_srvs/SetBool`
- **Purpose**: Enable/disable perception pipelines
- **Request**: Enable/disable flag

## Action Interfaces

### Navigation Tasks
- **Action**: `/ai_robot_brain/navigation/execute_path`
- **Type**: `nav_msgs/NavigateToPose` or custom action
- **Purpose**: Execute complex navigation tasks in the AI-Robot Brain environment
- **Feedback**: Progress updates during navigation
- **Result**: Success/failure of navigation task

### Training Tasks
- **Action**: `/ai_robot_brain/training/generate_dataset`
- **Type**: Custom action for dataset generation
- **Purpose**: Generate synthetic training datasets using Isaac Sim
- **Feedback**: Progress of data generation
- **Result**: Dataset generation success/failure

## Performance Requirements

### Timing Constraints
- Isaac ROS perception: <50ms processing latency for real-time performance
- Navigation planning: <100ms for dynamic obstacle avoidance
- Simulation update: Match real-time for effective training data
- Communication delay: <10ms between components

### Accuracy Requirements
- VSLAM localization: <5cm accuracy for indoor navigation
- Object detection: >90% accuracy for known objects
- Path planning: Collision-free paths with safety margin
- Synthetic data: Match real-world characteristics within 10%

## Error Handling

### Failure Modes
1. **Perception failure**: Fallback to alternative sensors or navigation modes
2. **Navigation failure**: Return to safe location or stop robot
3. **Simulation failure**: Graceful degradation with error reporting
4. **Communication failure**: Local decision making with limited capabilities

## Validation Criteria

### Interface Compliance
- All topics follow ROS 2 naming conventions
- Message types match standard ROS 2 interfaces where possible
- Services and actions have appropriate feedback mechanisms
- Error conditions are properly handled and reported