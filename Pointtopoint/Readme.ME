# ROS Point-to-Point Navigation with Square Trajectory

## Overview
This ROS package implements a point-to-point navigation system for a mobile robot, featuring a square trajectory movement pattern. The robot autonomously navigates through predefined waypoints forming a 3x3 meter square, demonstrating precise position control and path following capabilities.

## Features
- Autonomous navigation through predefined waypoints
- Square trajectory movement (3x3 meters)
- Position and orientation control using PID controllers
- Real-time odometry feedback
- Return-to-home functionality
- Thread-safe implementation

## Prerequisites
- ROS Noetic
- Ubuntu 20.04
- Python 3.8+
- Required ROS packages:
  - nav_msgs
  - geometry_msgs
  - tf

## Installation


# Create a catkin workspace (if not already existing)

mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/src


# Clone the repository

git clone https://github.com/boradj/ROS.git

# Build the workspace

cd ~/catkin_ws

catkin_make

# Source the workspace

source devel/setup.bash

## Usage
### Running the Navigation Node

# Launch the navigation node

rosrun Pointtopoint/multipoint_square.py

### Node Details
The main node (`Task1`) implements the following functionality:
- Subscribes to `/odom` for robot pose feedback
- Publishes to `/cmd_vel` for robot velocity control
- Implements PID control for both linear and angular velocities
- Executes a square trajectory with the following waypoints:
  1. (1.0, 1.0)
  2. (4.0, 1.0)
  3. (4.0, 4.0)
  4. (1.0, 4.0)
  5. Returns to (1.0, 1.0)
  6. Finally returns to initial position (0.0, 0.0)

## Parameters
- `/rate`: Update rate (default: 50 Hz)
- `/A`: Angle parameter (default: 90.0 degrees)
- `/pose_des`: Desired pose [x, y, z] (default: [0.5, 0.0, 0.2])

## Control Implementation
The system uses a combination of:
- Proportional-Integral (PI) control for linear velocity
- Proportional-Integral (PI) control for angular velocity
- Error calculation based on:
  - Distance to goal
  - Angular error
  - Current orientation

Control Parameters:

Linear Control:  

- Kp = 0.9

- Ki = 0.27

Angular Control:

- Kp = 0.7

- Ki = 0.2
```

## Code Structure
```

multipoint_square.py

├── Class: Task1

│   ├── __init__(): Initializes parameters and ROS components

│   ├── transform_pose(): Transforms poses between frames

│   ├── odometry_callback(): Handles odometry feedback

│   └── spin(): Main execution loop

## Contributing
1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License
This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

## Authors
- Jaydip Borad
- Contact: jaydipborad@gmail.com

## Acknowledgments
- Thanks to ROS community for the framework and tools
- Special thanks to Deggendorf Institute of Technology for support and resources
