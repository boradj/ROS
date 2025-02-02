# ROS Navigation and Trajectory Control

## Overview
This repository contains two ROS-based projects that focus on trajectory generation and point-to-point navigation:

1. **Trajectory Generation and Following with TurtleBot** - Implements trajectory generation using **SymPy** and controls a TurtleBot to follow the path using a **PI Controller**.
2. **ROS Point-to-Point Navigation with Square Trajectory** - A mobile robot navigation system that follows a predefined **square trajectory** using **PID controllers**.

## Repository Structure
```
ROS_Navigation_Project/
├── Trajectory_Generation/   # TurtleBot trajectory generation and control
│   ├── Traj_generator.py    # Generates an elliptical trajectory
│   ├── Trajectory_follow.py # Uses ROS to follow the trajectory
│   ├── README.md            # Detailed instructions for this project
│
├── Point_to_Point_Navigation/ # Square trajectory navigation
│   ├── multipoint_square.py  # Implements waypoints-based navigation
│   ├── README.md             # Detailed instructions for this project
│
├── LICENSE
├── .gitignore
└── README.md  # (This file) Master documentation
```

---

## 1. Trajectory Generation and Following with TurtleBot
### Features
- Generates an elliptical trajectory using **SymPy**.
- Controls the TurtleBot using a **PI controller**.
- Plots the followed trajectory for analysis.

### Installation
```bash
pip install sympy
```

### Execution
```bash
rosrun trajectory_generation Traj_generator.py
rosrun trajectory_generation Trajectory_follow.py
```

More details are available in the [Trajectory_Generation README](Trajectory_Generation/README.md).

---

## 2. ROS Point-to-Point Navigation with Square Trajectory
### Features
- Navigates a mobile robot through **waypoints** forming a 3x3m square.
- Uses **PID controllers** for accurate trajectory following.
- Implements **real-time odometry feedback**.

### Prerequisites
- ROS Noetic
- Ubuntu 20.04
- Required ROS packages: `nav_msgs`, `geometry_msgs`, `tf`

### Installation
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/boradj/ROS.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Execution
```bash
rosrun Point_to_Point_Navigation multipoint_square.py
```

More details are available in the [Point_to_Point_Navigation README](Point_to_Point_Navigation/README.md).

---

## Contributing
1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License
This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

## Authors
- **Jaydip Borad** (jaydipborad@gmail.com)

## Acknowledgments
- Thanks to the **ROS community** for the framework and tools.
- Special thanks to **Deggendorf Institute of Technology** for support and resources.


