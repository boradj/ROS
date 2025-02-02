# Trajectory Generation and Following with TurtleBot

## Overview  
This project focuses on generating and following a trajectory using a controller. The system utilizes **SymPy** for trajectory generation and a **Proportional-Integral (PI) Controller** for path following. The trajectory is designed in an elliptical shape, and the TurtleBot follows the path while adjusting its linear and angular velocities.

## Requirements  
Before running the project, install the required dependency:  
```bash
pip install sympy
```

## Project Structure  
- **`Traj_generator.py`**: Generates the trajectory using **SymPy** and calculates the velocity parameters.  
- **`Trajectory_follow.py`**: Implements the TurtleBot controller using **ROS (Robot Operating System)** to follow the generated trajectory.  

## Steps  

### 1. Generate Trajectory  
The `Traj_generator.py` script generates an elliptical trajectory using **SymPy**. It computes:  
- **Position (`x`, `y`)**  
- **Velocity (`v`)**  
- **Angular velocity (`ω`)**  

### 2. Follow the Trajectory  
The `Trajectory_follow.py` script:  
- Reads the generated trajectory points.  
- Computes linear and angular velocity for smooth navigation.  
- Uses ROS topics to control the TurtleBot.  

### 3. Plot Graphs  
The generated points and followed path are plotted for analysis.  

---

## Experiments and Results  

### **Experiment 1: Generating Points at a Fixed Delay**  
- **Trajectory Scan:** (Figure 1.1, Figure 1.2)  
- ✅ **Advantage:** Accurately follows the generated trajectory.  
- ❌ **Disadvantage:** Takes more time to complete the trajectory.  

### **Experiment 2: Continuously Generating Points**  
- **Trajectory Scan:** (Figure 2.1, Figure 2.2)  
- ✅ **Advantage:** Faster trajectory completion.  
- ❌ **Disadvantage:** Slight inaccuracy at the beginning; slight deviation in the y-direction at some curves.  

---

## **Results**  
- The TurtleBot successfully followed the trajectory path.  
- The PI controller effectively modulated speed and adjusted angles.  
- The plotted trajectory closely matched the theoretical shape, with minimal deviation.  

## **Conclusion**  
This project demonstrates the use of **ROS1 and Python** for robotic trajectory planning and control. The TurtleBot accurately followed an elliptical trajectory using a PI controller. This work highlights the potential of **automated trajectory following** in robotics applications.

