# Autonomous Navigation of a Multi-Trailer SUV System
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Gazebo](https://img.shields.io/badge/Simulation-Gazebo-orange)](https://gazebosim.org/)
[![SolidWorks](https://img.shields.io/badge/CAD-SolidWorks-red)](https://www.solidworks.com/)

This repository contains the implementation of a closed-loop control system for an SUV with two attached trailers. Developed as part of the **ENPM662: Introduction to Robot Modeling** course, this project covers the full robotics pipeline from CAD modeling to autonomous navigation.

## 🚀 Project Overview

The project demonstrates the autonomous navigation of a non-holonomic multi-body vehicle system. By integrating LiDAR sensors and a custom PID controller, the SUV system can navigate from a starting position to a designated goal while managing the complex dynamics of two trailing links.

### Key Features
* **Multi-Link Modeling:** Custom SUV + 2 Trailer assembly designed in SolidWorks and converted to URDF.
* **ROS 2 & Gazebo Integration:** Fully compatible URDF with Gazebo controllers for physics-based simulation.
* **Autonomous Navigation:** PID-based closed-loop control for point-to-point movement.
* **Teleoperation:** Keyboard-based manual control for testing dynamics and stability.
* **High-Fidelity Simulation:** Integration with FalconSim (Duality) and Unreal Engine via FBX conversion.

## 👥 Team members

1. Aakash Dammala

2. Akshun Sharma

3. Matthew Kalavritinos

4. Pranav Koli

In this exercise, we

1. Design a suv (with 2 trailers attached) model in Solidworks and convert to URDF - `Pranav`

2. Made URDF compatible with ros2 - `Mathew`

3. Add ros2 / gazebo controllers to the urdf model - `Mathew`

4. Visualize the robot model in gazebo and rviz - `Mathew`

5. Create a closed loop control node using pid algorithm, to move the suv from (0, 0) to (10, 10) - `Aakash`

6. Create a teleoperation script, to control the vehicle using keyboard - `Akshun`

7. Creating the project report for Phase 1 - `Akshun`

8. Converting URDF to FBX file - `Pranav`

9. Setting up logic and physics of the suv in the Unreal Editor- `Pranav`

10. Simulating in Falcon Sim - `Pranav, Akshun and Aakash`

## 📂 Repository Structure
```text
├── CAD/                        # SolidWorks model files
├── SUV/                        # ROS 2 package for the SUV model (URDF/Launch)
├── closed_loop_controller/     # PID control node
├── suv_teleop/                 # Keyboard teleoperation scripts
├── falconsim/                  # Simulation files for FalconSim
├── fbx/                        # Exported FBX models for Unreal Engine
└── SUV_report.pdf              # Detailed project report
```

## ⚙️ Installation & Setup
### Prerequisites
* Ubuntu 22.04
* ROS 2 Humble
* Gazebo Ignition / Classic
  
### Build Instructions
```bash
# Setup the directory structure and clone the repo
mkdir -p ~/modelling_ws/src
cd ~/modelling_ws/src
git clone git@github.com:AakashDammala/robot-modelling-project1.git

# Build the code
cd .. && colcon build --symlink-install
source install/setup.bash
```

## 🚦 Running the Simulation
**Autonomous Navigation (Closed-Loop)**
```bash
# Launch gazebo and closed loop controller node
ros2 launch SUV gazebo.launch.py
ros2 run closed_loop_controller controller
```

**Manual Teleoperation**
```bash
# Launch gazebo in 'competition arena' world, and teleop node
ros2 launch SUV gazebo_teleop.launch
ros2 run suv_teleop keyboard_teleop
```

## Outcomes
* Developed a complex multi-link robot model consisting of an SUV with two attached trailers in SolidWorks, exporting the assembly via URDF for ROS 2 compatibility in Gazebo and RViz.
* Engineered a closed-loop PID control system and integrated LiDAR sensors to enable autonomous point-to-point navigation and obstacle detection within high-fidelity simulation environments including FalconSim (Duality).
