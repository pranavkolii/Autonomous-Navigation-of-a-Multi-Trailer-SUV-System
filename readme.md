# Autonomous Navigation of a Multi-Trailer SUV System

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

## Team members

1. Aakash Dammala

2. Akshun Sharma

3. Matthew Kalavritinos

4. Pranav Koli

## Instructions to run the code
```bash
# Setup the directory structure and clone the repo
mkdir -p ~/modelling_ws/src
cd ~/modelling_ws/src
git clone git@github.com:AakashDammala/robot-modelling-project1.git

# Build the code
cd .. && colcon build --symlink-install
source install/setup.bash

# Launch gazebo and closed loop controller node
ros2 launch SUV gazebo.launch.py
ros2 run closed_loop_controller controller

# Launch gazebo in 'competition arena' world, and teleop node
ros2 launch SUV gazebo_teleop.launch
ros2 run suv_teleop keyboard_teleop
```

## Outcomes
* Developed a complex multi-link robot model consisting of an SUV with two attached trailers in SolidWorks, exporting the assembly via URDF for ROS 2 compatibility in Gazebo and RViz.
* Engineered a closed-loop PID control system and integrated LiDAR sensors to enable autonomous point-to-point navigation and obstacle detection within high-fidelity simulation environments including FalconSim (Duality).
