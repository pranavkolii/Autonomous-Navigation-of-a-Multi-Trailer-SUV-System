# Robot Modelling project 1

In this exercise, we

1. Design a suv (with 2 trailers attached) model in Solidworks

2. Export the solidworks model to urdf, and made compatible with ros2

3. Add ros2 / gazebo controllers to the urdf model

4. Visuaalize the robot model in gazebo and rviz

5. Create a closed loop control node using pid algorithm, to move the suv from (0, 0) to (10, 10)

6. Create a teleoperation script, to control the vehicle using keyboard  

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
git clone https://github.com/AakashDammala/robot-modelling-project1.git

# Build the code, and launch gazebo and closed loop controller
cd .. && colcon build --symlink-install
source install/setup.bash
ros2 launch SUV gazebo.launch.py
ros2 run closed_loop_controller controller
```
