# ROS2 SLAM + Waypoint Navigation (Pioneer Robot)

This project implements a full ROS2 robotics pipeline including simulation, SLAM mapping, teleoperation, and waypoint-based navigation using a Pioneer robot in Gazebo.

Features

- **SLAM Mapping** using `slam_toolbox`
- **Keyboard Teleoperation** (`teleop_twist_keyboard`)
- **Waypoint Navigation** using custom controller with `.txt` input
- **Gazebo Simulation** (custom oval environment)
- **Live Visualization** in RViz
- **ROS2-Gazebo Bridge** integration
- Modular ROS2 package architecture

## System Architecture

Gazebo → /scan → ros_gz_bridge → ROS2 → slam_toolbox → /map → RViz
↓
/cmd_vel ← teleop / waypoint controller

## Project Structure

src/
├── pioneer_bringup/
├── pioneer_controller/
├── pioneer_description/
├── pioneer_gazebo/
├── pioneer_navigation/

## How to Run

1. Build workspace
colcon build
source install/setup.bash

2. Start mapping system
ros2 launch pioneer_bringup mapping.launch.py

3. Control robot (keyboard)
i   o   u
j   k   l
m   ,   .

4. Run waypoint navigation
ros2 run pioneer_controller waypoint_controller

Waypoints are defined in:

waypoints.txt

Format:

x y
x y
...

Technologies Used
- ROS2 Jazzy
- Gazebo (Ignition)
- slam_toolbox
- RViz2
- Python

Future Improvements
- Improve obstacle detection in SLAM
- Add autonomous navigation (Nav2)
- Implement path planning (A*, RRT)
- Deploy on real robot

Author

Lorin Lau
Automation & Robotics Engineering Student
