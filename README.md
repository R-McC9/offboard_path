# Offboard Control of MAHRES Multirotors

This ROS2 package contains several scripts for flying multirotors created at the MARHES lab at the University of New Mexico. Specifically a fully-actuated aerial manipulator and a modified verison of PX4's Omnicopter while using a Vicon Motion Capture system for position feedback. This package uses PX4's offboard control features in conjunction with px4_ros_com to make this possible.

**Offboard mode is inherently dangerous. <br>
Be absolutely certain you know what you are doing before attempting to fly a real vehicle.**

## Installation
This package has been tested using ROS2 Foxy on Linux 20.04.

Clone the latest stable version into the src of your chosen ROS2 workspace. e.g. "ros2_ws", then build.

```
cd ros2_ws/src
git clone https://github.com/R-McC9/offboard_path
cd ..
colcon build --packages-select offboard_path
source ros2_ws/install/setup.bash
```

## Usage

Several scripts are contained within, each for a different multirotor. The are differentiated by the script after "offboard_ctrl_".
Executing ```ros2 run offboard_path offboard_ctrl_...``` will:
- Arm the multirotor
- Publish the requisite setpoint to the relevant ROS topic
- Follow said setpoints until the trajectory has ended
- Land the multirotor when interrupting the ```ros2 run``` terminal with ctrl+c.

## TO DO
Change from starting with ```ros2 run``` to using a launch file.
Change trajectories from part of the script to a CSV that is loaded in according to a parameter from the ```ros2 launch``` command.
