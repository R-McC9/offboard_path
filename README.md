# Offboard Control of MAHRES Lab Multirotors

This ROS2 package contains several scripts for flying multirotors created at the MARHES lab at the University of New Mexico. Specifically a fully-actuated aerial manipulator and a modified verison of PX4's Omnicopter, while using a Vicon Motion Capture system for position feedback. This package uses PX4's offboard control features in conjunction with px4_ros_com to make this possible.

**Offboard mode is inherently dangerous. <br>
Be absolutely certain you know what you are doing before attempting to fly a real vehicle.**

## Installation
This package has been tested using ROS2 Foxy on Ubuntu 20.04.

### Dependencies
This package depends on several other packages to function. These need to be added to your ROS2 workspaces src and built before the offboard_path package will function.
The [px4_msgs package](https://github.com/PX4/px4_msgs) for all the custom uORB messages used for sending data to the flight controller.
And the [offboard_msgs](https://github.com/R-McC9/offboard_msgs) package, a package of custom messages for logging data via ROS bags built specifically for this project.

Clone the latest stable version into the src of your chosen ROS2 workspace. e.g. "ros2_ws", then build and source.

```
cd ros2_ws/src
git clone https://github.com/R-McC9/offboard_path
cd ..
colcon build --packages-select offboard_path
source ros2_ws/install/setup.bash
```

## Usage

Several scripts are contained within, each for a different multirotor. The are differentiated by the words after ```offboard_ctrl_```. For example: ```offboard_ctrl_quad``` is intended for a quadrotor, while ```offboard_ctrl_omni``` is intended for the Omnicopter.
This package is intended to work identically in the real world and in simulation, so no changes should be required when changing between the two.

It is assumed the user is utilizing a motion capture system for precise feedback of position data to the multirotor over the ```/fmu/in/vehicle_visual_odometry``` topic.

**It is highly reccomended all flights be tested in PX4's Gazebo software in the loop simulation environment before attempting flights in the real world. <br>
Always have a backup pilot ready to take control in the event of an emergency.**

Executing ```ros2 run offboard_path offboard_ctrl_...``` will:
- Arm the multirotor
- Publish the requisite setpoint to the relevant ROS topic
- Follow said setpoints until the trajectory has ended
- Land the multirotor when interrupting the ```ros2 run``` terminal with ctrl+c.

## TO DO
Change from starting with ```ros2 run``` to using a launch file.

Change trajectories from part of the script to a CSV that is loaded in according to a parameter from the ```ros2 launch``` command.

Implement programatic landing instad of using keyboard interrupt exclusively.
