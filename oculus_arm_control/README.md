# Oculus Arm Control

A ROS package for controlling a robotic arm using an Oculus VR headset and controllers.

## Overview

This package provides a bridge between an Oculus VR system and a robotic arm. It enables intuitive control of the robot arm's end effector by mapping the Oculus controller's position and orientation to the robot's workspace.

## Features

- Maps Oculus controller movements to robot arm movements
- Adjustable scaling between VR workspace and robot workspace
- Button control for gripper open/close
- Frame transformation handling
- Support for both left and right controllers

## Dependencies

- ROS (tested on ROS Noetic)
- oculus_ros (Oculus ROS driver)
- tf2
- Your robot arm driver (e.g., Universal Robots ROS driver, Franka ROS, etc.)

## Installation

1. Clone this repository into your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/oculus_arm_control.git
```

2. Install dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:

```bash
cd ~/catkin_ws
catkin_make
```

## Usage

1. Make sure your Oculus hardware is properly connected.
2. Make sure your robot arm is properly connected and configured.
3. Launch the Oculus-to-arm control system:

```bash
roslaunch oculus_arm_control oculus_arm_control.launch
```

## Configuration

You can configure the system by modifying the parameters in the launch file or by passing them as arguments:

```bash
roslaunch oculus_arm_control oculus_arm_control.launch use_right_controller:=false scale_factor:=0.3
```

### Parameters

- `use_right_controller` (bool, default: true): Whether to use the right or left Oculus controller.
- `scale_factor` (double, default: 0.5): Scaling factor between VR space and robot space.
- `arm_base_frame` (string, default: "arm_base_link"): The base frame of the robot arm.
- `arm_ee_frame` (string, default: "arm_ee_link"): The end effector frame of the robot arm.
- `max_velocity` (double, default: 0.1): Maximum velocity for the robot arm movements.

## Customization

To adapt this package to your specific robot arm:

1. Modify the topic names in the `arm_controller_node.cpp` file to match your robot's topic names.
2. Adjust the frame names in the launch file.
3. Configure any additional safety limits or constraints for your specific robot.

## License

MIT License 