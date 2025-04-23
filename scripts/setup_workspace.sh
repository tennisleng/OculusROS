#!/bin/bash

# Script to set up a ROS workspace for Oculus VR and robotic arm control
set -e  # Exit on error

echo "Setting up ROS workspace for Oculus VR and Robotic Arm Control"
echo "==========================================================="

# Determine ROS version
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS environment not detected. Please source your ROS installation first."
    echo "Example: source /opt/ros/noetic/setup.bash"
    exit 1
fi

echo "Detected ROS distribution: $ROS_DISTRO"

# Create catkin workspace if it doesn't exist
WORKSPACE_PATH="$HOME/catkin_ws"
if [ ! -d "$WORKSPACE_PATH/src" ]; then
    echo "Creating new catkin workspace at $WORKSPACE_PATH"
    mkdir -p "$WORKSPACE_PATH/src"
    cd "$WORKSPACE_PATH"
    catkin_make
    source devel/setup.bash
else
    echo "Using existing workspace at $WORKSPACE_PATH"
    cd "$WORKSPACE_PATH"
    source devel/setup.bash
fi

# Install required ROS packages
echo "Installing required ROS packages..."
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-rosbridge-server \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-plot \
    python3-pip

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install numpy==1.20.0 \
    matplotlib==3.3.4 \
    pandas==1.2.2 \
    websocket-client==0.57.0

# Link the current repository to the workspace
if [ ! -L "$WORKSPACE_PATH/src/oculus_arm_control" ]; then
    echo "Linking repository to workspace..."
    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
    REPO_DIR="$(dirname "$SCRIPT_DIR")"
    ln -s "$REPO_DIR" "$WORKSPACE_PATH/src/oculus_arm_control"
    echo "Repository linked to workspace"
else
    echo "Repository already linked to workspace"
fi

# Build the workspace
echo "Building the workspace..."
cd "$WORKSPACE_PATH"
catkin_make

echo "Setup complete! You can now use the Oculus VR and Robotic Arm Control package."
echo "To get started, run:"
echo "  source $WORKSPACE_PATH/devel/setup.bash"
echo "  roslaunch oculus_arm_control oculus_arm_control.launch" 