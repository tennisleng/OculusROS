# Installation Guide

This guide provides detailed instructions for installing and setting up the Oculus VR and Robotic Arm Control system.

## System Requirements

### Hardware Requirements
- Oculus VR headset and controllers (Quest 2 or newer recommended)
- Compatible robotic arm with ROS support
- PC with sufficient GPU for VR rendering (NVIDIA GTX 1060 or better)

### Software Requirements
- Ubuntu 20.04 LTS
- ROS Noetic
- Python 3.6+
- Oculus SDK
- Rosbridge suite

## Installation Steps

### 1. Install ROS Noetic

Follow the official ROS installation instructions:

```bash
# Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Update package index
sudo apt update

# Install ROS Noetic
sudo apt install ros-noetic-desktop-full

# Initialize rosdep
sudo rosdep init
rosdep update

# Environment setup
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install build dependencies
sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### 2. Install Rosbridge Suite

Rosbridge is needed for the web interface:

```bash
sudo apt-get install ros-noetic-rosbridge-suite
```

### 3. Create a Catkin Workspace

If you don't already have a workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

### 4. Clone this Repository

```bash
cd ~/catkin_ws/src
git clone https://github.com/tennisleng/OculusROS.git
```

### 5. Install Dependencies

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 6. Build the Package

```bash
catkin_make
source devel/setup.bash
```

### 7. Install Oculus SDK for Linux

Follow the official Oculus SDK installation instructions or use our simplified setup script:

```bash
cd ~/catkin_ws/src/OculusROS/scripts
chmod +x install_oculus_sdk.sh
./install_oculus_sdk.sh
```

### 8. Test the Installation

```bash
# In terminal 1
roslaunch oculus_arm_control oculus_arm_control.launch

# In terminal 2 (for latency monitoring)
roslaunch oculus_arm_control latency_monitor.launch
```

### 9. Access the Web Interface

Open a web browser and navigate to:
```
http://localhost:8080
```

## Troubleshooting

### Common Issues

1. **Oculus Not Detected**:
   - Ensure the Oculus VR headset is connected properly
   - Check that the Oculus service is running

2. **ROS Nodes Not Connecting**:
   - Check network configuration
   - Ensure ROS_MASTER_URI is set correctly

3. **Web Interface Not Loading**:
   - Verify rosbridge is running
   - Check browser console for JavaScript errors

For more help, please open an issue on our GitHub repository. 