# Oculus VR and Robotic Arm ROS Integration

A comprehensive system for controlling robotic arms using Oculus VR controllers with advanced network latency monitoring and visualization.

## Features

- **VR Control**: Control robotic arms using Oculus VR controllers
- **Network Latency Monitoring**: Real-time measurement and visualization of network latency
- **Network Condition Simulation**: Simulate various network conditions for robustness testing
- **Web-based Dashboard**: Interactive visualization of performance metrics
- **Data Logging**: Comprehensive logging for offline analysis

## System Architecture

The system consists of the following main components:

- **Oculus Interface**: Reads positions and button states from Oculus VR controllers
- **Arm Controller**: Converts VR controller inputs to robotic arm commands
- **Latency Monitor**: Measures and analyzes network latency between components
- **Web Interface**: Visualizes latency and performance metrics

## Getting Started

### Prerequisites

- ROS Noetic
- Oculus SDK
- Rosbridge
- Python 3.6+

### Installation

1. Clone this repository into your ROS workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/tennisleng/OculusROS.git
```

2. Build the package:
```bash
cd ~/catkin_ws
catkin_make
```

3. Source the workspace:
```bash
source ~/catkin_ws/devel/setup.bash
```

### Running the System

#### Basic VR Control

```bash
roslaunch oculus_arm_control oculus_arm_control.launch
```

#### With Latency Monitoring

```bash
roslaunch oculus_arm_control latency_monitor.launch
```

#### With Network Simulation

```bash
roslaunch oculus_arm_control latency_monitor.launch simulate_network:=true
```

### Web Interface

Once the system is running with latency monitoring enabled, access the web interface at:
```
http://localhost:8080
```

## Documentation

Detailed documentation is available in the `docs` folder.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Based on ROS and Oculus SDK
- Leverages rosbridge for web connectivity 