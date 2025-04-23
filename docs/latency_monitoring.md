# Latency Monitoring System Guide

This guide explains how to use the comprehensive latency monitoring system for the Oculus VR and robotic arm control package.

## Overview

The latency monitoring system measures, analyzes, and visualizes network latency between different components of the VR-to-robot control pipeline. It offers:

- Real-time measurement of network latency
- Statistical analysis of latency, jitter, and packet loss
- Network condition simulation for robustness testing
- Web-based visualization dashboard
- Data logging for offline analysis

## Starting the Latency Monitor

### Basic Monitoring

```bash
roslaunch oculus_arm_control latency_monitor.launch
```

This will start:
- The latency monitoring node
- A responder node
- The rosbridge server
- A web server for the dashboard
- rqt_plot for visualization

### With Network Simulation

To enable network condition simulation:

```bash
roslaunch oculus_arm_control latency_monitor.launch simulate_network:=true
```

You can also specify specific network conditions:

```bash
roslaunch oculus_arm_control latency_monitor.launch \
  simulate_network:=true \
  fixed_delay:=50 \
  variable_delay:=10 \
  network_jitter:=5 \
  packet_loss:=0.01
```

### With Data Logging

To enable data logging:

```bash
roslaunch oculus_arm_control latency_monitor.launch enable_logging:=true
```

Logs will be saved to `[package_path]/logs/` directory.

## Web Dashboard

### Accessing the Dashboard

Open a web browser and navigate to:
```
http://localhost:8080
```

### Dashboard Features

The dashboard includes:

1. **Connection Panel**: Connect to the rosbridge server
2. **Dashboard Tab**: Quick overview of key metrics with gauges
3. **Charts Tab**: Detailed time-series charts for latency, jitter, and packet loss
4. **VR Latency Tab**: VR-specific latency metrics
5. **Network Controls Tab**: Simulate various network conditions
6. **Logs Tab**: View event logs

### Network Simulation Controls

The Network Controls tab allows you to simulate various network conditions:

- **Fixed Delay**: Add a constant delay to all packets
- **Variable Delay**: Add a random delay between 0 and the specified value
- **Network Jitter**: Add positive or negative random variations to delay
- **Packet Loss**: Randomly drop packets at the specified rate
- **Burst Loss**: Simulate periods of high packet loss

Preset configurations like "Good WiFi", "Mobile Network", etc. are also available.

## Command Line Interface

For advanced usage, the latency monitoring node supports command-line options:

```bash
rosrun oculus_arm_control network_latency_node --help
```

Available options include:
- `--rosbridge_url`: URL for rosbridge WebSocket
- `--ping_interval`: Time between pings in seconds
- `--window_size`: Sample window size for statistics
- `--timeout`: Timeout threshold for packet loss detection
- `--log_file`: Path to write latency data
- `--vr_monitor`: Enable VR-specific latency monitoring
- `--verbose`: Enable detailed console logging

## ROS Topics

### Published Topics

- `/latency/ping`: Ping messages for latency measurement
- `/latency/measurement`: Current latency measurements
- `/latency/jitter`: Jitter measurements
- `/latency/packet_loss`: Packet loss rate
- `/latency/quality_score`: Overall network quality score (0-5)
- `/latency/vr_to_arm`: VR to arm latency measurements (if enabled)

### Subscribed Topics

- `/latency/pong`: Responses to ping messages
- `/oculus/right_controller/pose`: VR controller pose (if VR monitoring enabled)
- `/arm_controller/status`: Arm controller status (if VR monitoring enabled)

## Data Analysis

### Log File Format

Log files are in CSV format with the following columns:
- Timestamp
- Sequence number
- Latency in milliseconds
- Timeout flag (1 = timeout, 0 = received)
- Current packet loss rate
- Current jitter

### Analyzing with External Tools

The CSV log files can be analyzed with tools like:
- Excel/LibreOffice Calc
- Python (pandas, matplotlib)
- R
- MATLAB

Example Python script for analyzing logs:

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load the data
data = pd.read_csv('latency_log.csv')

# Calculate statistics
mean_latency = data['latency_ms'].mean()
max_latency = data['latency_ms'].max()
packet_loss = data['is_timeout'].mean()

# Create a plot
plt.figure(figsize=(10, 6))
plt.plot(data['timestamp'], data['latency_ms'])
plt.xlabel('Time')
plt.ylabel('Latency (ms)')
plt.title(f'Latency Over Time (Mean: {mean_latency:.2f}ms, Loss: {packet_loss*100:.2f}%)')
plt.grid(True)
plt.savefig('latency_analysis.png')
```

## Troubleshooting

### Common Issues

1. **High Latency Values**:
   - Check system CPU usage
   - Check network congestion
   - Verify no competing network-intensive applications

2. **Web Interface Not Updating**:
   - Check browser console for errors
   - Verify rosbridge connection is established
   - Restart the browser if WebSocket connection fails

3. **Data Logging Issues**:
   - Check file permissions in the logs directory
   - Verify disk space is available

For more help, consult the [GitHub issues page](https://github.com/tennisleng/OculusROS/issues). 