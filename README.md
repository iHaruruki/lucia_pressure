# Pressure Serial Reader

A ROS 2 Humble package that reads pressure sensor data from serial communication and publishes it to ROS topics, based on the reference C++ program.

## Features

- Reads data from serial port (default `/dev/ttyUSB0`) at 2000000 baud rate
- Parses packets with header `0xAA` and tail `0x55`
- Processes pressure data when `vec[1] == 200` and packet length is 13
- Supports Board IDs 10, 11, 12 with proper sensor mapping (based on reference code):
  - Board 10: sensors 16-23 (vec[4-11] → matrix[16-23])
  - Board 11: sensors 8-15 (vec[4-11] → matrix[8-15]) 
  - Board 12: sensors 0-7 (vec[4-11] → matrix[0-7])
- Implements 3-sample moving average and stability detection (difference < 5)
- Publishes individual pressure values using `sensor_msgs/msg/FluidPressure` on topics:
  - `/pressure/sensor_0` through `/pressure/sensor_23`
- Publishes averaged pressure array using `std_msgs/msg/Float64MultiArray` on topic:
  - `/pressure/averaged_array`

## Dependencies

- rclcpp
- sensor_msgs
- std_msgs

## Parameters

- `serial_port`: Serial device path (default: `/dev/ttyUSB0`)
- `baud_rate`: Serial communication baud rate (default: `2000000`)
- `buffer_size`: Serial buffer size in bytes (default: `26`)
- `frame_id`: Frame ID for sensor messages (default: `pressure_sensor`)
- `publish_rate`: Data reading frequency in Hz (default: `10.0`)

## Installation

### Build the package

```bash
colcon build --packages-select pressure_serial_reader
```

## Usage

### Run the node

```bash
# Source the workspace
source install/setup.bash

# Run the node directly
ros2 run pressure_serial_reader pressure_serial_reader_node

# Or use the launch file (recommended)
ros2 launch pressure_serial_reader pressure_reader.launch.py
```

### Monitor pressure data

```bash
# Listen to individual sensor topics
ros2 topic echo /pressure/sensor_0
ros2 topic echo /pressure/sensor_15
ros2 topic echo /pressure/sensor_23

# Listen to averaged array
ros2 topic echo /pressure/averaged_array

# List all pressure topics
ros2 topic list | grep pressure
```

### Custom parameters

```bash
# Run with custom serial port
ros2 launch pressure_serial_reader pressure_reader.launch.py serial_port:=/dev/ttyUSB1

# Run with custom baud rate  
ros2 launch pressure_serial_reader pressure_reader.launch.py baud_rate:=1000000
```

## Serial Packet Format

The node expects serial packets in the following format:
- Header: `0xAA`
- vec[1]: Format type (must be 200 for pressure data)
- vec[2]: Board ID (10, 11, or 12)
- vec[4-11]: 8 pressure values (Board 12) or vec[4-7]: 4 pressure values (Boards 10, 11)
- Tail: `0x55`
- Total packet length: 13 bytes

## Data Processing

### Board ID Mapping
- **Board 10**: vec[4-11] maps to pressure sensors 16-23
- **Board 11**: vec[4-11] maps to pressure sensors 8-15  
- **Board 12**: vec[4-11] maps to pressure sensors 0-7

### Moving Average
- Maintains 3-sample history for each sensor
- Calculates rolling average: `(sample1 + sample2 + sample3) / 3`

### Stability Detection
- Checks if all 3 samples are within ±5 of the average
- Only publishes data when all sensors are stable
- Resets collection if instability detected

## Troubleshooting

### Serial Port Issues
```bash
# Check if device exists
ls -l /dev/ttyUSB*

# Check permissions
sudo chmod 666 /dev/ttyUSB0

# Add user to dialout group
sudo usermod -a -G dialout $USER
```

### High Baud Rate Issues
If 2000000 baud is not supported on your system, the node will automatically fall back to 1000000 baud with a warning message.

### No Data Received
- Verify serial connection and baud rate
- Check that data format matches expected packet structure
- Enable debug logging: `ros2 launch pressure_serial_reader pressure_reader.launch.py --ros-args --log-level debug`