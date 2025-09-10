# Pressure Serial Reader

A ROS 2 Humble package that reads pressure and potentiometer data from serial devices and publishes it to ROS topics.

## Features

- Reads data from serial port (default `/dev/ttyUSB0`) at configurable baud rate (default 9600)
- Parses packets with header `0xAA` and tail `0x55`
- Supports Board IDs 10, 11, 12 (pressure/biosignal boards)
- Extracts vital format data when `vec[1] == 197 (0xC5)`:
  - Pulse = vec[4]
  - SpO2 = (vec[6]*256 + vec[5]) * 0.1
  - BloPre = (vec[8]*256 + vec[7]) * 0.1
- Publishes pressure data using `sensor_msgs/msg/FluidPressure` on topics:
  - `/pressure/board_10`
  - `/pressure/board_11`
  - `/pressure/board_12`

## Dependencies

- rclcpp
- sensor_msgs

## Parameters

- `serial_port`: Serial device path (default: `/dev/ttyUSB0`)
- `baud_rate`: Serial communication baud rate (default: `9600`)
- `read_frequency`: Data reading frequency in Hz (default: `100.0`)

## Usage

### Build the package

```bash
colcon build --packages-select pressure_serial_reader
```

### Run the node

```bash
# Source the workspace
source install/setup.bash

# Run the node directly
ros2 run pressure_serial_reader pressure_serial_reader_node

# Or use the launch file
ros2 launch pressure_serial_reader pressure_serial_reader.launch.py
```

### Monitor pressure data

```bash
# Listen to all pressure topics
ros2 topic echo /pressure/board_10
ros2 topic echo /pressure/board_11
ros2 topic echo /pressure/board_12
```

## Serial Packet Format

The node expects serial packets in the following format:
- Header: `0xAA`
- Data bytes including board ID at position 2, format type at position 1
- Tail: `0x55`

For vital format (vec[1] == 197):
- vec[2]: Board ID (10, 11, or 12)
- vec[4]: Pulse
- vec[5], vec[6]: SpO2 data (little-endian 16-bit)
- vec[7], vec[8]: Blood pressure data (little-endian 16-bit)