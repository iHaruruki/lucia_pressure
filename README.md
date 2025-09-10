# Pressure Command Sender

A ROS 2 Humble package that sends control commands to pressure sensor boards via serial communication using **topic-based communication only**.

## Overview

This package sends command packets to pressure sensor boards using the following protocol:
- **Packet Structure**: `[0xAA, 0xC1, board_id, 0x00, 0x21, 0x55]`
- **Board Sequence**: Board 12 (0x0C) → Board 11 (0x0B) → Board 10 (0x0A)
- **Communication**: Serial port `/dev/ttyUSB0` at 2000000 baud rate
- **Timing**: 100ms delay between each command transmission

## Features

- **Serial Communication**: POSIX termios-based serial communication
- **Topic-Only Interface**: All control through topic publishing/subscribing (no services)
- **Status Publishing**: Real-time command transmission feedback
- **Configurable Parameters**: Flexible configuration via YAML parameters

## Package Structure

```
pressure_command_sender/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── pressure_command_sender_node.cpp
├── include/pressure_command_sender/
│   └── pressure_command_sender.hpp
├── launch/
│   └── command_sender.launch.py
├── config/
│   └── params.yaml
└── README.md
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | "/dev/ttyUSB0" | Serial device path |
| `baud_rate` | int | 2000000 | Communication baud rate |
| `command_delay_ms` | int | 100 | Delay between commands (ms) |
| `continuous_mode` | bool | true | Enable continuous transmission |
| `board_ids` | array | [12, 11, 10] | Board IDs to send commands to |
| `frame_id` | string | "pressure_command" | Frame ID for messages |

## Topics

### Command Input Topics (Subscribers)
- `~/command_single` (std_msgs/msg/Bool): Trigger single command cycle when true
- `~/command_continuous` (std_msgs/msg/Bool): Start/stop continuous mode  
- `~/command_board_10` (std_msgs/msg/Bool): Send command to board 10 when true
- `~/command_board_11` (std_msgs/msg/Bool): Send command to board 11 when true
- `~/command_board_12` (std_msgs/msg/Bool): Send command to board 12 when true

### Status Output Topics (Publishers)
- `~/command_status` (std_msgs/msg/String): Command transmission status
- `~/current_board` (std_msgs/msg/UInt8): Currently commanded board ID
- `~/transmission_count` (std_msgs/msg/UInt64): Total commands sent
- `~/continuous_mode_active` (std_msgs/msg/Bool): Current continuous mode status

## Building

```bash
# In your ROS 2 workspace
colcon build --packages-select pressure_command_sender
source install/setup.bash
```

## Usage

### Launch with default parameters
```bash
ros2 launch pressure_command_sender command_sender.launch.py
```

### Launch with custom parameters
```bash
ros2 launch pressure_command_sender command_sender.launch.py \
    serial_port:=/dev/ttyUSB1 \
    baud_rate:=115200 \
    continuous_mode:=false
```

### Topic commands
```bash
# Send single command cycle
ros2 topic pub --once /pressure_command_sender/command_single std_msgs/msg/Bool "data: true"

# Send command to specific boards
ros2 topic pub --once /pressure_command_sender/command_board_10 std_msgs/msg/Bool "data: true"
ros2 topic pub --once /pressure_command_sender/command_board_11 std_msgs/msg/Bool "data: true"
ros2 topic pub --once /pressure_command_sender/command_board_12 std_msgs/msg/Bool "data: true"

# Start/stop continuous mode
ros2 topic pub --once /pressure_command_sender/command_continuous std_msgs/msg/Bool "data: true"
ros2 topic pub --once /pressure_command_sender/command_continuous std_msgs/msg/Bool "data: false"

# Monitor status
ros2 topic echo /pressure_command_sender/command_status
ros2 topic echo /pressure_command_sender/current_board
ros2 topic echo /pressure_command_sender/transmission_count
ros2 topic echo /pressure_command_sender/continuous_mode_active
```

## Integration

This package is designed to work with the `pressure_serial_reader` package:
- Commands from this sender trigger data collection from pressure boards
- The reader processes the responses and publishes pressure data
- Together they form a complete pressure sensing system

## Technical Details

### Command Packet Format
Each command packet is 16 bytes with the following structure:
```
Byte 0: 0xAA (Header)
Byte 1: 0xC1 (Command Type)
Byte 2: Board ID (0x0A=10, 0x0B=11, 0x0C=12)
Byte 3: 0x00 (Reserved)
Byte 4: 0x21 (Command Parameter)
Byte 5: 0x55 (Tail)
Bytes 6-15: Padding (zeros)
```

### Serial Port Configuration
The serial port is configured to match the reference implementation:
- Baud rate: 2000000
- Data bits: 8
- No parity
- POSIX termios with ioctl configuration

## Troubleshooting

### Permission Issues
```bash
# Add user to dialout group for serial port access
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

### Serial Port Issues
- Verify device exists: `ls -l /dev/ttyUSB*`
- Check permissions: `ls -l /dev/ttyUSB0`
- Test communication: `sudo dmesg | grep tty`