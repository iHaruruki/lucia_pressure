# Pressure Command Sender

A ROS 2 Humble package that sends control commands to pressure sensor boards via serial communication.

## Overview

This package sends command packets to pressure sensor boards using the following protocol:
- **Packet Structure**: `[0xAA, 0xC1, board_id, 0x00, 0x21, 0x55]`
- **Board Sequence**: Board 12 (0x0C) → Board 11 (0x0B) → Board 10 (0x0A)
- **Communication**: Serial port `/dev/ttyUSB0` at 2000000 baud rate
- **Timing**: 100ms delay between each command transmission

## Features

- **Serial Communication**: POSIX termios-based serial communication
- **ROS 2 Services**: Manual command triggering and board-specific commands
- **Topic Interface**: External command control via topics
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
├── srv/
│   └── BoardCommand.srv
├── msg/
│   └── CommandStatus.msg
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

## Services

### `~/send_single_command` (std_srvs/srv/Trigger)
Sends one complete cycle of commands to all configured boards.

### `~/send_board_command` (pressure_command_sender/srv/BoardCommand)
Sends a command to a specific board.
- **Request**: `uint8 board_id`
- **Response**: `bool success, string message`

### `~/start_continuous` (std_srvs/srv/SetBool)
Starts or stops continuous command transmission.
- **Request**: `bool data` (true=start, false=stop)
- **Response**: `bool success, string message`

## Topics

### Published
- `~/command_status` (std_msgs/msg/String): Simple status messages
- `~/command_status_detailed` (pressure_command_sender/msg/CommandStatus): Detailed status with timestamp and board info

### Subscribed
- `~/board_command` (std_msgs/msg/String): Listen for board ID to send command to

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

### Service calls
```bash
# Send single command cycle
ros2 service call /pressure_command_sender/send_single_command std_srvs/srv/Trigger

# Send command to specific board
ros2 service call /pressure_command_sender/send_board_command \
    pressure_command_sender/srv/BoardCommand "{board_id: 12}"

# Start/stop continuous mode
ros2 service call /pressure_command_sender/start_continuous \
    std_srvs/srv/SetBool "{data: false}"
```

### Topic commands
```bash
# Send command to board via topic
ros2 topic pub /pressure_command_sender/board_command std_msgs/msg/String "{data: '12'}"
```

### Monitor status
```bash
# Simple status
ros2 topic echo /pressure_command_sender/command_status

# Detailed status
ros2 topic echo /pressure_command_sender/command_status_detailed
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