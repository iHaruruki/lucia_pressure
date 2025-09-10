#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('pressure_command_sender')
    
    # Path to the parameters file
    params_file = os.path.join(package_dir, 'config', 'params.yaml')
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port device path'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='2000000',
        description='Serial communication baud rate'
    )
    
    continuous_mode_arg = DeclareLaunchArgument(
        'continuous_mode',
        default_value='true',
        description='Enable continuous command transmission'
    )
    
    command_delay_ms_arg = DeclareLaunchArgument(
        'command_delay_ms',
        default_value='100',
        description='Delay between commands in milliseconds'
    )

    # Node configuration
    pressure_command_sender_node = Node(
        package='pressure_command_sender',
        executable='pressure_command_sender_node',
        name='pressure_command_sender_node',
        parameters=[
            params_file,
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'continuous_mode': LaunchConfiguration('continuous_mode'),
                'command_delay_ms': LaunchConfiguration('command_delay_ms'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        continuous_mode_arg,
        command_delay_ms_arg,
        pressure_command_sender_node,
    ])