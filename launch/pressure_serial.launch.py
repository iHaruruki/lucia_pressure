from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pressure_simple',
            executable='pressure_serial_node',
            name='pressure_serial_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baud_rate': 2000000,
                'command_delay_ms': 100,
                'board_ids': [10, 11, 12],
            }]
        )
    ])