from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pressure_serial_reader',
            executable='pressure_serial_reader_node',
            name='pressure_serial_reader_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},
                {'baud_rate': 9600},
                {'read_frequency': 100.0}
            ]
        )
    ])