from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the params file
    config = os.path.join(
        get_package_share_directory('pressure_serial_reader'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='pressure_serial_reader',
            executable='pressure_serial_reader_node',
            name='pressure_serial_reader_node',
            output='screen',
            parameters=[config],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])