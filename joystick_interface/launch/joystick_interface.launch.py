from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(get_package_share_directory('joystick_interface'), 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='joystick_interface',
            executable='joystick_interface_node',
            name='joystick_interface',
            output='screen',
            parameters=[config]
        ),
    ])