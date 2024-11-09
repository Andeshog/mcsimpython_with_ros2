import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    description_package = get_package_share_directory('ma1_description')
    joystick_package = get_package_share_directory('joystick_interface')

    rviz_launch = os.path.join(description_package, 'launch', 'description.launch.py')
    joystick_launch = os.path.join(joystick_package, 'launch', 'joystick_interface.launch.py')

    # Create the IncludeLaunchDescription actions
    rviz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch)
    )

    joystick_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joystick_launch)
    )

    odom_demo_node = Node(
        package='mcsim_odom_demo',
        executable='odom_demo.py',
        name='odom_waves_demo',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(rviz_launch_description)
    ld.add_action(joystick_launch_description)
    ld.add_action(odom_demo_node)

    return ld
