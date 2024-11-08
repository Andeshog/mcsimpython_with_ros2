from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    xacro_path = PathJoinSubstitution([FindPackageShare('ma1_description'), 'urdf', 'model', 'ma1.urdf.xacro'])
    rviz_config_path = PathJoinSubstitution([FindPackageShare('ma1_description'), 'rviz', 'ma_view.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=xacro_path, description='Path to the Xacro file'),
        DeclareLaunchArgument('rvizconfig', default_value=rviz_config_path, description='Path to the RViz config file'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('model')])
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            output='screen'
        )
    ])