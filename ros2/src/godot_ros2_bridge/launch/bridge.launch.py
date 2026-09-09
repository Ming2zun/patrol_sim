from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='127.0.0.1',
                              description='IP address of the Godot simulator computer'),
        DeclareLaunchArgument('params_file',
            default_value=PathJoinSubstitution([FindPackageShare('godot_ros2_bridge'),'config','bridge.yaml'])),
        Node(package='godot_ros2_bridge', executable='bridge', name='godot_bridge',
             output='screen', parameters=[LaunchConfiguration('params_file'),
                                         {'host':LaunchConfiguration('host')}])
    ])
