"""CottonSim KISS-ICP port, driven by Alpine's simulated odometry prior."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share=Path(get_package_share_directory('godot_ros2_bridge'))
    sim={'use_sim_time':True}
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('host',default_value='127.0.0.1'),
        Node(package='godot_ros2_bridge',executable='bridge',name='godot_bridge',output='screen',
             parameters=[str(share/'config/bridge.yaml'),sim,{'host':LaunchConfiguration('host'),
               'odom_frame':'map','odom_topic':'/sim/localization/odom','ground_truth_topic':'/sim/ground_truth/pose',
               'publish_odom_tf':True}]),
        Node(package='godot_ros2_bridge',executable='trajectory',output='screen',parameters=[sim]),
        Node(package='alpine_sim_slam',executable='mapping',output='screen',parameters=[sim]),
        Node(package='godot_ros2_bridge',executable='accumulated_cloud',output='screen',
             parameters=[sim,{'min_z_m':-1000.,'radius_m':80.,'voxel_size_m':.25,'publish_hz':1.}]),
        Node(package='godot_ros2_bridge',executable='vehicle_markers',output='screen',parameters=[sim]),
        Node(package='godot_ros2_bridge',executable='rviz_path',output='screen',parameters=[sim]),
        Node(package='godot_ros2_bridge',executable='path_follower',output='screen',parameters=[sim]),
        Node(package='rviz2',executable='rviz2',output='screen',condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[sim,{'test_c_corner_radius_m':3.5,'test_c_min_corner_radius_m':3.0}],
             arguments=['-d',str(share/'rviz/alpine.rviz')])
    ])
