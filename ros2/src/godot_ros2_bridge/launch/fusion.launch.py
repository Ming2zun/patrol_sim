"""Start the isolated bridge and installed LIO-SAM ROS 2 backend."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share=Path(get_package_share_directory('godot_ros2_bridge'))
    bridge_config=LaunchConfiguration('bridge_config')
    fusion_config=LaunchConfiguration('fusion_config')
    actions=[
        DeclareLaunchArgument('bridge_config',default_value=str(share/'config/bridge.yaml')),
        DeclareLaunchArgument('fusion_config',default_value=str(share/'config/lio_sam_36.yaml')),
        Node(package='godot_ros2_bridge',executable='bridge',name='godot_bridge',output='screen',
             parameters=[bridge_config,{'publish_odom_tf':False,'odom_topic':'/sim/odom',
                 'ground_truth_topic':'/sim/ground_truth/pose','odom_frame':'sim_odom'}]),
        Node(package='tf2_ros',executable='static_transform_publisher',name='lio_map_odom',
             arguments=['--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0',
                        '--frame-id','map','--child-frame-id','odom'],parameters=[{'use_sim_time':True}])]
    for executable in ['lio_sam_imuPreintegration','lio_sam_imageProjection','lio_sam_featureExtraction','lio_sam_mapOptimization']:
        actions.append(Node(package='lio_sam',executable=executable,name=executable,
                            parameters=[fusion_config],output='screen'))
    return LaunchDescription(actions)
