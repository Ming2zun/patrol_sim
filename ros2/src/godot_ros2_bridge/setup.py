from setuptools import setup
from glob import glob
package_name = 'godot_ros2_bridge'
setup(name=package_name, version='0.8.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),
                  ('share/'+package_name,['package.xml','LIO_SAM_LICENSE.txt','FUSION_README.md','PORTING_NOTES.md','LICENSE.apache-2.0']),
                  ('share/'+package_name+'/config',glob('config/*.yaml')),
                  ('share/'+package_name+'/launch',glob('launch/*.py')),
                  ('share/'+package_name+'/rviz',glob('rviz/*.rviz'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Alpine Patrol Project', maintainer_email='local@example.invalid',
      description='Independent Godot / ROS 2 network bridge', license='MIT',
      entry_points={'console_scripts':['bridge = godot_ros2_bridge.bridge_node:main','accumulated_cloud = godot_ros2_bridge.segmented_cloud:main','vehicle_markers = godot_ros2_bridge.vehicle_markers:main','rviz_path = godot_ros2_bridge.rviz_path:main','path_follower = godot_ros2_bridge.path_follower:main','trajectory = godot_ros2_bridge.trajectory:main']})
