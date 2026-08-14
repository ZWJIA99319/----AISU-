import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
 root=get_package_share_directory('vehicle_bridge')
 nodes=[Node(package='vehicle_bridge',executable='vehicle_bridge_node',output='screen'),
 Node(package='tf2_ros',executable='static_transform_publisher',arguments=['0.12','0.06','0','0','0','0','base_footprint','base_link']),
 Node(package='tf2_ros',executable='static_transform_publisher',arguments=['0','0','0','0','0','0','base_footprint','gyro_link']),
 Node(package='imu_filter_madgwick',executable='imu_filter_madgwick_node',parameters=[os.path.join(root,'config','imu.yaml')]),
 Node(package='robot_localization',executable='ekf_node',remappings=[('odometry/filtered','odom_combined')],parameters=[os.path.join(root,'config','ekf.yaml')])]
 return LaunchDescription(nodes)
