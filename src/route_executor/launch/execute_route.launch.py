from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    names={'line_v':'1.0','line_kp':'-0.030','route_mode':'1'}
    declarations=[DeclareLaunchArgument(k,default_value=v) for k,v in names.items()]
    process=Node(package='route_executor',executable='execute_route',output='screen',
                 parameters=[{k:LaunchConfiguration(k) for k in names}])
    return LaunchDescription(declarations+[process])
