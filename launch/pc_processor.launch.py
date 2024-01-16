#!/usr/bin/python3

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    use_sim_time_val = LaunchConfiguration('use_sim_time', default=False)

    robot_name_val = LaunchConfiguration('robot_name', default='default_robot_name')
    
    return LaunchDescription([
        Node(
            namespace=robot_name_val,
            package='point_cloud_processor',
            executable='point_cloud_processor.py',
            name='pc_processor',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_val,
                         'robot_name': robot_name_val}]
        )
                
    ])