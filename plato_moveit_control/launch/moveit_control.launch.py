import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    moveit_control_node =Node(
            package='plato_moveit_control',
            executable='moveit_control',
            name='moveit_control',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        moveit_control_node,
    ])