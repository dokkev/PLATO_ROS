#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('joint_position_controller')
    default_params = os.path.join(pkg_dir, 'config', 'impedance_presets.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to impedance presets YAML'
    )

    controller_node = Node(
        package='joint_position_controller',
        executable='joint_position_controller_node',
        name='joint_position_controller_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_arg,
        controller_node,
    ])
