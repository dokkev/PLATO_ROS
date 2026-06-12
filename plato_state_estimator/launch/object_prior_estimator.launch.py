#!/usr/bin/env python3
"""Launch the tactile object prior estimator."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    state_estimator_share = get_package_share_directory('plato_state_estimator')
    plato_description_share = get_package_share_directory('plato_description')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            state_estimator_share, 'config', 'object_prior_estimator.yaml'),
        description='Object prior estimator parameter file',
    )
    robot_description_path_arg = DeclareLaunchArgument(
        'robot_description_path',
        default_value=os.path.join(
            plato_description_share, 'urdf', 'generated', 'aristo.urdf'),
        description='URDF path used to build the Pinocchio model',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz with the object-prior estimator display config',
    )
    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            state_estimator_share, 'rviz', 'object_prior_estimator.rviz'),
        description='RViz config file for object-prior estimator visualization',
    )

    object_prior_estimator_node = Node(
        package='plato_state_estimator',
        executable='object_prior_estimator_node',
        name='object_prior_estimator',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'robot_description_path': LaunchConfiguration('robot_description_path')},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='object_prior_estimator_rviz',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        config_file_arg,
        robot_description_path_arg,
        use_rviz_arg,
        rviz_config_file_arg,
        object_prior_estimator_node,
        rviz_node,
    ])
