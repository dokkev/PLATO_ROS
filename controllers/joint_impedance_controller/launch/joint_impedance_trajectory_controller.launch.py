#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('joint_impedance_controller')
    estimator_pkg_dir = get_package_share_directory('plato_state_estimator')
    tactile_pkg_dir = get_package_share_directory('tactile_sensing')

    default_controller_params = os.path.join(pkg_dir, 'config', 'impedance_preset.yaml')
    default_estimator_params = os.path.join(estimator_pkg_dir, 'config', 'object_state_estimator.yaml')

    controller_params_arg = DeclareLaunchArgument(
        'controller_params_file',
        default_value=default_controller_params,
        description='Path to impedance preset YAML'
    )

    estimator_params_arg = DeclareLaunchArgument(
        'estimator_params_file',
        default_value=default_estimator_params,
        description='Path to object state estimator configuration YAML'
    )

    controller_node = Node(
        package='joint_impedance_controller',
        executable='impedance_trajectory_controller_node',
        name='impedance_trajectory_controller_node',
        output='screen',
        parameters=[{
            'impedance_preset_yaml_path': LaunchConfiguration('controller_params_file'),
        }],
    )

    object_state_estimator_node = Node(
        package='plato_state_estimator',
        executable='object_state_estimator_node',
        name='object_state_estimator',
        output='screen',
        parameters=[LaunchConfiguration('estimator_params_file')],
    )

    impedance_keyboard_node = Node(
        package='plato_teleop',
        executable='impedance_gain_keyboard',
        name='impedance_gain_keyboard',
        output='screen',
        emulate_tty=True,
    )

    tactile_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(tactile_pkg_dir, 'launch', 'two_naritouchs.xml')
        )
    )

    return LaunchDescription([
        tactile_launch,
        controller_params_arg,
        estimator_params_arg,
        controller_node,
        object_state_estimator_node,
        # impedance_keyboard_node,
    ])
