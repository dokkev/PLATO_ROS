#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    jpc_pkg_dir = get_package_share_directory('joint_impedance_controller')
    estimator_pkg_dir = get_package_share_directory('plato_state_estimator')
    tactile_pkg_dir = get_package_share_directory('tactile_sensing')

    controller_params_default = os.path.join(jpc_pkg_dir, 'config', 'impedance_preset.yaml')
    estimator_params_default = os.path.join(estimator_pkg_dir, 'config', 'object_state_estimator.yaml')

    controller_params_arg = DeclareLaunchArgument(
        'controller_params_file',
        default_value=controller_params_default,
        description='Path to impedance preset YAML for joint position controller'
    )

    estimator_params_arg = DeclareLaunchArgument(
        'estimator_params_file',
        default_value=estimator_params_default,
        description='Path to reference grasp force generator configuration YAML'
    )

    joint_impedance_trajectory_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(jpc_pkg_dir, 'launch', 'joint_impedance_trajectory_controller.launch.py')
        ),
        launch_arguments={
            'controller_params_file': LaunchConfiguration('controller_params_file'),
        }.items(),
    )

    tactile_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(tactile_pkg_dir, 'launch', 'two_naritouchs.xml')
        )
    )

    reference_grasp_force_generator_node = Node(
        package='plato_state_estimator',
        executable='object_state_estimator_node',
        name='reference_grasp_force_generator',
        output='screen',
        parameters=[LaunchConfiguration('estimator_params_file')],
    )

    parallel_grasp_node = Node(
        package='parallel_grasp_controller',
        executable='parallel_grasp_node',
        name='parallel_grasp_node',
        output='screen',
    )

    return LaunchDescription([
        controller_params_arg,
        estimator_params_arg,
        tactile_launch,
        reference_grasp_force_generator_node,
        joint_impedance_trajectory_controller_launch,
        parallel_grasp_node,
    ])
