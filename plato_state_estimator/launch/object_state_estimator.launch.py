#!/usr/bin/env python3
"""
Launch file for the reference grasp force generator.

The executable name is kept temporarily for compatibility, but the node
publishes a scalar normal-force reference and diagnostics rather than a full
object-state estimate.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('plato_state_estimator')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'object_state_estimator.yaml'),
        description='Full path to the reference grasp force generator configuration file'
    )

    # Create the node
    reference_grasp_force_generator_node = Node(
        package='plato_state_estimator',
        executable='object_state_estimator_node',
        name='reference_grasp_force_generator',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )

    return LaunchDescription([
        config_file_arg,
        reference_grasp_force_generator_node
    ])
