#!/usr/bin/env python3
"""
Launch file for Object State Estimator node.

This node estimates the slip state of a grasped object using two tactile sensors
and calculates the minimal force needed to prevent slip based on the theory from:
"Theoretical Derivation and Realization of Adaptive Grasping Based on
Rotational Incipient Slip Detection" (T. Narita et al., ICRA 2020)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('plato2_state_estimator')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'object_state_estimator.yaml'),
        description='Full path to the configuration file'
    )

    # Create the node
    object_state_estimator_node = Node(
        package='plato2_state_estimator',
        executable='object_state_estimator_node',
        name='object_state_estimator',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )

    return LaunchDescription([
        config_file_arg,
        object_state_estimator_node
    ])
