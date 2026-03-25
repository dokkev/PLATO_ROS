#!/usr/bin/env python3

"""
Launch file for parallel grasp force control node

This launches the force-controlled parallel grasp node with state machine:
  - kIdle: Default position control
  - kContact: Contact detection
  - kAdaptive: Slip-based adaptive force control
  - kForce: User-specified force tracking
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('parallel_grasp_controller')

    # Default config file path
    default_config = os.path.join(pkg_dir, 'config', 'grasp_force_control.yaml')

    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to grasp force control configuration file'
    )

    # Create node
    grasp_force_control_node = Node(
        package='parallel_grasp_controller',
        executable='grasp_node',
        name='grasp_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )

    return LaunchDescription([
        config_arg,
        grasp_force_control_node
    ])
