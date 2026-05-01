#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    jpc_pkg_dir = get_package_share_directory("joint_position_controller")
    controller_params_default = os.path.join(
        jpc_pkg_dir, "config", "impedance_presets.yaml"
    )

    controller_params_arg = DeclareLaunchArgument(
        "controller_params_file",
        default_value=controller_params_default,
        description="Path to impedance presets YAML for joint position controller",
    )

    initial_thumb_state_arg = DeclareLaunchArgument(
        "initial_thumb_state",
        default_value="1",
        description="Initial thumb state for fingertip_dist_node.",
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate_hz",
        default_value="50.0",
        description="Rate for republishing latest fingertip distance target.",
    )

    state_transition_duration_arg = DeclareLaunchArgument(
        "state_transition_duration",
        default_value="0.8",
        description="Duration in seconds for fingertip_dist_node state transitions.",
    )

    position_interpolation_alpha_arg = DeclareLaunchArgument(
        "position_interpolation_alpha",
        default_value="1.0",
        description="Interpolation alpha for joint_position_controller_node.",
    )

    joint_position_controller_node = Node(
        package="joint_position_controller",
        executable="joint_position_controller_node",
        name="joint_position_controller_node",
        output="screen",
        parameters=[
            LaunchConfiguration("controller_params_file"),
            {"interpolation_alpha": LaunchConfiguration("position_interpolation_alpha")},
        ],
    )

    fingertip_dist_node = Node(
        package="parallel_grasp_controller",
        executable="fingertip_dist_node",
        name="fingertip_dist_node",
        output="screen",
        parameters=[{
            "initial_thumb_state": LaunchConfiguration("initial_thumb_state"),
            "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
            "state_transition_duration": LaunchConfiguration("state_transition_duration"),
        }],
    )

    return LaunchDescription([
        controller_params_arg,
        initial_thumb_state_arg,
        publish_rate_arg,
        state_transition_duration_arg,
        position_interpolation_alpha_arg,
        joint_position_controller_node,
        fingertip_dist_node,
    ])
