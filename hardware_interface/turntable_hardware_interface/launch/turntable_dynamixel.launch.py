import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("turntable_hardware_interface")
    default_config_file = os.path.join(pkg_dir, "config", "turntable_dynamixel.yaml")

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_file,
            description="Path to turntable Dynamixel YAML config.",
        ),
        Node(
            package="turntable_hardware_interface",
            executable="turntable_dynamixel_node",
            name="turntable_dynamixel_node",
            output="screen",
            parameters=[{
                "config_file": config_file,
            }],
        ),
    ])
