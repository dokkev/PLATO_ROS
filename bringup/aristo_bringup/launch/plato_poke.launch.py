# plato_poke.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory("aristo_bringup")

    # Include aristo_hardware launch file
    aristo_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "aristo_hardware.launch.py")
        )
    )

    # Joint impedance publisher node
    joint_impedance_publisher = Node(
        package="aristo_bringup",
        executable="joint_impedance_publisher.py",
        name="joint_impedance_publisher",
        output="screen",
    )

    return LaunchDescription([
        aristo_hardware_launch,
        joint_impedance_publisher,
    ])
