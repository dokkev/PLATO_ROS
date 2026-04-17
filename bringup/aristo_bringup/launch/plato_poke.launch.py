# plato_poke.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    publish_rate = LaunchConfiguration("publish_rate")
    default_stiffness = LaunchConfiguration("default_stiffness")
    default_damping = LaunchConfiguration("default_damping")

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
        parameters=[{
            "publish_rate": publish_rate,
            "default_stiffness": default_stiffness,
            "default_damping": default_damping,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "publish_rate",
            default_value="100.0",
            description="Publish rate [Hz] for impedance command stream.",
        ),
        DeclareLaunchArgument(
            "default_stiffness",
            default_value="2.0",
            description="Uniform stiffness command for all Aristo joints.",
        ),
        DeclareLaunchArgument(
            "default_damping",
            default_value="0.1",
            description="Uniform damping command for all Aristo joints.",
        ),
        aristo_hardware_launch,
        joint_impedance_publisher,
    ])
