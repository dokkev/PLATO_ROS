from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        ),
        DeclareLaunchArgument(
            "plato_ns",
            default_value="plato2",
            description="Namespace for Plato2 hand",
        )
    ]

    # Launch Arguments
    rviz = LaunchConfiguration("rviz")
    plato_ns = LaunchConfiguration("plato_ns")

    # Include the hardware launch file
    pkg_plato2_hardware = FindPackageShare("plato_hardware_interface")
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_plato2_hardware, '/launch/plato2_hardware.launch.py']
        ),
        launch_arguments={'plato_ns': plato_ns, 'rviz': 'false'}.items()
    )

    # RViz configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("plato_description"), "rviz", "plato2.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(declared_arguments + [hardware_launch, rviz_node])
