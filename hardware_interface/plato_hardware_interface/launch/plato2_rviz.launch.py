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
        ),
        DeclareLaunchArgument(
            "can_backend",
            default_value="pcan",
            description="CAN transport backend: pcan or socketcan.",
        ),
        DeclareLaunchArgument(
            "socketcan_interface",
            default_value="can0",
            description="Linux SocketCAN interface used when can_backend is socketcan.",
        ),
    ]

    # Launch Arguments
    rviz = LaunchConfiguration("rviz")
    plato_ns = LaunchConfiguration("plato_ns")
    can_backend = LaunchConfiguration("can_backend")
    socketcan_interface = LaunchConfiguration("socketcan_interface")

    # Include the hardware launch file
    pkg_plato2_hardware = FindPackageShare("plato_hardware_interface")
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_plato2_hardware, '/launch/plato2_hardware.launch.py']
        ),
        launch_arguments={
            'plato_ns': plato_ns,
            'rviz': 'false',
            'can_backend': can_backend,
            'socketcan_interface': socketcan_interface,
        }.items()
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
