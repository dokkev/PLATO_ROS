from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "urdf_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("plato_description"),
                "urdf",
                "generated",
                "aristo.urdf",
            ]),
            description="Absolute URDF path for the Pinocchio model.",
        ),
        Node(
            package="aristo_grasp_controller",
            executable="aristo_grasp_node",
            name="aristo_grasp_node",
            output="screen",
            parameters=[{
                "urdf_path": LaunchConfiguration("urdf_path"),
            }],
        ),
    ])
