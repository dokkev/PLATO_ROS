from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_arguments = [
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("plato_ns", default_value="plato2"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("zeroing", default_value="false"),
    ]

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("plato_bringup"),
                    "launch",
                    "plato_hardware.launch.py",
                ]
            )
        ),
        launch_arguments={
            "rviz": LaunchConfiguration("rviz"),
            "plato_ns": LaunchConfiguration("plato_ns"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "zeroing": LaunchConfiguration("zeroing"),
            "can_backend": "socketcan",
            "socketcan_interface": "can0",
        }.items(),
    )

    return LaunchDescription(launch_arguments + [hardware_launch])
