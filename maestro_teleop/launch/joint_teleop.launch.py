import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    jpc_pkg_dir = get_package_share_directory("joint_impedance_controller")

    source_topic = LaunchConfiguration("source_topic")
    target_topic = LaunchConfiguration("target_topic")
    expected_size = LaunchConfiguration("expected_size")

    controller_params_file = LaunchConfiguration("controller_params_file")
    hand_namespace = LaunchConfiguration("hand_namespace")

    controller_params_default = os.path.join(
        jpc_pkg_dir, "config", "impedance_preset.yaml")

    joint_impedance_trajectory_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                jpc_pkg_dir,
                "launch",
                "joint_impedance_trajectory_controller.launch.py",
            )
        ),
        launch_arguments={
            "controller_params_file": controller_params_file,
            "hand_namespace": hand_namespace,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "controller_params_file",
            default_value=controller_params_default,
            description="Path to impedance preset YAML.",
        ),
        DeclareLaunchArgument(
            "hand_namespace",
            default_value="plato2",
            description="Namespace used to derive default hand topics for trajectory control.",
        ),
        DeclareLaunchArgument(
            "source_topic",
            default_value="/plato2/joint_impedance_controller/commands_float8array_HY",
            description="Input Float64MultiArray joint position command.",
        ),
        DeclareLaunchArgument(
            "target_topic",
            default_value="/plato2/joint_impedance_trajectory_controller/commands",
            description="Output Float64MultiArray consumed by the LPF trajectory controller.",
        ),
        DeclareLaunchArgument(
            "expected_size",
            default_value="8",
            description="Expected input array length; set <= 0 to disable length checking.",
        ),
        joint_impedance_trajectory_controller_launch,
        Node(
            package="maestro_teleop",
            executable="float8array_to_impedance_trajectory",
            name="float8array_to_impedance_trajectory",
            output="screen",
            parameters=[{
                "source_topic": source_topic,
                "target_topic": target_topic,
                "expected_size": ParameterValue(expected_size, value_type=int),
            }],
        ),
    ])
