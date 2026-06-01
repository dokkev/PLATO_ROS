from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    source_topic = LaunchConfiguration("source_topic")
    target_topic = LaunchConfiguration("target_topic")
    expected_size = LaunchConfiguration("expected_size")

    return LaunchDescription([
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
