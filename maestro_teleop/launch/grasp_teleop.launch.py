from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    source_topic = LaunchConfiguration("source_topic")
    target_topic = LaunchConfiguration("target_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "source_topic",
            default_value="/plato2/fingertip_dist_cmd",
            description="Input Float64MultiArray: [t-i_dis, t-m_dis, T_IP, I_PIP, M_PIP].",
        ),
        DeclareLaunchArgument(
            "target_topic",
            default_value="/plato2/parallel_grasp_controller/teleop_commands",
            description="Output Float64MultiArray: [t-i_dis, I_PIP].",
        ),
        Node(
            package="maestro_teleop",
            executable="fingertip_dist_to_parallel_grasp",
            name="fingertip_dist_to_parallel_grasp",
            output="screen",
            parameters=[{
                "source_topic": source_topic,
                "target_topic": target_topic,
            }],
        ),
    ])
