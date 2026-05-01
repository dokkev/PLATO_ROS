from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch retargeting_converter and the joint position controller."""

    controller_params_file = LaunchConfiguration("controller_params_file")

    controller_params_arg = DeclareLaunchArgument(
        "controller_params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("joint_position_controller"),
            "config",
            "impedance_presets.yaml",
        ]),
        description="Path to impedance presets YAML for joint_position_controller_node.",
    )

    retarget_node = Node(
        package="plato_teleop",
        executable="retargeting_converter",
        name="retargeting_converter",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "publish_rate_hz": 50.0,
            "joint_state_topic": "/plato2/joint_states",
            "preset_joint_indices": [0, 1],
            "preset_joint_positions": [0.0, 0.0],
            "thumb_state_enabled": True,
            "thumb_state_topic": "/plato2/thumb_state",
            "thumb_state_0_positions": [0.785, 0.314],
            "thumb_state_1_positions": [0.0, 0.0],
            "thumb_state_2_positions": [-0.430, -0.450],
            "thumb_state_3_positions": [-0.3, -0.3],
        }],
    )

    position_controller_node = Node(
        package="joint_position_controller",
        executable="joint_position_controller_node",
        name="joint_position_controller_node",
        output="screen",
        parameters=[controller_params_file, {"use_sim_time": False}],
    )

    return LaunchDescription([
        controller_params_arg,
        retarget_node,
        position_controller_node,
    ])
