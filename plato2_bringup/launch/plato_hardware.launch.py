from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    plato_ns = LaunchConfiguration("plato_ns")
    use_sim_time = LaunchConfiguration("use_sim_time")
    zeroing = LaunchConfiguration("zeroing")
    direct_tx_inter_frame_gap_us = LaunchConfiguration("direct_tx_inter_frame_gap_us")

    declared_arguments = [
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2 automatically."),
        DeclareLaunchArgument("plato_ns", default_value="plato2", description="Namespace for Plato hand."),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulated clock if true."),
        DeclareLaunchArgument(
            "zeroing",
            default_value="false",
            description="Set current actuator positions as software zero after the first full feedback snapshot.",
        ),
        DeclareLaunchArgument(
            "direct_tx_inter_frame_gap_us",
            default_value="100",
            description="Minimum delay between consecutive CAN TX frames in microseconds.",
        ),
    ]

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([FindPackageShare("plato2_description"), "urdf", "plato.urdf.xacro"]),
            " ",
            "zeroing:=",
            zeroing,
            " ",
            "direct_tx_inter_frame_gap_us:=",
            direct_tx_inter_frame_gap_us,
        ]
    )
    rviz_config = PathJoinSubstitution([FindPackageShare("plato2_description"), "rviz", "plato2.rviz"])

    robot_description = {"robot_description": robot_description_content}

    controller_yaml = PathJoinSubstitution(
        [FindPackageShare("plato2_bringup"), "config", "plato2_joint_impedance_controller.yaml"]
    )
    controller_manager_path = PathJoinSubstitution(["/", plato_ns, "controller_manager"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_yaml, {"use_sim_time": use_sim_time}],
        output="both",
        namespace=plato_ns,
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}, {"publish_rate": 100.0}],
        output="both",
        namespace=plato_ns,
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["plato2_joint_state_broadcaster", "--controller-manager", controller_manager_path],
        namespace=plato_ns,
        output="screen",
    )

    joint_impedance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_impedance_controller", "--controller-manager", controller_manager_path],
        namespace=plato_ns,
        output="screen",
    )

    start_rviz_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[rviz])
    )
    start_impedance_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_impedance_controller_spawner],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_state_pub,
            joint_state_broadcaster_spawner,
            start_impedance_after_jsb,
            start_rviz_after_jsb,
        ]
    )
