# aristo_bringup.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    # --- Args ---
    gui = LaunchConfiguration("gui")
    plato_ns = LaunchConfiguration("plato_ns")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declared_arguments = [
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Start RViz2 automatically."
        ),
        DeclareLaunchArgument(
            "plato_ns", default_value="plato2",
            description="Namespace for Aristo hand."
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use simulated clock if true."
        ),
    ]

    # --- URDF via xacro ---
    pkg_share = get_package_share_directory("plato_description")
    xacro_file = os.path.join(pkg_share, "urdf", "aristo_mock_hardware.urdf.xacro")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("plato_description"), "rviz", "plato2.rviz"]
    )

    robot_description_xml = xacro.process_file(xacro_file).toxml()
    robot_description = {"robot_description": robot_description_xml}

    # --- Controllers YAML (position/impedance config) ---
    position_ctrl_yaml = PathJoinSubstitution([
        FindPackageShare("aristo_bringup"),
        "config",
        "plato2_joint_impedance_controller.yaml",
    ])
    controller_manager_path = PathJoinSubstitution(["/", plato_ns, "controller_manager"])

    # --- Nodes ---
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, position_ctrl_yaml, {"use_sim_time": use_sim_time}],
        output="both",
        namespace=plato_ns,
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="both",
        namespace=plato_ns,
        # If your joint_states are NOT namespaced, uncomment the next line:
        # remappings=[("joint_states", "/joint_states")],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(gui),
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "plato2_joint_state_broadcaster",
            "--controller-manager", controller_manager_path,
        ],
        namespace=plato_ns,
        output="screen",
    )

    joint_impedance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_impedance_controller",
            "--controller-manager", controller_manager_path,
        ],
        namespace=plato_ns,
        output="screen",
    )

    # --- Ordering: after JS broadcaster spawns, bring up RViz and then main controller ---
    start_rviz_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz],
        )
    )

    start_impedance_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_impedance_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub,
        joint_state_broadcaster_spawner,
        start_impedance_after_jsb,
        start_rviz_after_jsb,
    ]

    return LaunchDescription(declared_arguments + nodes)
