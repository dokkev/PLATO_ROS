# plato2_bringup.launch.py
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
            description="Namespace for Plato2 hand."
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use simulated clock if true."
        ),
    ]

    # --- URDF via xacro ---
    pkg_share = get_package_share_directory("plato2_description")
    xacro_file = os.path.join(pkg_share, "urdf", "aristo.urdf.xacro")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("plato2_description"), "rviz", "plato2.rviz"]
    )

    robot_description_xml = xacro.process_file(xacro_file).toxml()
    robot_description = {"robot_description": robot_description_xml}

    # --- Controllers YAML (position/impedance config) ---
    position_ctrl_yaml = PathJoinSubstitution([
        FindPackageShare("plato2_bringup"),
        "config",
        "plato2_joint_impedance_controller.yaml",
    ])

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
        parameters=[robot_description, {"use_sim_time": use_sim_time}, {"publish_rate": 200.0}],
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

    # TF Merger: republish optimo TF to global TF tree
    tf_merger = Node(
        package="plato2_bringup",
        executable="tf_merger.py",
        name="tf_merger",
        output="screen",
    )

    # Index fingertip pose publisher
    index_fingertip_pose_pub = Node(
        package="plato2_bringup",
        executable="index_fingertip_pose_publisher.py",
        name="index_fingertip_pose_publisher",
        output="screen",
    )

    # Static transform from optimo ee to plato2 base_link
    static_tf_optimo_to_plato = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_optimo_to_plato",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0.0",
            "--pitch", "-1.5708",
            "--yaw", "3.14159",
            "--frame-id", "ee",
            "--child-frame-id", "base_link",
        ],
        output="screen",
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "plato2_joint_state_broadcaster",
            "--controller-manager", "/plato2/controller_manager",
        ],
        namespace=plato_ns,
        output="screen",
    )

    joint_impedance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_impedance_controller",
            "--controller-manager", "/plato2/controller_manager",
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
        static_tf_optimo_to_plato,
        joint_state_broadcaster_spawner,
        start_impedance_after_jsb,
        start_rviz_after_jsb,
        tf_merger,
        index_fingertip_pose_pub,
    ]

    return LaunchDescription(declared_arguments + nodes)
