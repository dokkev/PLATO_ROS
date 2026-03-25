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
    zeroing = LaunchConfiguration("zeroing")

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([FindPackageShare("plato_description"), "urdf", "plato.urdf.xacro"]),
            " ",
            "zeroing:=",
            zeroing,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("plato_bringup"),
            "config",
            "plato2_joint_impedance_controller.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("plato_description"), "rviz", "plato2.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        namespace=plato_ns,
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        namespace=plato_ns,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    contact_estimation_node = Node(
        package="plato_state_estimator",
        executable="contact_estimator",
        name="contact_estimator",
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["plato2_joint_state_broadcaster", "--controller-manager", "/plato2/controller_manager"],
        namespace=plato_ns,
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_impedance_controller", "--controller-manager", "/plato2/controller_manager"],
        namespace=plato_ns,
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        ),
        DeclareLaunchArgument(
            "plato_ns",
            default_value="plato2",
            description="Namespace for Plato2 hand",
        ),
        DeclareLaunchArgument(
            "zeroing",
            default_value="false",
            description="Set current actuator positions as software zero after the first full feedback snapshot.",
        ),
    ]

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        contact_estimation_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
