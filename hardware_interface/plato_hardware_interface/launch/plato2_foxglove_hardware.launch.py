from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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

    foxglove_websocket = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        parameters=[{"port": 8765}],
        namespace=plato_ns,
    )

    foxglove_graph = Node(
        package="plato_foxglove",
        executable="foxglove_graph.py",
        namespace=plato_ns,
    )

    foxglove_commands = Node(
        package="plato_foxglove",
        executable="foxglove_commands.py",
        namespace=plato_ns,
    )

    foxglove_trajectory = Node(
        package="plato_foxglove",
        executable="foxglove_trajectory.py",
        namespace=plato_ns,
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

    optimo_plato_transform_broadcaster = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_broadcaster",
        arguments=[
            "0",
            "0",
            "0.157",
            "0.707388",
            "0.0005629",
            "0.706825",
            "0.0005633",
            "link7_passive",
            "plato2/base_link",
        ],
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    declared_arguments = [
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
        foxglove_websocket,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        foxglove_graph,
        foxglove_commands,
        foxglove_trajectory,
        optimo_plato_transform_broadcaster,
    ]

    return LaunchDescription(declared_arguments + nodes)
