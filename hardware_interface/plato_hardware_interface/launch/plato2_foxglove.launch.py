from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "plato_ns",
            default_value="plato2",
            description="Namespace for Plato2 hand",
        )
    ]

    # Launch Arguments
    plato_ns = LaunchConfiguration("plato_ns")

    # ✅ Include the base robot control launch file
    pkg_plato2_hardware = FindPackageShare("plato_hardware_interface")
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_plato2_hardware, '/launch/plato2_control.launch.py']
        ),
        launch_arguments={'plato_ns': plato_ns}.items()
    )

    # ✅ Foxglove nodes
    foxglove_websocket = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{'port': 8765}],
        namespace=plato_ns,
    )

    foxglove_graph = Node(
        package='plato_foxglove',
        executable='foxglove_graph.py',
        namespace=plato_ns,
    )

    foxglove_commands = Node(
        package='plato_foxglove',
        executable='foxglove_commands.py',
        namespace=plato_ns,
    )

    foxglove_trajectory = Node(
        package='plato_foxglove',
        executable='foxglove_trajectory.py',
        namespace=plato_ns,
    )

    # ✅ Ensure Foxglove starts only after the base system is running
    foxglove_start_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=base_launch,
            on_exit=[foxglove_websocket, foxglove_graph, foxglove_commands, foxglove_trajectory],
        )
    )

    return LaunchDescription(declared_arguments + [
        base_launch,
        foxglove_start_delay,
    ])
