from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "plato_ns",
            default_value="plato2",
            description="Namespace for Plato2 hand",
        )
    ]

    # Initialize Arguments
    plato_ns = LaunchConfiguration("plato_ns")

    # Get URDF via xacro
    pkg_name = 'plato2_description'
    pkg_share = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_share, 'urdf/plato2.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()
    robot_description = {"robot_description": robot_description_content}

    # Controller configurations
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("plato2_hardware_interface"),
            "config",
            "plato2_position_controller.yaml",
        ]
    )
    
    ft_sensor_controller = PathJoinSubstitution(
        [
            FindPackageShare("plato2_hardware_interface"),
            "config",
            "ft_sensor_broadcaster_controller.yaml",
        ]
    )

    # Nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers, ft_sensor_controller],
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

    ft_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ft_sensor_broadcaster", "--controller-manager", "/plato2/controller_manager"],
        namespace=plato_ns,
    )

    # Event handlers
    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    delay_ft_sensor_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[ft_sensor_broadcaster_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner,
        delay_ft_sensor_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)