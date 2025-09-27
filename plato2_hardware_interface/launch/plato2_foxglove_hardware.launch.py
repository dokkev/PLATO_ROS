from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition,UnlessCondition
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
import xacro


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
            DeclareLaunchArgument(
            "plato_ns",
            default_value="plato2",
            description="Namespace for Plato2 hand",
        )
    )

    # Initialize Arguments
    plato_ns = LaunchConfiguration("plato_ns")

    # Get URDF via xacro
    pkg_name = 'plato2_description'
    pkg_share= get_package_share_directory(pkg_name)
    urdf_path = 'urdf/plato2.urdf.xacro'
    xacro_file = os.path.join(pkg_share, urdf_path)

    robot_description_content = xacro.process_file(xacro_file).toxml()
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("plato2_hardware_interface"),
            "config",
            "plato2_position_controller.yaml",
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
    
    # Foxlove Stuff
    foxglove_websocket = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{'port': 8765}],
        namespace=plato_ns,
    )
    
    foxglove_graph = Node(
        package='plato2_foxglove',
        executable='foxglove_graph.py',
        namespace=plato_ns,
    )
    
    foxglove_commands = Node(
        package='plato2_foxglove',
        executable='foxglove_commands.py',
        namespace=plato_ns,
    )

    foxglove_trajectory = Node(
        package='plato2_foxglove',
        executable='foxglove_trajectory.py',
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
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_broadcaster',
            arguments=['0', '0', '0.157', '0.707388', '0.0005629', '0.706825', '0.0005633', 'link7_passive', 'plato2/hand_base'],
        )  

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    
    position_control_node = Node(
        package='joint_position_controller',  
        executable='position_control_node',
        name='position_control_node',
        namespace=plato_ns,  # Use the same namespace as other nodes
        output='screen'
    )
    
    delay_position_control_node_after_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[position_control_node],
        )
    )
    
    ft_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ft_sensor_broadcaster", "--controller-manager", "/plato2/controller_manager"],
        namespace=plato_ns,  
    )


    nodes = [
        foxglove_websocket,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        # ft_sensor_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        foxglove_graph,
        foxglove_commands,
        foxglove_trajectory,
        # delay_position_control_node_after_controller_spawner
        optimo_plato_transform_broadcaster,
    ]

    return LaunchDescription(declared_arguments + nodes)
