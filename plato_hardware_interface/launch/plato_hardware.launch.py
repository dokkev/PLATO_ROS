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
            default_value="plato",
            description="Namespace for Plato hand",
        )
    )


    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    plato_ns = LaunchConfiguration("plato_ns")

    # Get URDF via xacro
    pkg_name = 'plato_description'
    pkg_share= get_package_share_directory(pkg_name)
    urdf_path = 'urdf/plato_hand.urdf.xacro'
    rviz_config_file = pkg_share + '/rviz/plato.rviz'
    xacro_file = os.path.join(pkg_share, urdf_path)

    robot_description_content = xacro.process_file(xacro_file).toxml()
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("plato_bringup"),
            "config",
            "plato_hand_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("plato_description"), "rviz", "plato.rviz"]
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["plato_joint_state_broadcaster", "--controller-manager", "/plato/controller_manager"],
        namespace=plato_ns,  
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["plato_joint_controller", "--controller-manager", "/plato/controller_manager"],
        namespace=plato_ns,  
    )

    optimo_plato_transform_broadcaster = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_broadcaster',
            arguments=['0', '0', '0.157', '0.707388', '0.0005629', '0.706825', '0.0005633', 'link7_passive', 'plato_base_link'],
        )  

    # Event handlers remain unchanged
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


    rqt_joint_trajectory_controller = Node(
        package="rqt_joint_trajectory_controller",
        executable="rqt_joint_trajectory_controller",
        name="rqt_joint_trajectory_controller",
        output="screen",
        namespace=plato_ns,  
    )


    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        # delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        optimo_plato_transform_broadcaster,
        rqt_joint_trajectory_controller
    ]

    return LaunchDescription(declared_arguments + nodes)
