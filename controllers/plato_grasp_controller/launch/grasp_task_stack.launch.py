#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_trajectory = LaunchConfiguration("use_trajectory")
    joint_state_topic = LaunchConfiguration("joint_state_topic")
    motion_state_topic = LaunchConfiguration("motion_state_topic")
    task_topic = LaunchConfiguration("task_topic")
    trajectory_goal_topic = LaunchConfiguration("trajectory_goal_topic")
    impedance_command_topic = LaunchConfiguration("impedance_command_topic")
    control_rate_hz = LaunchConfiguration("control_rate_hz")
    default_goal_duration_sec = LaunchConfiguration("default_goal_duration_sec")
    task_update_rate_hz = LaunchConfiguration("task_update_rate_hz")
    log_level = LaunchConfiguration("log_level")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_trajectory",
            default_value="false",
            description=(
                "If true, route grasp commands through the impedance trajectory "
                "controller. If false, publish directly to the joint "
                "impedance controller."
            ),
        ),
        DeclareLaunchArgument(
            "joint_state_topic",
            default_value="/plato2/joint_states",
            description="Joint state topic consumed by the grasp controller.",
        ),
        DeclareLaunchArgument(
            "motion_state_topic",
            default_value="/plato2/plato_grasp_controller/motion_state",
            description="Manual motion-state command topic.",
        ),
        DeclareLaunchArgument(
            "task_topic",
            default_value="/plato2/plato_grasp_controller/task",
            description="Task command topic.",
        ),
        DeclareLaunchArgument(
            "trajectory_goal_topic",
            default_value="/plato2/joint_impedance_trajectory_controller/goal_command",
            description="Trajectory goal topic published by the grasp controller.",
        ),
        DeclareLaunchArgument(
            "impedance_command_topic",
            default_value="/plato2/joint_impedance_controller/commands",
            description="Impedance command topic published by the trajectory controller.",
        ),
        DeclareLaunchArgument(
            "control_rate_hz",
            default_value="100.0",
            description="Update rate for the impedance trajectory controller node.",
        ),
        DeclareLaunchArgument(
            "default_goal_duration_sec",
            default_value="1.0",
            description="Default trajectory duration used by the impedance trajectory controller.",
        ),
        DeclareLaunchArgument(
            "task_update_rate_hz",
            default_value="50.0",
            description="Update rate for the plato grasp task runner.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level for the grasp controller node.",
        ),
    ]

    impedance_trajectory_controller_node = Node(
        package="joint_impedance_controller",
        executable="impedance_trajectory_controller_node",
        name="impedance_trajectory_controller_node",
        output="screen",
        condition=IfCondition(use_trajectory),
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {
                "joint_state_topic": joint_state_topic,
                "goal_command_topic": trajectory_goal_topic,
                "impedance_command_topic": impedance_command_topic,
                "control_rate_hz": control_rate_hz,
                "default_goal_duration_sec": default_goal_duration_sec,
            }
        ],
    )

    grasp_controller_node_via_trajectory = Node(
        package="plato_grasp_controller",
        executable="plato_grasp_controller_node",
        name="plato_grasp_controller_node",
        output="screen",
        condition=IfCondition(use_trajectory),
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {
                "joint_state_topic": joint_state_topic,
                "motion_state_topic": motion_state_topic,
                "task_topic": task_topic,
                "trajectory_goal_topic": trajectory_goal_topic,
                "task_update_rate_hz": task_update_rate_hz,
            }
        ],
    )

    grasp_controller_node_direct = Node(
        package="plato_grasp_controller",
        executable="plato_grasp_controller_node",
        name="plato_grasp_controller_node",
        output="screen",
        condition=UnlessCondition(use_trajectory),
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {
                "joint_state_topic": joint_state_topic,
                "motion_state_topic": motion_state_topic,
                "task_topic": task_topic,
                "trajectory_goal_topic": impedance_command_topic,
                "task_update_rate_hz": task_update_rate_hz,
            }
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            impedance_trajectory_controller_node,
            grasp_controller_node_via_trajectory,
            grasp_controller_node_direct,
        ]
    )
