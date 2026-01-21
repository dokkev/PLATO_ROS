#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    use_monitor = DeclareLaunchArgument(
        'use_monitor', default_value='true',
        description='Run FT sensor monitor node'
    )
    use_collision = DeclareLaunchArgument(
        'use_collision', default_value='true',
        description='Run collision test node'
    )
    auto_start = DeclareLaunchArgument(
        'auto_start', default_value='false',
        description='Auto start collision node movement'
    )
    joint4_velocity = DeclareLaunchArgument(
        'joint4_velocity', default_value='0.1',
        description='Joint 4 velocity for collision node'
    )
    force_threshold = DeclareLaunchArgument(
        'force_threshold', default_value='2.0',
        description='Force threshold for collision node'
    )
    torque_threshold = DeclareLaunchArgument(
        'torque_threshold', default_value='0.1',
        description='Torque threshold for collision node'
    )

    # Paths
    bringup_pkg = FindPackageShare('plato2_bringup')
    hw_launch = PathJoinSubstitution([bringup_pkg, 'launch', 'aristo_hardware.launch.py'])

    hw_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hw_launch)
    )

    monitor_node = Node(
        condition=IfCondition(LaunchConfiguration('use_monitor')),
        package='plato2_hardware_interface',
        executable='ft_sensor_monitor_node',
        name='ft_sensor_monitor_node',
        output='screen',
        parameters=[{
            'topics': [
                '/plato2/ft_sensor_broadcaster_1/wrench',
                '/plato2/ft_sensor_broadcaster_2/wrench',
                '/plato2/ft_sensor_broadcaster_3/wrench'
            ],
            'print_rate_hz': 20.0,
        }],
        prefix=['xterm', '-e'],
    )

    collision_node = Node(
        condition=IfCondition(LaunchConfiguration('use_collision')),
        package='plato2_hardware_interface',
        executable='ft_collision_test_node',
        name='ft_collision_test_node',
        output='screen',
        parameters=[{
            'joint4_velocity': LaunchConfiguration('joint4_velocity'),
            'force_threshold': LaunchConfiguration('force_threshold'),
            'torque_threshold': LaunchConfiguration('torque_threshold'),
            'auto_start': LaunchConfiguration('auto_start'),
        }],
        prefix=['xterm', '-e'],
    )

    ld = LaunchDescription([
        use_monitor,
        use_collision,
        auto_start,
        joint4_velocity,
        force_threshold,
        torque_threshold,
        hw_include,
        monitor_node,
        collision_node,
    ])

    return ld
