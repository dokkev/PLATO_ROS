#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    joint4_velocity_arg = DeclareLaunchArgument(
        'joint4_velocity',
        default_value='0.1',
        description='Joint 4 velocity in rad/s'
    )
    
    force_threshold_arg = DeclareLaunchArgument(
        'force_threshold',
        default_value='2.0',
        description='Force threshold for collision detection in N'
    )
    
    torque_threshold_arg = DeclareLaunchArgument(
        'torque_threshold',
        default_value='0.1',
        description='Torque threshold for collision detection in Nm'
    )
    
    joint4_min_arg = DeclareLaunchArgument(
        'joint4_min',
        default_value='-1.5',
        description='Minimum joint 4 position in rad'
    )
    
    joint4_max_arg = DeclareLaunchArgument(
        'joint4_max',
        default_value='1.5',
        description='Maximum joint 4 position in rad'
    )
    
    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='100.0',
        description='Control loop rate in Hz'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Automatically start movement on launch'
    )
    
    # FT Collision Test Node
    ft_collision_test_node = Node(
        package='aristo_hardware_interface',
        executable='ft_collision_test_node',
        name='ft_collision_test_node',
        output='screen',
        parameters=[{
            'joint4_velocity': LaunchConfiguration('joint4_velocity'),
            'force_threshold': LaunchConfiguration('force_threshold'),
            'torque_threshold': LaunchConfiguration('torque_threshold'),
            'joint4_min': LaunchConfiguration('joint4_min'),
            'joint4_max': LaunchConfiguration('joint4_max'),
            'control_rate': LaunchConfiguration('control_rate'),
            'auto_start': LaunchConfiguration('auto_start'),
        }],
        prefix=['xterm -e'],  # Run in separate terminal for keyboard input
    )
    
    return LaunchDescription([
        joint4_velocity_arg,
        force_threshold_arg,
        torque_threshold_arg,
        joint4_min_arg,
        joint4_max_arg,
        control_rate_arg,
        auto_start_arg,
        ft_collision_test_node,
    ])
