import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')
    plato_ns = LaunchConfiguration('plato_ns')

    pkg_name = 'plato_description'
    pkg_share = get_package_share_directory(pkg_name)
    urdf_path = 'urdf/plato.urdf.xacro'
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'plato2.rviz')

    xacro_file = os.path.join(pkg_share, urdf_path)
    robot_description_content = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim (Gazebo) clock when True'
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Use joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            'plato_ns',
            default_value='plato',
            description='Namespace for Plato hand.'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace=plato_ns,
            condition=IfCondition(use_gui),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=plato_ns,
            condition=UnlessCondition(use_gui),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content, 'use_sim_time': use_sim_time}],
            namespace=plato_ns,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=plato_ns,
            arguments=['-d', rviz_config_file]
        ),
    ])
