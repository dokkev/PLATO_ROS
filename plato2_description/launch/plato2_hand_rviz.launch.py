import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition,UnlessCondition
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
import xacro


def generate_launch_description():

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')
    plato_ns = LaunchConfiguration('plato_ns')

    # getting the package path
    pkg_name = 'plato2_description'
    pkg_share= get_package_share_directory(pkg_name)

    # URDF file path
    urdf_path = 'urdf/aristo.urdf.xacro'

    # RVIZ config file path
    rviz_config_file = pkg_share + '/rviz/plato2.rviz'
    
    # extracting the robot deffinition from the xacro file
    xacro_file = os.path.join(pkg_share, urdf_path)
    robot_description_content = xacro.process_file(xacro_file).toxml()



    # Run the nodes
    return LaunchDescription([

        ##################### Arguments #####################
        
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
            default_value='plato2',
            description='Namespace for Plato2 hand.'
        ),


        ##################### Nodes #####################

        # Joint state publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace=plato_ns,
            condition=IfCondition(use_gui),
        ),

        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=plato_ns,
            condition=UnlessCondition(use_gui),
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content, 'use_sim_time': use_sim_time}],
            namespace=plato_ns,
        ),
        
        # Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=plato_ns,
            arguments= ['-d', rviz_config_file]
        ),

    ])