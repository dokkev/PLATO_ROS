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

    # getting the package path
    pkg_name = 'optimo_description'
    pkg_share= get_package_share_directory(pkg_name)

    # URDF file path
    urdf_path = 'urdf/plato.urdf.xacro'

    # RVIZ config file path
    rviz_config_file = pkg_share + '/rviz/plato.rviz'
    
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
            default_value='false',
            description='Use joint_state_publisher_gui'
        ),


        ##################### Nodes #####################

        # Joint state publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
        ),

        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(use_gui),
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content, 'use_sim_time': use_sim_time}]
        ),
        
        # Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments= ['-d', rviz_config_file]
        ),

    ])