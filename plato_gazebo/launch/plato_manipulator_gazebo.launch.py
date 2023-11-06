import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
import xacro



def generate_launch_description():

   # Check if we're told to use sim time
  

    models_path = os.path.join(get_package_share_directory('plato_gazebo'), 'models')

    # RVIZ config file path
    rviz_config_file = get_package_share_directory('plato_description') + '/rviz/plato.rviz'


    ###### Nodes #####
    spawn_entity_node = Node(package='gazebo_ros', executable='spawn_entity.py',
                             arguments=['-topic', 'robot_description',
                                        '-entity', 'plato_manipulator',
                                        '-x', '0.1',
                                        '-y', '0.1',
                                        '-z', '0.01'
                                        ],
                             output='screen')
    
    # spawning the joint broadcaster
    spawn_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

  
    node_rviz =  Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments= ['-d', rviz_config_file]
    )

    ###### Include #####
    rsp_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('plato_description'),'launch','plato_manipulator_rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )


    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': os.path.join(get_package_share_directory('plato_gazebo'), 'worlds', 'optimo.world')}.items()
    )


    
    return LaunchDescription([
        # SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
        rsp_launch,
        gazebo_launch,
        spawn_broadcaster,
        spawn_controller,
        spawn_entity_node,
        # node_rviz
 
        
  
    ])