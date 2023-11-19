import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, Shutdown, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():

    ##### Arguments #####
    use_sim_time = LaunchConfiguration('use_sim_time')

    ##### Parameters #####
    pkg_path = os.path.join(get_package_share_directory('plato_gazebo'))
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path = os.environ['GAZEBO_MODEL_PATH'] + ':' + 'models'+ pkg_path
    else:
        model_path = 'models'+ pkg_path
    world_path = os.path.join(pkg_path, 'worlds', 'plato.world')
    plato_xacro = os.path.join(get_package_share_directory('plato_moveit_config'), 'config', 'plato.urdf.xacro')
    robot_description_content = xacro.process_file(plato_xacro, mappings={"hardware_type": "gazebo"}).toxml()

    ###### Nodes #####
    spawn_entity_node = Node(package='gazebo_ros', executable='spawn_entity.py',
                             arguments=['-topic', 'robot_description',
                                        '-entity', 'plato_manipulator',
                                        '-x', '0.1',
                                        '-y', '0.1',
                                        '-z', '0.01'
                                        ],
                             output='screen')
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': robot_description_content}]

    )

    # spawning the joint broadcaster
    spawn_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_optimo_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["optimo_arm_controller"],
        output="screen",
    )

    spawn_plato_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["plato_hand_controller"],
        output="screen",
    )

    


    ###### Include #####

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_path}.items()
    )

   
    return LaunchDescription([
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        node_robot_state_publisher,
        gazebo_launch,
        spawn_broadcaster,
        spawn_optimo_controller,
        spawn_plato_controller,
        spawn_entity_node,   
  
    ])