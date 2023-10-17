import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch_ros.descriptions
import launch.substitutions
import launch_ros
import launch_ros.substitutions

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_package_path = get_package_share_directory('optimo_description')
    urdf_file_path = os.path.join(urdf_package_path, 'urdf', 'plato.urdf.xacro')
    urdf_file_sub = Command(['xacro ', urdf_file_path])


    optimo_description_package = launch_ros.substitutions.FindPackageShare(package='optimo_description').find('optimo_description')

    rviz_config_file = get_package_share_directory('optimo_description') + '/rviz/plato.rviz'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),


        LogInfo(msg=[
            "Loading URDF from '", urdf_file_path, "' as Xacro model..."
        ]),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': launch_ros.descriptions.ParameterValue( launch.substitutions.Command(['xacro ',os.path.join(optimo_description_package,'urdf/plato.urdf.xacro')]), value_type=str)  
            }]
          
        ),

        # Joint state publisher
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': use_sim_time,
        #     }]
        # ),

        # Joint state publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        ),
        
        # Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
