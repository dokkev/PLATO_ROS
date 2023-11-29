import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder



def generate_launch_description():

    ###### Include #####
    plato_manipulator_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('plato_gazebo'), 'launch'), '/plato_manipulator_gazebo.launch.py']), 
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('plato_moveit_config'), 'launch'), '/move_group.launch.py']))
    
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('plato_moveit_config'), 'launch'), '/moveit_rviz.launch.py']))


    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit2_tutorials")
        + "/launch/moveit_cpp_tutorial.rviz"
    )
    moveit_config = MoveItConfigsBuilder("plato", package_name="plato_moveit_config").to_moveit_configs()
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )
    
    return LaunchDescription([
        plato_manipulator_gazebo_launch,
        move_group,
        moveit_rviz,
        # rviz_node
    ])