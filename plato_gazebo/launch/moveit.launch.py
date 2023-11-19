import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription



def generate_launch_description():

    ###### Include #####
    plato_manipulator_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('plato_gazebo'), 'launch'), '/plato_manipulator.launch.py']), 
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('plato_moveit_config'), 'launch'), '/move_group.launch.py']))
    
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('plato_moveit_config'), 'launch'), '/moveit_rviz.launch.py']))

    
    return LaunchDescription([
        plato_manipulator_gazebo_launch,
        move_group,
        moveit_rviz,
    ])