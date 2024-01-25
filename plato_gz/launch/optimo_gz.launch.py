# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    pkg_desc_dir = get_package_share_directory("optimo_description")
    pkg_dir = get_package_share_directory("plato_gz")
    # Add paths for meshes
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = pkg_desc_dir + "/..:" + pkg_desc_dir
    os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] = (
        pkg_dir + "/..:" + pkg_dir + ":" + pkg_dir + "/../../lib/plato_gz"
    )
    os.environ["IGN_GAZEBO_PLUGIN_PATH"] = (
        pkg_dir + "/..:" + pkg_dir + ":" + pkg_dir + "/lib/plato_gz"
    )

    # Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)


    # xacro_file = os.path.join(pkg_desc_dir, "urdf", "optimo.urdf.xacro")
    pkg_name = 'optimo_description'
    urdf_path = 'urdf/optimo.urdf.xacro'
    pkg_share= get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_share, urdf_path)
    robot_description_content = xacro.process_file(xacro_file).toxml()
    params = {"robot_description": robot_description_content}



    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    # params = {"robot_description": doc.toxml()}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    ignition_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            "-name",
            doc
            "optimo",
            "-allow_renaming",
            "true",
        ],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_joint_effort_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "effort_controller",
        ],
        output="screen",
    )
    # Can test if effort control working with : ros2 topic pub /effort_controller/commands
    # std_msgs/msg/Float64MultiArray "{data: [0,100,0,0,0,0,0],layout: {dim:[], data_offset: 1"}}

    # Visualize in RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_desc_dir, "config", "urdf.rviz")],
    )

    optimo_controller =   Node(
         package = "optimo_controller", executable="optimo_controller", output="screen",arguments=["SIMULATED"]
    )

    return LaunchDescription(
        [
            # Launch gazebo environment
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("ros_gz_sim"),
                            "launch",
                            "gz_sim.launch.py",
                        )
                    ]
                ),
                launch_arguments=[("gz_args", [" -r -v 3 empty.sdf"])],
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=ignition_spawn_entity,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_joint_effort_controller],
                )
            ),
            node_robot_state_publisher,
            ignition_spawn_entity,
            # Launch Arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="If true, use simulated clock",
            ),
            # rviz,
            # optimo_controller
        ]
    )
