from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch SpaceMouse State Machine node (reads SpaceMouse hardware directly)."""

    state_machine_node = Node(
        package="plato_teleop",
        executable="spacemouse_state_machine",
        name="spacemouse_state_machine",
        output="screen",
        parameters=[
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([state_machine_node])
