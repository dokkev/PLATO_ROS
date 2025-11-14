from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch SpaceMouse Twist Publisher node."""

    spacemouse_twist_node = Node(
        package="plato_teleop",
        executable="spacemouse_twist",
        name="spacemouse_twist_publisher",
        output="screen",
        parameters=[
            {"translation_scale": 0.3},
            {"rotation_scale": 0.3},
            {"translation_threshold": 0.3},
            {"rotation_threshold": 0.3},
            {"publish_rate": 900.0},
            {"twist_topic": "/optimo/servo/twist_cmd"},
            {"frame_id": "world"},
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([spacemouse_twist_node])
