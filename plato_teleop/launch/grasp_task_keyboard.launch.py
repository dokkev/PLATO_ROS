from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    task_topic_arg = DeclareLaunchArgument(
        'task_topic',
        default_value='/plato2/plato_grasp_controller/task',
        description='Topic that receives grasp task names as std_msgs/String',
    )

    keyboard_node = Node(
        package='plato_teleop',
        executable='grasp_task_keyboard',
        name='grasp_task_keyboard',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'task_topic': LaunchConfiguration('task_topic')},
            {
                'bindings': [
                    '0:idle',
                    '1:index_pinch_ready',
                    '2:index_pinch',
                    '3:lateral_pinch_ready',
                    '4:lateral_pinch',
                ]
            },
        ],
    )

    return LaunchDescription([task_topic_arg, keyboard_node])
