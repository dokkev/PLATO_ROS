from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    plato_ns = LaunchConfiguration("plato_ns")
    robot_description_xacro_path = LaunchConfiguration(
        "robot_description_xacro_path"
    )
    rviz_config_path = LaunchConfiguration("rviz_config_path")

    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start joint_state_publisher_gui instead of joint_state_publisher.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated clock if true.",
        ),
        DeclareLaunchArgument(
            "plato_ns",
            default_value="plato2",
            description="Namespace for the Aristo hand visualization nodes.",
        ),
        DeclareLaunchArgument(
            "robot_description_xacro_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("plato_description"), "urdf", "aristo.urdf.xacro"]
            ),
            description="Absolute path to the Aristo robot description xacro.",
        ),
        DeclareLaunchArgument(
            "rviz_config_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("plato_description"), "rviz", "plato2.rviz"]
            ),
            description="Absolute path to the RViz config file.",
        ),
    ]

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_xacro_path,
            " ",
            "fake_hardware:=true",
            " ",
            "zeroing:=false",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        namespace=plato_ns,
        condition=IfCondition(gui),
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        namespace=plato_ns,
        condition=UnlessCondition(gui),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=plato_ns,
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
            {"publish_rate": 100.0},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription(
        declared_arguments
        + [
            joint_state_publisher_gui,
            joint_state_publisher,
            robot_state_publisher,
            rviz,
        ]
    )
