from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    plato_ns = LaunchConfiguration("plato_ns")
    use_sim_time = LaunchConfiguration("use_sim_time")
    fake_hardware = LaunchConfiguration("fake_hardware")
    zeroing = LaunchConfiguration("zeroing")
    robot_description_xacro_path = LaunchConfiguration("robot_description_xacro_path")
    controller_config_path = LaunchConfiguration("controller_config_path")
    rviz_config_path = LaunchConfiguration("rviz_config_path")
    controller_manager_name = LaunchConfiguration("controller_manager_name")
    joint_state_broadcaster_name = LaunchConfiguration("joint_state_broadcaster_name")
    aristo_controller_name = LaunchConfiguration("aristo_controller_name")
    actuator_config_yaml_path = LaunchConfiguration("actuator_config_yaml_path")
    publish_world_pose = LaunchConfiguration("publish_world_pose")
    world_frame = LaunchConfiguration("world_frame")

    declared_arguments = [
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2 automatically."),
        DeclareLaunchArgument("plato_ns", default_value="plato2", description="Namespace for Aristo hand."),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulated clock if true."),
        DeclareLaunchArgument(
            "fake_hardware",
            default_value="false",
            description="Use ros2_control mock hardware instead of the Aristo hardware interface.",
        ),
        DeclareLaunchArgument(
            "zeroing",
            default_value="false",
            description="Run one-shot embedded actuator zeroing during activation.",
        ),
        DeclareLaunchArgument(
            "robot_description_xacro_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("plato_description"), "urdf", "aristo.urdf.xacro"]
            ),
            description="Absolute path to the Aristo robot description xacro.",
        ),
        DeclareLaunchArgument(
            "controller_config_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("aristo_bringup"), "config", "plato_ros_controllers.yaml"]
            ),
            description="Absolute path to the ros2_control controller manager parameters YAML.",
        ),
        DeclareLaunchArgument(
            "rviz_config_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("plato_description"), "rviz", "plato2.rviz"]
            ),
            description="Absolute path to the RViz config file.",
        ),
        DeclareLaunchArgument(
            "controller_manager_name",
            default_value="controller_manager",
            description="Controller manager node name inside the namespace.",
        ),
        DeclareLaunchArgument(
            "joint_state_broadcaster_name",
            default_value="plato2_joint_state_broadcaster",
            description="Joint state broadcaster controller name.",
        ),
        DeclareLaunchArgument(
            "aristo_controller_name",
            default_value="aristo_controller",
            description="Aristo controller name.",
        ),
        DeclareLaunchArgument(
            "actuator_config_yaml_path",
            default_value="",
            description=(
                "Optional override path for the Aristo actuator config YAML. "
                "Leave empty to use the hardware package default."
            ),
        ),
        DeclareLaunchArgument(
            "publish_world_pose",
            default_value="false",
            description="Publish index fingertip pose in world_frame when that TF tree is available.",
        ),
        DeclareLaunchArgument(
            "world_frame",
            default_value="stand",
            description="World frame for optional index fingertip pose publishing.",
        ),
    ]

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_xacro_path,
            " ",
            "fake_hardware:=",
            fake_hardware,
            " ",
            "zeroing:=",
            zeroing,
            " ",
            "actuator_config_yaml_path:=",
            actuator_config_yaml_path,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    controller_manager_path = PathJoinSubstitution(["/", plato_ns, controller_manager_name])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config_path, {"use_sim_time": use_sim_time}],
        output="both",
        namespace=plato_ns,
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}, {"publish_rate": 100.0}],
        output="both",
        namespace=plato_ns,
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(gui),
    )

    tf_merger = Node(
        package="aristo_bringup",
        executable="tf_merger.py",
        name="tf_merger",
        output="screen",
    )

    index_fingertip_pose_pub = Node(
        package="aristo_bringup",
        executable="index_fingertip_pose_publisher.py",
        name="index_fingertip_pose_publisher",
        parameters=[
            {
                "publish_world_pose": ParameterValue(publish_world_pose, value_type=bool),
                "world_frame": world_frame,
            }
        ],
        output="screen",
    )

    static_tf_optimo_to_plato = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_optimo_to_plato",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0.0",
            "--pitch", "-1.5708",
            "--yaw", "3.14159",
            "--frame-id", "ee",
            "--child-frame-id", "base_link",
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[joint_state_broadcaster_name, "--controller-manager", controller_manager_path],
        namespace=plato_ns,
        output="screen",
    )

    aristo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            aristo_controller_name,
            "--controller-manager",
            controller_manager_path,
            "--service-call-timeout",
            "60.0",
        ],
        namespace=plato_ns,
        output="screen",
    )

    start_rviz_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[rviz])
    )
    start_aristo_controller_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[aristo_controller_spawner],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_state_pub,
            static_tf_optimo_to_plato,
            tf_merger,
            index_fingertip_pose_pub,
            joint_state_broadcaster_spawner,
            start_aristo_controller_after_jsb,
            start_rviz_after_jsb,
        ]
    )
