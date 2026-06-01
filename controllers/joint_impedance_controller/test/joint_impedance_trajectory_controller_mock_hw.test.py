import os
import time
import unittest

import launch
import launch.actions
import launch.launch_description_sources
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

try:
    from plato_interfaces.msg import ImpedanceCommands
    _import_error = None
except ModuleNotFoundError as exc:
    ImpedanceCommands = None
    _import_error = exc


@pytest.mark.launch_test
def generate_test_description():
    bringup_share = get_package_share_directory("aristo_bringup")
    bringup_launch_file = os.path.join(bringup_share, "launch", "aristo_hardware.launch.py")

    mock_bringup = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(bringup_launch_file),
        launch_arguments={
            "fake_hardware": "true",
            "gui": "false",
            "plato_ns": "plato2",
            "use_sim_time": "false",
        }.items(),
    )

    position_controller_node = launch_ros.actions.Node(
        package="joint_impedance_controller",
        executable="impedance_trajectory_controller_node",
        name="impedance_trajectory_controller_node",
        output="screen",
        parameters=[{
            "control_rate_hz": 100.0,
            "position_filter_alpha": 0.1,
            "joint_state_topic": "/plato2/joint_states",
            "position_command_topic": "/plato2/joint_impedance_trajectory_controller/commands",
            "impedance_command_topic": "/plato2/joint_impedance_controller/commands",
        }],
    )

    return (
        launch.LaunchDescription([
            mock_bringup,
            position_controller_node,
            launch.actions.TimerAction(
                period=5.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {"position_controller_node": position_controller_node},
    )


class TestImpedanceTrajectoryControllerWithMockHardware(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if _import_error is not None:
            raise RuntimeError(
                "Failed to import 'plato_interfaces.msg.ImpedanceCommands'. "
                "Build and source the workspace before running this launch test."
            ) from _import_error
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("joint_impedance_trajectory_controller_mock_hw_test")
        self.joint_state_count = 0
        self.latest_impedance = None

        self.joint_state_sub = self.node.create_subscription(
            JointState,
            "/plato2/joint_states",
            self._on_joint_state,
            10,
        )
        self.impedance_sub = self.node.create_subscription(
            ImpedanceCommands,
            "/plato2/joint_impedance_controller/commands",
            self._on_impedance,
            10,
        )
        self.command_pub = self.node.create_publisher(
            Float64MultiArray,
            "/plato2/joint_impedance_trajectory_controller/commands",
            10,
        )

    def tearDown(self):
        self.node.destroy_subscription(self.joint_state_sub)
        self.node.destroy_subscription(self.impedance_sub)
        self.node.destroy_publisher(self.command_pub)
        self.node.destroy_node()

    def _on_joint_state(self, _msg):
        self.joint_state_count += 1

    def _on_impedance(self, msg):
        self.latest_impedance = msg

    def _spin_until(self, predicate, timeout_sec):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def test_joint_state_stream_exists(self):
        ok = self._spin_until(lambda: self.joint_state_count > 0, timeout_sec=20.0)
        self.assertTrue(ok, "No /plato2/joint_states messages received from mock hardware")

    def test_position_command_produces_impedance_output(self):
        has_joint_state = self._spin_until(lambda: self.joint_state_count > 0, timeout_sec=20.0)
        self.assertTrue(has_joint_state, "joint state stream did not start")

        target = [0.20] * 8
        cmd_msg = Float64MultiArray()
        cmd_msg.data = target

        publish_deadline = time.time() + 1.0
        while time.time() < publish_deadline:
            self.command_pub.publish(cmd_msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        got_impedance = self._spin_until(lambda: self.latest_impedance is not None, timeout_sec=5.0)
        self.assertTrue(got_impedance, "No impedance output after position command")

        self.assertEqual(len(self.latest_impedance.position), 8)
        for i, expected in enumerate(target):
            self.assertAlmostEqual(
                self.latest_impedance.position[i],
                expected,
                delta=0.06,
                msg=f"joint {i} did not converge near commanded target",
            )


@launch_testing.post_shutdown_test()
class TestProcessesExit(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
