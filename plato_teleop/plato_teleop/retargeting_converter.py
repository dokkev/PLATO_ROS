#!/usr/bin/env python3
"""Retargeting converter with low-pass filtering.

Subscribes to a Float64MultiArray command stream and republishes a smoothed
version to the joint position controller topic. Filtering uses an exponential
moving average: y = alpha * y_prev + (1 - alpha) * x, with alpha in [0, 1].
Joint indices 0 and 1 are held at preset values, optionally selected by
/plato2/thumb_state, so retargeting commands only control the remaining joints.
"""

from __future__ import annotations

from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32


class RetargetingConverter(Node):
    def __init__(self) -> None:
        super().__init__("retargeting_converter")

        self.filter_alpha: float = self.declare_parameter("filter_alpha", 0.2).value
        self.filter_alpha = min(1.0, max(0.0, float(self.filter_alpha)))

        self.source_topic: str = self.declare_parameter(
            "source_topic", "/plato2/joint_impedance_controller/commands_float8array_HY"
        ).value
        self.target_topic: str = self.declare_parameter(
            "target_topic", "/plato2/joint_position_controller/commands"
        ).value
        self.publish_rate_hz: float = float(
            self.declare_parameter("publish_rate_hz", 50.0).value
        )
        self.joint_state_topic: str = self.declare_parameter(
            "joint_state_topic", "/plato2/joint_states"
        ).value
        self.joint_names: List[str] = [
            str(name)
            for name in self.declare_parameter(
                "joint_names",
                ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"],
            ).value
        ]
        self.output_joint_count: int = int(self.declare_parameter("output_joint_count", 8).value)
        self.preset_joint_indices: List[int] = [
            int(index)
            for index in self.declare_parameter("preset_joint_indices", [0, 1]).value
        ]
        self.preset_joint_positions: List[float] = [
            float(position)
            for position in self.declare_parameter("preset_joint_positions", [0.0, 0.0]).value
        ]
        self.thumb_state_enabled: bool = bool(
            self.declare_parameter("thumb_state_enabled", True).value
        )
        self.thumb_state_topic: str = self.declare_parameter(
            "thumb_state_topic", "/plato2/thumb_state"
        ).value
        self.thumb_state_presets: Dict[int, List[float]] = {
            0: self._get_float_list_parameter("thumb_state_0_positions", [0.785, 0.314]),
            1: self._get_float_list_parameter("thumb_state_1_positions", [0.0, 0.0]),
            2: self._get_float_list_parameter("thumb_state_2_positions", [-0.430, -0.450]),
            3: self._get_float_list_parameter("thumb_state_3_positions", [-0.3, -0.3]),
        }
        self.thumb_state: int | None = None

        if len(self.preset_joint_positions) != len(self.preset_joint_indices):
            self.get_logger().warn(
                "preset_joint_positions length does not match preset_joint_indices; "
                "missing preset values will default to 0.0"
            )

        self._filtered_cmd: List[float] = []
        self._latest_joint_positions: List[float] | None = None

        self.subscription = self.create_subscription(
            Float64MultiArray, self.source_topic, self.input_callback, 10
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )
        self.publisher = self.create_publisher(Float64MultiArray, self.target_topic, 10)
        self.publish_timer = None
        if self.publish_rate_hz > 0.0:
            self.publish_timer = self.create_timer(
                1.0 / self.publish_rate_hz,
                self.publish_current_command,
            )
        if self.thumb_state_enabled:
            self.thumb_state_sub = self.create_subscription(
                Int32,
                self.thumb_state_topic,
                self.thumb_state_callback,
                10,
            )

        self.get_logger().info(
            f"retargeting_converter: source={self.source_topic} -> target={self.target_topic}, "
            f"alpha={self.filter_alpha:.3f}, publish_rate={self.publish_rate_hz:.1f} Hz"
        )
        self.get_logger().info(
            f"retargeting_converter: preset joints={self.preset_joint_indices}, "
            f"positions={self.preset_joint_positions}"
        )
        if self.thumb_state_enabled:
            self.get_logger().info(
                f"retargeting_converter: thumb states enabled on {self.thumb_state_topic}; "
                f"states={self.thumb_state_presets}"
            )

    def thumb_state_callback(self, msg: Int32) -> None:
        state = int(msg.data)
        if state not in self.thumb_state_presets:
            self.get_logger().warn(f"Unknown thumb state {state}; expected one of 0, 1, 2, 3")
            return

        self.thumb_state = state
        self.preset_joint_positions = self.thumb_state_presets[state]
        self.get_logger().info(
            f"Thumb state {state}: joints {self.preset_joint_indices} -> "
            f"{self.preset_joint_positions}"
        )
        if self._ensure_command_initialized():
            self._apply_joint_presets(self._filtered_cmd)
            self.publish_current_command()

    def joint_state_callback(self, msg: JointState) -> None:
        position_by_name = {
            name: position
            for name, position in zip(msg.name, msg.position)
        }
        positions: List[float] = []
        for joint_name in self.joint_names:
            if joint_name not in position_by_name:
                return
            positions.append(float(position_by_name[joint_name]))
        self._latest_joint_positions = positions

    def input_callback(self, msg: Float64MultiArray) -> None:
        if not msg.data:
            return

        command = self._apply_joint_presets(list(msg.data))

        if len(self._filtered_cmd) != len(command):
            # Initialize filter state with first sample or dimension change.
            self._filtered_cmd = command
        else:
            alpha = self.filter_alpha
            beta = 1.0 - alpha
            for i, value in enumerate(command):
                self._filtered_cmd[i] = alpha * self._filtered_cmd[i] + beta * value

        self._apply_joint_presets(self._filtered_cmd)

        self.publish_current_command()

    def publish_current_command(self) -> None:
        if not self._filtered_cmd:
            return

        out = Float64MultiArray()
        out.data = list(self._filtered_cmd)
        self.publisher.publish(out)

    def _ensure_command_initialized(self) -> bool:
        if self._filtered_cmd:
            return True

        if self._latest_joint_positions:
            self._filtered_cmd = list(self._latest_joint_positions)
            return True

        self.get_logger().warn(
            "Thumb state received, but no retargeting command or joint state has been received yet."
        )
        return False

    def _apply_joint_presets(self, command: List[float]) -> List[float]:
        target_len = max(
            len(command),
            self.output_joint_count,
            max(self.preset_joint_indices, default=-1) + 1,
        )
        if len(command) < target_len:
            command.extend([0.0] * (target_len - len(command)))

        for i, joint_index in enumerate(self.preset_joint_indices):
            if joint_index < 0:
                continue
            preset_position = (
                self.preset_joint_positions[i]
                if i < len(self.preset_joint_positions)
                else 0.0
            )
            command[joint_index] = preset_position

        return command

    def _get_float_list_parameter(self, name: str, default: List[float]) -> List[float]:
        return [
            float(value)
            for value in self.declare_parameter(name, default).value
        ]


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RetargetingConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
