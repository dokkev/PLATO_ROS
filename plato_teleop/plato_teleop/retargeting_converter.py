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

        self.subscription = self.create_subscription(
            Float64MultiArray, self.source_topic, self.input_callback, 10
        )
        self.publisher = self.create_publisher(Float64MultiArray, self.target_topic, 10)
        if self.thumb_state_enabled:
            self.thumb_state_sub = self.create_subscription(
                Int32,
                self.thumb_state_topic,
                self.thumb_state_callback,
                10,
            )

        self.get_logger().info(
            f"retargeting_converter: source={self.source_topic} -> target={self.target_topic}, "
            f"alpha={self.filter_alpha:.3f}"
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

        out = Float64MultiArray()
        out.data = self._filtered_cmd
        self.publisher.publish(out)

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
