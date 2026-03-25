#!/usr/bin/env python3
"""Retargeting converter with low-pass filtering.

Subscribes to a Float64MultiArray command stream and republishes a smoothed
version to the joint position controller topic. Filtering uses an exponential
moving average: y = alpha * y_prev + (1 - alpha) * x, with alpha in [0, 1].
"""

from __future__ import annotations

from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class RetargetingConverter(Node):
    def __init__(self) -> None:
        super().__init__("retargeting_converter")

        self.filter_alpha: float = self.declare_parameter("filter_alpha", 0.2).value
        self.filter_alpha = min(1.0, max(0.0, float(self.filter_alpha)))

        self.source_topic: str = self.declare_parameter(
            "source_topic", "/plato2/joint_impedance_controller/commands_float8array_HY"
        ).value
        self.target_topic: str = self.declare_parameter(
            "target_topic", "/plato2/joint_impedance_trajectory_controller/commands"
        ).value

        self._filtered_cmd: List[float] = []

        self.subscription = self.create_subscription(
            Float64MultiArray, self.source_topic, self.input_callback, 10
        )
        self.publisher = self.create_publisher(Float64MultiArray, self.target_topic, 10)

        self.get_logger().info(
            f"retargeting_converter: source={self.source_topic} -> target={self.target_topic}, "
            f"alpha={self.filter_alpha:.3f}"
        )

    def input_callback(self, msg: Float64MultiArray) -> None:
        if not msg.data:
            return

        if len(self._filtered_cmd) != len(msg.data):
            # Initialize filter state with first sample or dimension change.
            self._filtered_cmd = list(msg.data)
        else:
            alpha = self.filter_alpha
            beta = 1.0 - alpha
            for i, value in enumerate(msg.data):
                self._filtered_cmd[i] = alpha * self._filtered_cmd[i] + beta * value

        out = Float64MultiArray()
        out.data = self._filtered_cmd
        self.publisher.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RetargetingConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
