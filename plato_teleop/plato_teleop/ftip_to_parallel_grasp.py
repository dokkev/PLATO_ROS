#!/usr/bin/env python3
"""Convert fingertip distance/angle signals into parallel grasp commands.

Subscribes to `/plato2/joint_impedance_controller/FTip_DistsAndAngles_HY`
Float64MultiArray:
  [u_d (thumb-index dist), thumb-mid dist, thumb IP, u_phi (index DIP), middle DIP]

Publishes `/plato2/parallel_grasp_controller/commands` Float64MultiArray:
  data[0] = u_d distance (normalized from source[0])
  data[1] = u_phi angle (normalized from source[3])
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


class FtipToParallelGrasp(Node):
    def __init__(self) -> None:
        super().__init__("ftip_to_parallel_grasp")

        self.source_topic: str = self.declare_parameter(
            "source_topic", "/plato2/joint_impedance_controller/FTip_DistsAndAngles_HY"
        ).value
        self.target_topic: str = self.declare_parameter(
            "target_topic", "/plato2/parallel_grasp_controller/commands"
        ).value
        publish_rate: float = float(self.declare_parameter("publish_rate_hz", 100.0).value)
        period = 1.0 / max(1e-3, publish_rate)

        self._latest: Optional[Float64MultiArray] = None

        self.subscription = self.create_subscription(
            Float64MultiArray, self.source_topic, self.input_callback, 10
        )
        self.publisher = self.create_publisher(Float64MultiArray, self.target_topic, 10)
        self.timer = self.create_timer(period, self.publish_command)

        self.get_logger().info(
            f"ftip_to_parallel_grasp: {self.source_topic} -> {self.target_topic} at {publish_rate:.1f} Hz"
        )

    def input_callback(self, msg: Float64MultiArray) -> None:
        self._latest = msg

    def publish_command(self) -> None:
        if self._latest is None or not self._latest.data:
            return

        data = self._latest.data
        distance = _clamp01(data[0]) if len(data) > 0 else 0.0
        angle = _clamp01(data[3]) if len(data) > 3 else 0.0

        out = Float64MultiArray()
        out.data = [distance, angle]
        self.publisher.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FtipToParallelGrasp()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
