#!/usr/bin/env python3
"""Keyboard teleop for thumb state control.

Publishes std_msgs/Int32 commands to /plato2/thumb_state. The retargeting
converter uses those commands to choose preset positions for joint indices 0
and 1 while ignoring upstream retargeting values for those joints.
"""

from __future__ import annotations

import sys
import termios
import tty
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class ThumbKeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("thumb_keyboard_teleop")

        self.thumb_state_topic: str = self.declare_parameter(
            "thumb_state_topic", "/plato2/thumb_state"
        ).value

        self.publisher = self.create_publisher(Int32, self.thumb_state_topic, 10)
        self.current_state: Optional[int] = None
        self.state_map = {
            "0": (0, "open [0.785, 0.314]"),
            "1": (1, "index pinch [0.0, 0.0]"),
            "2": (2, "middle pinch [-0.430, -0.450]"),
            "3": (3, "all pinch [-0.3, -0.3]"),
        }

        self.get_logger().info(f"Thumb keyboard teleop publishing to {self.thumb_state_topic}")
        self._print_instructions()

    def _print_instructions(self) -> None:
        print("\nTHUMB STATE KEYBOARD TELEOP")
        print("0: open")
        print("1: index pinch")
        print("2: middle pinch")
        print("3: all pinch")
        print("q: quit\n")

    def _get_key(self) -> str:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def publish_state(self, state: int) -> None:
        msg = Int32()
        msg.data = state
        self.publisher.publish(msg)
        self.current_state = state

        description = next(
            (desc for _, (mapped_state, desc) in self.state_map.items() if mapped_state == state),
            "unknown",
        )
        self.get_logger().info(f"Published thumb state {state}: {description}")
        print(f"State {state} published: {description}")

    def run(self) -> None:
        if not sys.stdin.isatty():
            self.get_logger().error("thumb_keyboard_teleop requires an interactive terminal.")
            return

        while rclpy.ok():
            key = self._get_key()
            if key.lower() == "q" or key == "\x03":
                print("\nExiting thumb keyboard teleop...")
                break
            if key in self.state_map:
                state, _ = self.state_map[key]
                self.publish_state(state)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ThumbKeyboardTeleop()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
