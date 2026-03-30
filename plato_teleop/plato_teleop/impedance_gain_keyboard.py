#!/usr/bin/env python3
import sys
import termios
import tty
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ImpedanceGainKeyboard(Node):
    def __init__(self):
        super().__init__('impedance_gain_keyboard')
        self._lock = threading.Lock()
        self._publisher = self.create_publisher(
            Float64,
            '/impedance_trajectory_controller_node/impedance_level',
            10,
        )
        self.get_logger().info(
            "Impedance gain keyboard: 0-9=set level, a=10, q=quit")
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._key_loop, daemon=True)
        self._thread.start()

    def _publish_level(self, level: float):
        with self._lock:
            msg = Float64()
            msg.data = float(level)
            self._publisher.publish(msg)
            self.get_logger().info(f"Impedance level -> {level:.1f}")

    def _key_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            while rclpy.ok() and not self._stop.is_set():
                ch = sys.stdin.read(1)
                if ch.isdigit():
                    self._publish_level(float(ch))
                elif ch in ('a', 'A'):
                    self._publish_level(10.0)
                elif ch in ('q', 'Q'):
                    self.get_logger().info("Quitting keyboard control.")
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def destroy_node(self):
        self._stop.set()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImpedanceGainKeyboard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
