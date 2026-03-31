#!/usr/bin/env python3
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GraspTaskKeyboard(Node):
    def __init__(self):
        super().__init__('grasp_task_keyboard')
        self._lock = threading.Lock()
        self._stop = threading.Event()

        self._task_topic = self.declare_parameter(
            'task_topic',
            '/plato2/plato_grasp_controller/task',
        ).value
        binding_specs = self.declare_parameter(
            'bindings',
            ['0:idle', '1:dorsal_index_pinch'],
        ).value

        self._bindings = self._parse_bindings(binding_specs)
        self._publisher = self.create_publisher(String, self._task_topic, 10)

        self._print_help()
        self._thread = threading.Thread(target=self._key_loop, daemon=True)
        self._thread.start()

    def _normalize_key(self, key: str) -> str:
        return key.lower() if key.isalpha() else key

    def _parse_bindings(self, binding_specs):
        bindings = {}
        for raw_spec in binding_specs:
            spec = str(raw_spec).strip()
            key, separator, task_name = spec.partition(':')
            key = self._normalize_key(key.strip())
            task_name = task_name.strip()

            if separator != ':' or len(key) != 1 or not task_name:
                raise ValueError(
                    f"Invalid binding '{raw_spec}'. Expected format '<key>:<task_name>'."
                )
            if key in ('q', 'h'):
                raise ValueError(
                    f"Binding key '{key}' is reserved for quit/help."
                )
            if key in bindings:
                raise ValueError(f"Duplicate binding for key '{key}'.")

            bindings[key] = task_name

        if not bindings:
            raise ValueError("At least one key binding is required.")

        return bindings

    def _print_help(self):
        binding_text = ', '.join(
            f"{key}={task_name}" for key, task_name in self._bindings.items()
        )
        self.get_logger().info(
            f"Grasp task keyboard ready. Publishing to {self._task_topic}")
        self.get_logger().info(
            f"Bindings: {binding_text}. h=help, q=quit")

    def _publish_task(self, task_name: str):
        with self._lock:
            message = String()
            message.data = task_name
            self._publisher.publish(message)
            self.get_logger().info(f"Triggered grasp task '{task_name}'")

    def _key_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            while rclpy.ok() and not self._stop.is_set():
                ch = sys.stdin.read(1)
                if not ch:
                    continue

                key = self._normalize_key(ch)
                if key in self._bindings:
                    self._publish_task(self._bindings[key])
                elif key == 'h':
                    self._print_help()
                elif key == 'q' or ch == '\x03':
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
    node = GraspTaskKeyboard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
