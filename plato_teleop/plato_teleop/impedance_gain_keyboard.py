#!/usr/bin/env python3
import sys
import termios
import tty
import threading
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class ImpedanceGainKeyboard(Node):
    def __init__(self):
        super().__init__('impedance_gain_keyboard')
        self._lock = threading.Lock()
        self._clients = {
            '0': self._make_client('zero', '/joint_position_controller_node/impedance_gains/soft'),
            '1': self._make_client('soft', '/joint_position_controller_node/impedance_gains/soft'),
            '2': self._make_client('medium', '/joint_position_controller_node/impedance_gains/medium'),
            '3': self._make_client('hard', '/joint_position_controller_node/impedance_gains/hard'),
        }
        self.get_logger().info("Impedance gain keyboard: 0=zero, 1=soft, 2=medium, 3=hard, q=quit")
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._key_loop, daemon=True)
        self._thread.start()

    def _make_client(self, label: str, service_name: str):
        client = self.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Service {service_name} ({label}) not available yet.")
        return client

    def _call(self, key: str):
        with self._lock:
            client = self._clients.get(key)
            if client is None:
                return
            if not client.service_is_ready():
                self.get_logger().warn(f"Service {client.srv_name} not ready.")
                return
            req = Trigger.Request()
            future = client.call_async(req)
            future.add_done_callback(lambda f, k=key: self._log_result(k, f))

    def _log_result(self, key: str, future):
        try:
            resp = future.result()
            self.get_logger().info(f"Key {key}: {resp.message} (success={resp.success})")
        except Exception as exc:
            self.get_logger().error(f"Key {key}: service call failed: {exc}")

    def _key_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            while rclpy.ok() and not self._stop.is_set():
                ch = sys.stdin.read(1)
                if ch in ('0', '1', '2', '3'):
                    self._call(ch)
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
