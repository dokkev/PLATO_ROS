#!/usr/bin/env python3
"""
Joint Impedance Controller Publisher
Publishes commands directly to the joint impedance controller
"""

import rclpy
from rclpy.node import Node
from plato2_interfaces.msg import ImpedanceCommands


class JointImpedancePublisher(Node):
    def __init__(self):
        super().__init__('joint_impedance_publisher')

        # Publisher for impedance commands
        self.cmd_pub = self.create_publisher(
            ImpedanceCommands,
            '/plato2/joint_impedance_controller/commands',
            10
        )

        # Initialize command message with fixed values
        self.cmd_msg = ImpedanceCommands()
        self.cmd_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.cmd_msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.cmd_msg.effort_ff = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.cmd_msg.stiffness = [0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 0.0, 0.0]
        self.cmd_msg.damping = [0.15, 0.15, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

        # Publish rate parameter
        self.declare_parameter('publish_rate', 100.0)
        rate = self.get_parameter('publish_rate').value

        # Timer to publish commands
        self.timer = self.create_timer(1.0 / rate, self.publish_command)

        self.get_logger().info(f'Joint Impedance Publisher started at {rate} Hz')
        self.get_logger().info('Publishing to /plato2/joint_impedance_controller/commands')
        self.get_logger().info(f'Command: position={self.cmd_msg.position}')
        self.get_logger().info(f'         damping={self.cmd_msg.damping}')

    def publish_command(self):
        """Publish impedance commands"""
        self.cmd_pub.publish(self.cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointImpedancePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
