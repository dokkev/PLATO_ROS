#!/usr/bin/env python3
"""
TF Merger Node
Subscribes to /optimo/tf and /optimo/tf_static and republishes to /tf and /tf_static
This allows the optimo robot TF tree to be merged with the global TF tree.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage


class TFMerger(Node):
    def __init__(self):
        super().__init__('tf_merger')

        # QoS profile for TF static (TRANSIENT_LOCAL durability)
        qos_static = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Subscribers
        self.sub_tf = self.create_subscription(
            TFMessage,
            '/optimo/tf',
            self.tf_callback,
            10
        )

        self.sub_tf_static = self.create_subscription(
            TFMessage,
            '/optimo/tf_static',
            self.tf_static_callback,
            qos_static
        )

        # Publishers
        self.pub_tf = self.create_publisher(TFMessage, '/tf', 10)
        self.pub_tf_static = self.create_publisher(TFMessage, '/tf_static', qos_static)

        self.get_logger().info('TF Merger node started')
        self.get_logger().info('Subscribing to /optimo/tf and /optimo/tf_static')
        self.get_logger().info('Publishing to /tf and /tf_static')

    def tf_callback(self, msg):
        """Republish dynamic TF transforms"""
        self.pub_tf.publish(msg)

    def tf_static_callback(self, msg):
        """Republish static TF transforms"""
        self.pub_tf_static.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TFMerger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
