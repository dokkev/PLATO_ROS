#!/usr/bin/env python3
"""
Index Fingertip Pose Publisher
Publishes the pose of the index_fingertip frame relative to base_link (local frame)
and optionally relative to a world frame.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class IndexFingertipPosePublisher(Node):
    def __init__(self):
        super().__init__('index_fingertip_pose_publisher')

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for index fingertip pose (local frame)
        self.pose_pub_local = self.create_publisher(
            PoseStamped,
            '/plato2/index_fingertip_pose_local',
            10
        )

        # Timer to publish pose at a regular rate (e.g., 50 Hz)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('publish_world_pose', False)
        self.declare_parameter('world_frame', 'stand')
        rate = self.get_parameter('publish_rate').value
        self.publish_world_pose = self.get_parameter('publish_world_pose').value
        self.world_frame = self.get_parameter('world_frame').value

        self.pose_pub_world = None
        if self.publish_world_pose:
            self.pose_pub_world = self.create_publisher(
                PoseStamped,
                '/plato2/index_fingertip_pose_world',
                10
            )

        self.timer = self.create_timer(1.0 / rate, self.publish_pose)

        self.get_logger().info(f'Index Fingertip Pose Publisher started at {rate} Hz')
        self.get_logger().info('Publishing to /plato2/index_fingertip_pose_local (base_link)')
        if self.publish_world_pose:
            self.get_logger().info(
                f'Publishing to /plato2/index_fingertip_pose_world ({self.world_frame})'
            )

    def publish_pose(self):
        """Get transforms and publish as PoseStamped messages"""
        current_time = self.get_clock().now()

        # Publish local pose (base_link to index_fingertip)
        try:
            transform_local = self.tf_buffer.lookup_transform(
                'base_link',  # target frame
                'index_fingertip',  # source frame
                rclpy.time.Time()  # get latest available
            )

            pose_msg_local = PoseStamped()
            pose_msg_local.header.stamp = current_time.to_msg()
            pose_msg_local.header.frame_id = 'base_link'

            pose_msg_local.pose.position.x = transform_local.transform.translation.x
            pose_msg_local.pose.position.y = transform_local.transform.translation.y
            pose_msg_local.pose.position.z = transform_local.transform.translation.z

            pose_msg_local.pose.orientation.x = transform_local.transform.rotation.x
            pose_msg_local.pose.orientation.y = transform_local.transform.rotation.y
            pose_msg_local.pose.orientation.z = transform_local.transform.rotation.z
            pose_msg_local.pose.orientation.w = transform_local.transform.rotation.w

            self.pose_pub_local.publish(pose_msg_local)

        except TransformException as ex:
            if not hasattr(self, '_last_error_time_local') or \
               (current_time - self._last_error_time_local).nanoseconds > 2e9:
                self.get_logger().warn(
                    f'Could not transform base_link to index_fingertip: {ex}'
                )
                self._last_error_time_local = current_time

        if not self.publish_world_pose:
            return

        # Publish world pose (world_frame to index_fingertip)
        try:
            transform_world = self.tf_buffer.lookup_transform(
                self.world_frame,  # target frame (e.g., 'stand')
                'index_fingertip',  # source frame
                rclpy.time.Time()  # get latest available
            )

            pose_msg_world = PoseStamped()
            pose_msg_world.header.stamp = current_time.to_msg()
            pose_msg_world.header.frame_id = self.world_frame

            pose_msg_world.pose.position.x = transform_world.transform.translation.x
            pose_msg_world.pose.position.y = transform_world.transform.translation.y
            pose_msg_world.pose.position.z = transform_world.transform.translation.z

            pose_msg_world.pose.orientation.x = transform_world.transform.rotation.x
            pose_msg_world.pose.orientation.y = transform_world.transform.rotation.y
            pose_msg_world.pose.orientation.z = transform_world.transform.rotation.z
            pose_msg_world.pose.orientation.w = transform_world.transform.rotation.w

            self.pose_pub_world.publish(pose_msg_world)

        except TransformException as ex:
            if not hasattr(self, '_last_error_time_world') or \
               (current_time - self._last_error_time_world).nanoseconds > 2e9:
                self.get_logger().warn(
                    f'Could not transform {self.world_frame} to index_fingertip: {ex}'
                )
                self._last_error_time_world = current_time


def main(args=None):
    rclpy.init(args=args)
    node = IndexFingertipPosePublisher()

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
