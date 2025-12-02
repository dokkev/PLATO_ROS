#!/usr/bin/env python3
"""SpaceMouse Twist Publisher Node.

This node replaces the functionality of controller_spaceMouse.cpp by:
1. Reading SpaceMouse data directly using the spacemouse_hardware module
2. Publishing TwistStamped messages for robot control
3. Deciding between translation and rotation based on magnitude thresholds
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import math
from plato_teleop.spacemouse_hardware import SpaceMouseHardware, SpaceMouseState


class SpaceMouseTwistPublisher(Node):
    """ROS2 node that publishes twist commands from SpaceMouse input."""

    def __init__(self):
        super().__init__('spacemouse_twist_publisher')

        # Declare parameters
        self.declare_parameter('translation_scale', 0.01)
        self.declare_parameter('rotation_scale', 0.05)
        self.declare_parameter('translation_threshold', 0.001)
        self.declare_parameter('rotation_threshold', 0.001)
        self.declare_parameter('publish_rate', 900.0)  # Hz
        self.declare_parameter('twist_topic', '/optimo/servo/twist_cmd')
        self.declare_parameter('frame_id', 'world')

        # Get parameter values
        self.translation_scale = self.get_parameter('translation_scale').value
        self.rotation_scale = self.get_parameter('rotation_scale').value
        self.translation_threshold = self.get_parameter('translation_threshold').value
        self.rotation_threshold = self.get_parameter('rotation_threshold').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.twist_topic = self.get_parameter('twist_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        # Initialize SpaceMouse hardware
        self.spacemouse = SpaceMouseHardware()
        if not self.spacemouse.open():
            self.get_logger().error('Failed to open SpaceMouse device')
            raise RuntimeError('SpaceMouse initialization failed')

        self.get_logger().info('SpaceMouse initialized successfully')

        # Create publisher for twist commands
        self.twist_pub = self.create_publisher(
            TwistStamped,
            self.twist_topic,
            10
        )

        # Create timer for reading SpaceMouse and publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'SpaceMouse Twist Publisher started:\n'
            f'  - Topic: {self.twist_topic}\n'
            f'  - Rate: {self.publish_rate} Hz\n'
            f'  - Translation scale: {self.translation_scale}\n'
            f'  - Rotation scale: {self.rotation_scale}\n'
            f'  - Translation threshold: {self.translation_threshold}\n'
            f'  - Rotation threshold: {self.rotation_threshold}'
        )

    def calculate_translation_magnitude(self, state: SpaceMouseState) -> float:
        """Calculate the magnitude of translation motion.

        Args:
            state: Current SpaceMouse state.

        Returns:
            float: Translation magnitude.
        """
        return math.sqrt(state.x**2 + state.y**2 + state.z**2)

    def calculate_rotation_magnitude(self, state: SpaceMouseState) -> float:
        """Calculate the magnitude of rotation motion.

        Args:
            state: Current SpaceMouse state.

        Returns:
            float: Rotation magnitude.
        """
        return math.sqrt(state.roll**2 + state.pitch**2 + state.yaw**2)

    def decide_movement(self, state: SpaceMouseState) -> str:
        """Decide whether to translate, rotate, or do nothing based on magnitudes.

        Args:
            state: Current SpaceMouse state.

        Returns:
            str: 'translation', 'rotation', or 'none'.
        """
        translation_magnitude = self.calculate_translation_magnitude(state)
        rotation_magnitude = self.calculate_rotation_magnitude(state)

        # Determine which set of actions to take based on thresholds and magnitudes
        if (translation_magnitude > self.translation_threshold and
                translation_magnitude > rotation_magnitude):
            return 'translation'
        elif rotation_magnitude > self.rotation_threshold:
            return 'rotation'
        return 'none'

    def publish_twist(self, linear_x: float = 0.0, linear_y: float = 0.0,
                      linear_z: float = 0.0, angular_x: float = 0.0,
                      angular_y: float = 0.0, angular_z: float = 0.0):
        """Publish a twist message.

        Args:
            linear_x, linear_y, linear_z: Linear velocity components.
            angular_x, angular_y, angular_z: Angular velocity components.
        """
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.frame_id

        twist_msg.twist.linear.x = linear_x
        twist_msg.twist.linear.y = linear_y
        twist_msg.twist.linear.z = linear_z

        twist_msg.twist.angular.x = angular_x
        twist_msg.twist.angular.y = angular_y
        twist_msg.twist.angular.z = angular_z

        self.twist_pub.publish(twist_msg)

    def timer_callback(self):
        """Timer callback to read SpaceMouse and publish twist commands."""
        # Read SpaceMouse state
        state = self.spacemouse.read()
        if state is None:
            self.get_logger().warn('Failed to read SpaceMouse state', throttle_duration_sec=1.0)
            return

        # Apply axis mapping from the original spaceMouse_publisher
        # The mapping: y -> axes[0], -x -> axes[1], z -> axes[2]
        # roll -> axes[3], pitch -> axes[4], -yaw -> axes[5]
        mapped_x = state.y
        mapped_y = -state.x
        mapped_z = state.z
        mapped_roll = state.roll
        mapped_pitch = state.pitch
        mapped_yaw = -state.yaw

        # Create a mapped state for decision making
        mapped_state = SpaceMouseState(
            x=mapped_x,
            y=mapped_y,
            z=mapped_z,
            roll=mapped_roll,
            pitch=mapped_pitch,
            yaw=mapped_yaw,
            buttons=state.buttons
        )

        # Decide movement type
        action = self.decide_movement(mapped_state)

        if action == 'none':
            # Publish zero twist
            self.publish_twist()
        elif action == 'translation':
            # Publish translation twist
            self.publish_twist(
                linear_x=mapped_x * self.translation_scale,
                linear_y=mapped_y * self.translation_scale,
                linear_z=mapped_z * self.translation_scale
            )
        elif action == 'rotation':
            # Publish rotation twist
            self.publish_twist(
                angular_x=mapped_roll * self.rotation_scale,
                angular_y=mapped_pitch * self.rotation_scale,
                angular_z=mapped_yaw * self.rotation_scale
            )

        # Handle button presses if needed
        pressed, released = self.spacemouse.get_button_transitions(state.buttons)
        if pressed:
            self.get_logger().info(f'Buttons pressed: {pressed}')
        if released:
            self.get_logger().info(f'Buttons released: {released}')

    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        self.spacemouse.close()
        self.get_logger().info('SpaceMouse closed successfully')
        super().destroy_node()


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)

    try:
        node = SpaceMouseTwistPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
