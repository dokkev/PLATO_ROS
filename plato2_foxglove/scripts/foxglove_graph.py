#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from threading import Lock


class JointStateSplitter(Node):
    def __init__(self):
        super().__init__('foxglove_graph')

        # Number of joints
        self.declare_parameter('joint_count', 8)
        self.joint_count = self.get_parameter('joint_count').get_parameter_value().integer_value

        # Subscribe to the joint states and position commands
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/plato2/joint_states',
            self.joint_state_callback,
            10
        )
        self.position_command_sub = self.create_subscription(
            Float64MultiArray,
            'plato2_position_controller/commands',
            self.position_command_callback,
            10
        )

        # Publishers for each joint state
        self.joint_publishers = [
            self.create_publisher(JointState, f'/plato2/joint_states/joint{i+1}', 10)
            for i in range(self.joint_count)
        ]

        # Publisher for sorted joint states
        self.sorted_joint_states_pub = self.create_publisher(
            JointState,
            '/plato2/joint_states/sorted',
            10
        )

        # Publishers for position commands
        self.command_publishers = [
            self.create_publisher(Float64, f'/plato2/plato2_position_controller/joint{i+1}', 10)
            for i in range(self.joint_count)
        ]

        # Initialize the latest command for each joint
        self.latest_commands = [0.0] * self.joint_count

        # Lock to protect shared resources
        self.lock = Lock()

        # Flag to indicate a new command has been received
        self.new_command_received = False

        # Create a timer to repeatedly publish commands
        self.command_publish_timer = self.create_timer(0.5, self.timer_publish_commands)

    def joint_state_callback(self, msg):
        # Sort the joint states based on joint names
        joint_data = list(zip(msg.name, msg.position, msg.velocity, msg.effort))
        sorted_joint_data = sorted(joint_data, key=lambda x: x[0])  # Sort by joint name

        sorted_names, sorted_positions, sorted_velocities, sorted_efforts = zip(*sorted_joint_data)

        # Create and publish the sorted joint states message
        sorted_msg = JointState()
        sorted_msg.header = msg.header
        sorted_msg.name = list(sorted_names)
        sorted_msg.position = list(sorted_positions)
        sorted_msg.velocity = list(sorted_velocities)
        sorted_msg.effort = list(sorted_efforts)

        self.sorted_joint_states_pub.publish(sorted_msg)

        # Publish individual joint states
        for i, name in enumerate(sorted_names):
            joint_msg = JointState()
            joint_msg.header = msg.header
            joint_msg.name = [name]
            joint_msg.position = [sorted_positions[i]]
            joint_msg.velocity = [sorted_velocities[i]]
            joint_msg.effort = [sorted_efforts[i]]

            self.joint_publishers[i].publish(joint_msg)

    def position_command_callback(self, msg):
        with self.lock:
            # Update the latest commands for each joint
            if len(msg.data) == self.joint_count:
                self.latest_commands = list(msg.data)
                self.new_command_received = True
                # self.get_logger().info("Received new position commands.")
                # Publish immediately
                self.publish_commands()
            else:
                self.get_logger().warn(
                    f"Received position command with {len(msg.data)} elements, expected {self.joint_count}."
                )

    def timer_publish_commands(self):
        with self.lock:
            if not self.new_command_received:
                # No new command received since last timer callback
                self.publish_commands()
            else:
                # Reset the flag to wait for the next interval
                self.new_command_received = False

    def publish_commands(self):
        # Publish the latest commands
        for i in range(self.joint_count):
            command_msg = Float64()
            command_msg.data = self.latest_commands[i]
            self.command_publishers[i].publish(command_msg)
            # self.get_logger().debug(f"Publishing command for joint{i+1}: {self.latest_commands[i]}")


def main(args=None):
    rclpy.init(args=args)
    node = JointStateSplitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
