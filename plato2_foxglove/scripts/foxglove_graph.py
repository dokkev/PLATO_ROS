#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray


class JointStateSplitter(Node):
    def __init__(self):
        super().__init__('foxglove_graph')

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
            for i in range(8)
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
            for i in range(8)
        ]

        # Initialize the latest command for each joint
        self.latest_commands = [0.0] * 8

        # Create a timer to repeatedly publish commands
        self.command_publish_timer = self.create_timer(0.5, self.publish_commands)

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
        # Update the latest commands for each joint
        if len(msg.data) == 8:
            self.latest_commands = list(msg.data)
            self.get_logger().info("Updated position commands.")

    def publish_commands(self):
        # Publish the latest commands continuously
        for i in range(8):
            command_msg = Float64()
            command_msg.data = self.latest_commands[i]
            self.command_publishers[i].publish(command_msg)


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
