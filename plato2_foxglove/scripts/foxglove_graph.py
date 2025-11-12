#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from plato2_interfaces.msg import ImpedanceCommands
from threading import Lock


class JointStateSplitter(Node):
    def __init__(self):
        super().__init__('foxglove_graph')

        # Number of joints
        self.declare_parameter('joint_count', 8)
        self.joint_count = self.get_parameter('joint_count').get_parameter_value().integer_value

        # Subscribe to the joint states and impedance commands
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/plato2/joint_states',
            self.joint_state_callback,
            10
        )
        self.impedance_command_sub = self.create_subscription(
            ImpedanceCommands,
            '/plato2/joint_impedance_controller/commands',
            self.impedance_command_callback,
            10
        )

        # Publishers for each joint state (renamed under foxglove_graph, 0-based indexing)
        self.joint_publishers = [
            self.create_publisher(JointState, f'/plato2/foxglove_graph/joint{i}_states', 10)
            for i in range(self.joint_count)
        ]

        # Publisher for sorted joint states
        self.sorted_joint_states_pub = self.create_publisher(
            JointState,
            '/plato2/joint_states/sorted',
            10
        )

        # Per-joint publishers using JointState format for commands
        self.joint_command_pubs = [
            self.create_publisher(JointState, f'/plato2/foxglove_graph/joint{i}_commands', 10)
            for i in range(self.joint_count)
        ]

        # Initialize the latest command caches for each joint
        self.latest_position = [0.0] * self.joint_count
        self.latest_velocity = [0.0] * self.joint_count
        self.joint_names = [f'joint{i}' for i in range(self.joint_count)]

        # Lock to protect shared resources
        self.lock = Lock()

        # Flag to indicate a new command has been received
        self.new_command_received = False

    # Create a timer to repeatedly publish commands (for consistent streaming)
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

    def impedance_command_callback(self, msg: ImpedanceCommands):
        with self.lock:
            # Robustly size-check and assign each field
            def take(arr, n):
                a = list(arr) if arr is not None else []
                if len(a) < n:
                    a.extend([0.0] * (n - len(a)))
                return a[:n]

            self.latest_position = take(msg.position, self.joint_count)
            self.latest_velocity = take(msg.velocity, self.joint_count)
            # Ignore stiffness/damping/effort_ff for graphing

            self.new_command_received = True
            # Publish immediately
            self.publish_commands()

    def timer_publish_commands(self):
        with self.lock:
            if not self.new_command_received:
                # No new command received since last timer callback
                self.publish_commands()
            else:
                # Reset the flag to wait for the next interval
                self.new_command_received = False

    def publish_commands(self):
        # Publish per-joint commands using JointState format
        for i in range(self.joint_count):
            cmd_msg = JointState()
            cmd_msg.name = [self.joint_names[i]]
            cmd_msg.position = [self.latest_position[i]]
            cmd_msg.velocity = [self.latest_velocity[i]]
            cmd_msg.effort = []  # Empty for commands
            self.joint_command_pubs[i].publish(cmd_msg)


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
