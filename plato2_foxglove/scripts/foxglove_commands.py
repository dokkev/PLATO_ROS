#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import yaml
import os


class JointStateRecorder(Node):
    def __init__(self):
        super().__init__('joint_state_recorder')
        
        # Subscriber for joint state name to record
        self.string_subscriber = self.create_subscription(
            String,
            'joint_state_name',
            self.string_callback,
            10
        )
        
        # Subscriber for joint state name to fetch and publish
        self.fetch_subscriber = self.create_subscription(
            String,
            'fetch_joint_state',
            self.fetch_callback,
            10
        )
        
        # Publisher for the fetched joint state
        self.commands_publisher = self.create_publisher(
            Float64MultiArray,
            '/plato2/plato2_position_controller/commands',
            10
        )
        
        self.joint_state_subscriber = None
        self.joint_state_name = None
        self.joint_state_data = None
        self.get_logger().info("Joint State Recorder Node initialized.")

    def string_callback(self, msg):
        self.joint_state_name = msg.data
        self.get_logger().info(f"Received string: {self.joint_state_name}")
        if not self.joint_state_subscriber:
            self.joint_state_subscriber = self.create_subscription(
                JointState,
                '/plato2/joint_states/sorted',
                self.joint_state_callback,
                10
            )
            self.get_logger().info("Subscribed to /plato2/joint_states/sorted topic.")

    def joint_state_callback(self, msg):
        self.joint_state_data = msg.position
        self.save_to_yaml()
        # Unsubscribe from /joint_states to avoid repeated saving
        self.destroy_subscription(self.joint_state_subscriber)
        self.joint_state_subscriber = None
        self.get_logger().info("Joint state data saved to YAML file.")

    def save_to_yaml(self):
        if self.joint_state_name and self.joint_state_data is not None:
            # Prepare data to save
            data_to_save = {self.joint_state_name: list(self.joint_state_data)}

            file_path = os.path.join(os.getcwd(), 'joint_states.yaml')

            # Check if the file exists and update the content
            if os.path.exists(file_path):
                with open(file_path, 'r') as file:
                    try:
                        existing_data = yaml.safe_load(file) or {}
                    except yaml.YAMLError:
                        self.get_logger().error("Error reading YAML file. Creating a new file.")
                        existing_data = {}
                existing_data.update(data_to_save)
            else:
                existing_data = data_to_save

            # Write back the updated content
            with open(file_path, 'w') as file:
                yaml.dump(existing_data, file, default_flow_style=False)

            self.get_logger().info(f"Data saved in {file_path}")

    def fetch_callback(self, msg):
        requested_name = msg.data
        file_path = os.path.join(os.getcwd(), 'joint_states.yaml')

        # Check if the YAML file exists
        if not os.path.exists(file_path):
            self.get_logger().error(f"YAML file not found at {file_path}.")
            return

        # Read the YAML file
        with open(file_path, 'r') as file:
            try:
                data = yaml.safe_load(file) or {}
            except yaml.YAMLError:
                self.get_logger().error("Error reading YAML file.")
                return

        # Check if the requested name exists
        if requested_name in data:
            joint_state = data[requested_name]
            self.publish_joint_state(joint_state)
        else:
            self.get_logger().error(f"No entry found for '{requested_name}' in YAML file.")

    def publish_joint_state(self, joint_state):
        msg = Float64MultiArray()
        msg.data = joint_state
        self.commands_publisher.publish(msg)
        self.get_logger().info(f"Published joint state: {joint_state}")


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
