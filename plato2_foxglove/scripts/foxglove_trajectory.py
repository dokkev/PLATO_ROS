#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from plato2_foxglove.msg import Trajectory
import yaml
import os
import time

class TrajectoryManager(Node):
    def __init__(self):
        super().__init__('trajectory_manager')

        # Subscriber for Trajectory messages to save
        self.trajectory_subscriber = self.create_subscription(
            Trajectory,
            'save_trajectory',
            self.trajectory_callback,
            10
        )

        # Subscriber for trajectory execution commands
        self.command_subscriber = self.create_subscription(
            String,
            'execute_trajectory',
            self.execute_callback,
            10
        )

        # Publisher for the joint commands
        self.commands_publisher = self.create_publisher(
            Float64MultiArray,
            '/plato2/plato2_position_controller/commands',
            10
        )

        # Load joint states from joint_states.yaml
        self.joint_states = self.load_yaml_file('joint_states.yaml')

        self.get_logger().info("Trajectory Manager Node initialized.")

    def load_yaml_file(self, file_name):
        file_path = os.path.join(os.getcwd(), file_name)

        if not os.path.exists(file_path):
            self.get_logger().warning(f"{file_name} not found at {file_path}. Creating a new one.")
            return {}

        with open(file_path, 'r') as file:
            try:
                data = yaml.safe_load(file) or {}
                self.get_logger().info(f"Loaded data from {file_name}")
                return data
            except yaml.YAMLError as e:
                self.get_logger().error(f"Error reading {file_name}: {e}")
                return {}

    def save_yaml_file(self, file_name, data):
        file_path = os.path.join(os.getcwd(), file_name)
        with open(file_path, 'w') as file:
            yaml.dump(data, file, default_flow_style=False)
        self.get_logger().info(f"Saved data to {file_name}")

    def trajectory_callback(self, msg):
        trajectory_name = msg.trajectory_name.strip()
        position_names = msg.position_name
        motion_times = msg.motion_time
        hold_times = msg.hold_time

        if len(position_names) != len(motion_times) or len(position_names) != len(hold_times):
            self.get_logger().error("Lengths of position_name, motion_time, and hold_time arrays must be equal.")
            return

        # Build the trajectory data
        trajectory_data = {}
        for i in range(len(position_names)):
            position_name = position_names[i]
            motion_time = motion_times[i]
            hold_time = hold_times[i]

            trajectory_data[position_name] = {
                'Motion_Time': motion_time,
                'Hold_Time': hold_time
            }

        # Load existing trajectories
        trajectories = self.load_yaml_file('trajectory.yaml')

        # Save the new trajectory
        trajectories[trajectory_name] = trajectory_data
        self.save_yaml_file('trajectory.yaml', trajectories)

        self.get_logger().info(f"Trajectory '{trajectory_name}' saved.")

    def execute_callback(self, msg):
        trajectory_name = msg.data.strip()
        trajectories = self.load_yaml_file('trajectory.yaml')

        if trajectory_name in trajectories:
            trajectory_plan = trajectories[trajectory_name]
            self.execute_trajectory(trajectory_plan)
        else:
            self.get_logger().error(f"Trajectory '{trajectory_name}' not found in trajectory.yaml.")

    def execute_trajectory(self, trajectory_plan):
        # Start from zero position
        if 'zero_position' in self.joint_states:
            current_positions = self.joint_states['zero_position']
        else:
            current_positions = [0.0] * self.get_joint_count()
            self.get_logger().warning("Zero position not found in joint_states.yaml. Using zeros.")

        self.get_logger().info(f"Starting trajectory execution from zero position: {current_positions}")

        # Initialize time accumulator
        total_time = 0.0

        # Iterate over the trajectory plan
        for position_name, timings in trajectory_plan.items():
            if position_name not in self.joint_states:
                self.get_logger().error(f"Position '{position_name}' not found in joint_states.yaml.")
                return

            target_positions = self.joint_states[position_name]
            motion_time = timings.get('Motion_Time', 0.0)
            hold_time = timings.get('Hold_Time', 0.0)

            self.get_logger().info(f"Moving to '{position_name}' over {motion_time}s and holding for {hold_time}s.")

            # Interpolate from current_positions to target_positions over motion_time
            self.interpolate_and_publish(current_positions, target_positions, motion_time)

            # Hold at target_positions for hold_time
            if hold_time > 0:
                self.publish_position(target_positions)
                self.get_logger().info(f"Holding position '{position_name}' for {hold_time}s.")
                time.sleep(hold_time)

            # Update current positions
            current_positions = target_positions

        self.get_logger().info("Trajectory execution completed.")

    def interpolate_and_publish(self, start_positions, end_positions, duration):
        if duration <= 0.0:
            # Immediate move to the target position
            self.publish_position(end_positions)
            return

        steps = int(duration * 10)  # For example, 10 steps per second
        sleep_time = duration / steps

        for i in range(1, steps + 1):
            interpolated_positions = [
                start + (end - start) * (i / steps)
                for start, end in zip(start_positions, end_positions)
            ]
            self.publish_position(interpolated_positions)
            time.sleep(sleep_time)

    def publish_position(self, positions):
        msg = Float64MultiArray()
        msg.data = positions
        self.commands_publisher.publish(msg)
        self.get_logger().debug(f"Published positions: {positions}")

    def get_joint_count(self):
        # Get the number of joints from zero_position or any other position
        if self.joint_states:
            first_key = next(iter(self.joint_states))
            positions = self.joint_states[first_key]
            return len(positions)
        else:
            self.get_logger().error("No joint states found.")
            return 0

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Trajectory Manager Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
