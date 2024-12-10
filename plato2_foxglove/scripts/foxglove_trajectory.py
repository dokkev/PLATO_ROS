#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from plato2_interfaces.msg import Trajectory
from std_msgs.msg import Float64MultiArray
from collections import OrderedDict
import yaml
import os
import time
import numpy as np  # **Added Import**

# Custom representer for OrderedDict to ensure correct YAML output
def represent_ordereddict(dumper, data):
    return dumper.represent_mapping('tag:yaml.org,2002:map', data.items())

yaml.add_representer(OrderedDict, represent_ordereddict)

class TrajectoryManager(Node):
    def __init__(self):
        super().__init__('trajectory_manager')

        # Subscriber for Trajectory messages to save
        self.trajectory_subscriber = self.create_subscription(
            Trajectory,
            '/plato2/trajectory_save',
            self.trajectory_callback,
            10
        )

        # Subscriber for trajectory execution commands
        self.command_subscriber = self.create_subscription(
            String,
            '/plato2/trajectory_execute',
            self.execute_callback,
            10
        )

        # Publisher for the joint commands
        self.commands_publisher = self.create_publisher(
            Float64MultiArray,
            '/plato2/plato2_position_controller/commands',
            10
        )

        # Initial load of joint states and trajectories
        self.joint_states = self.load_yaml_file('joint_states.yaml')
        self.trajectories = self.load_yaml_file('trajectory.yaml')

        # Get joint names (assuming all positions have the same number of joints)
        self.joint_names = self.get_joint_names()

        if not self.joint_names:
            self.get_logger().error("No joint names found. Ensure joint_states.yaml has at least one position.")
        else:
            self.get_logger().info(f"Joint names: {self.joint_names}")

        self.get_logger().info("Trajectory Manager Node initialized.")

    def load_yaml_file(self, file_name):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        ws_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_dir))))
        file_path = os.path.join(ws_dir, 'src', 'PLATO_ROS', 'plato2_foxglove', 'config', file_name)

        if not os.path.exists(file_path):
            if file_name == 'trajectory.yaml':
                self.get_logger().info(f"{file_name} not found. It will be created upon saving a trajectory.")
            else:
                self.get_logger().error(f"{file_name} not found at {file_path}. Please ensure it exists.")
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
        script_dir = os.path.dirname(os.path.abspath(__file__))
        ws_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_dir))))
        file_path = os.path.join(ws_dir, 'src', 'PLATO_ROS', 'plato2_foxglove', 'config', file_name)
        with open(file_path, 'w') as file:
            yaml.dump(data, file, default_flow_style=False)
        self.get_logger().info(f"Saved data to {file_name}")

    def get_joint_names(self):
        # Extract joint names from the first entry in joint_states
        if self.joint_states:
            first_key = next(iter(self.joint_states))
            positions = self.joint_states[first_key]
            if isinstance(positions, list):
                # Create default joint names if not specified
                joint_count = len(positions)
                joint_names = [f'joint{i+1}' for i in range(joint_count)]
                return joint_names
        self.get_logger().error("No joint states found or invalid format.")
        return []

    def trajectory_callback(self, msg):
        # Reload YAML files whenever a trajectory is saved
        self.joint_states = self.load_yaml_file('joint_states.yaml')
        self.trajectories = self.load_yaml_file('trajectory.yaml')

        trajectory_name = msg.trajectory_name.strip()
        position_names = msg.position_name
        motion_times = msg.motion_time
        hold_times = msg.hold_time

        self.get_logger().info(f"Received Trajectory message to save: '{trajectory_name}'")

        # Validate message lengths
        if not (len(position_names) == len(motion_times) == len(hold_times)):
            self.get_logger().error("Lengths of position_name, motion_time, and hold_time arrays must be equal.")
            return

        # Build the trajectory data as a list
        trajectory_steps = []
        for i in range(len(position_names)):
            position_name = position_names[i]
            motion_time = motion_times[i] if motion_times[i] is not None else 0.0
            hold_time = hold_times[i] if hold_times[i] is not None else 0.0

            if position_name not in self.joint_states:
                self.get_logger().error(f"Position '{position_name}' not found in joint_states.yaml.")
                return

            # Use OrderedDict to maintain key order
            step = OrderedDict()
            step['position_name'] = position_name
            step['Motion_Time'] = motion_time
            step['Hold_Time'] = hold_time
            trajectory_steps.append(step)

            self.get_logger().debug(f"Added step: {trajectory_steps[-1]}")

        # Save the new trajectory
        self.trajectories[trajectory_name] = trajectory_steps
        self.save_yaml_file('trajectory.yaml', self.trajectories)

        self.get_logger().info(f"Trajectory '{trajectory_name}' saved successfully.")

    def execute_callback(self, msg):
        # Reload YAML files whenever a trajectory is executed
        self.joint_states = self.load_yaml_file('joint_states.yaml')
        self.trajectories = self.load_yaml_file('trajectory.yaml')

        trajectory_name = msg.data.strip()
        self.get_logger().info(f"Received command to execute trajectory: '{trajectory_name}'")

        if trajectory_name not in self.trajectories:
            self.get_logger().error(f"Trajectory '{trajectory_name}' not found in trajectory.yaml.")
            return

        trajectory_plan = self.trajectories[trajectory_name]
        self.execute_trajectory(trajectory_plan)

    def execute_trajectory(self, trajectory_plan):
        # Start from zero position
        if 'zero_position' in self.joint_states:
            current_positions = self.joint_states['zero_position']
        else:
            current_positions = [0.0] * len(self.joint_names)
            self.get_logger().warning("Zero position not found in joint_states.yaml. Using zeros.")

        self.get_logger().info(f"Starting trajectory execution from zero position: {current_positions}")

        # Move to zero position first (if not already there)
        self.publish_position(current_positions)
        time.sleep(0.5)  # Small delay to ensure the robot starts from zero

        # Iterate over the trajectory plan
        for step in trajectory_plan:
            position_name = step['position_name']
            motion_time = step.get('Motion_Time', 0.0)
            hold_time = step.get('Hold_Time', 0.0)

            if position_name not in self.joint_states:
                self.get_logger().error(f"Position '{position_name}' not found in joint_states.yaml.")
                return

            target_positions = self.joint_states[position_name]

            self.get_logger().info(f"Moving to '{position_name}' over {motion_time}s and holding for {hold_time}s.")

            # Interpolate from current_positions to target_positions over motion_time using minimum jerk
            self.interpolate_and_publish(current_positions, target_positions, motion_time)

            # Hold at target_positions for hold_time
            if hold_time > 0.0:
                self.publish_position(target_positions)
                self.get_logger().info(f"Holding position '{position_name}' for {hold_time}s.")
                time.sleep(hold_time)

            # Update current positions
            current_positions = target_positions

        self.get_logger().info("Trajectory execution completed.")

    def minimum_jerk_trajectory(self, init_pos, target_pos, total_time=0.5, dt=0.01):
        """
        Generates a minimum jerk trajectory between init_pos and target_pos.

        :param init_pos: List of initial positions for each joint.
        :param target_pos: List of target positions for each joint.
        :param total_time: Total time to execute the trajectory.
        :param dt: Time step for trajectory sampling.
        :return: List of interpolated positions at each time step.
        """
        xi = np.array(init_pos)
        xf = np.array(target_pos)
        d = total_time

        if d == 0:
            return [xf.tolist()]

        num_steps = int(d / dt)
        list_x = []

        for step in range(1, num_steps + 1):
            t = step * dt
            s = 10 * (t / d) ** 3 - 15 * (t / d) ** 4 + 6 * (t / d) ** 5  # Minimum Jerk Polynomial
            interpolated = xi + (xf - xi) * s
            list_x.append(interpolated.tolist())

        return list_x

    def interpolate_and_publish(self, start_positions, end_positions, duration):
        if duration <= 0.0:
            # Immediate move to the target position
            self.publish_position(end_positions)
            return

        dt = 0.01 # Time step (seconds)
        interpolated_positions = self.minimum_jerk_trajectory(start_positions, end_positions, total_time=duration, dt=dt)

        self.get_logger().debug(f"Interpolating using minimum jerk over {len(interpolated_positions)} steps with {dt}s between steps.")

        for pos in interpolated_positions:
            self.publish_position(pos)
            time.sleep(dt)

    def publish_position(self, positions):
        msg = Float64MultiArray()
        msg.data = positions
        self.commands_publisher.publish(msg)
        self.get_logger().debug(f"Published positions: {positions}")

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
