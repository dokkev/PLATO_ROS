#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from plato2_interfaces.msg import Trajectory
from std_msgs.msg import Float64MultiArray
from collections import OrderedDict
import yaml
import os
import threading
import numpy as np  # Imported numpy for numerical operations
import time

# Custom representer for OrderedDict to ensure correct YAML output
def represent_ordereddict(dumper, data):
    return dumper.represent_mapping('tag:yaml.org,2002:map', data.items())

yaml.add_representer(OrderedDict, represent_ordereddict)

class TrajectoryManager(Node):
    def __init__(self):
        super().__init__('trajectory_manager')

        # Declare and get parameters
        self.declare_parameter('interpolation_rate', 100)  # Steps per second (dt = 0.01s)
        self.interpolation_rate = self.get_parameter('interpolation_rate').value

        # Subscribers and Publishers
        self.trajectory_subscriber = self.create_subscription(
            Trajectory,
            '/plato2/trajectory_save',
            self.trajectory_callback,
            10
        )
        self.command_subscriber = self.create_subscription(
            String,
            '/plato2/trajectory_execute',
            self.execute_callback,
            10
        )
        self.commands_publisher = self.create_publisher(
            Float64MultiArray,
            '/plato2/plato2_position_controller/commands',
            10
        )

        # Threading for trajectory execution
        self.execution_thread = None
        self.execution_lock = threading.Lock()
        self.shutdown_flag = False

        # Define hard-coded paths to YAML files
        script_dir = os.path.dirname(os.path.abspath(__file__))
        ws_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_dir))))
        self.joint_states_path = os.path.join(ws_dir, 'src', 'PLATO_ROS', 'plato2_foxglove', 'config', 'joint_states.yaml')
        self.trajectories_path = os.path.join(ws_dir, 'src', 'PLATO_ROS', 'plato2_foxglove', 'config', 'trajectory.yaml')

        # Load configurations initially
        self.joint_states = self.load_yaml_file(self.joint_states_path)
        self.trajectories = self.load_yaml_file(self.trajectories_path)

        # Extract joint names
        self.joint_names = self.get_joint_names()

        if not self.joint_names:
            self.get_logger().error("No joint names found. Ensure joint_states.yaml has at least one position.")
        else:
            self.get_logger().info(f"Joint names: {self.joint_names}")

        self.get_logger().info("Trajectory Manager Node initialized.")

    def load_yaml_file(self, file_path):
        """
        Loads a YAML file from the specified hard-coded path.

        Args:
            file_path (str): Absolute path to the YAML file to load.

        Returns:
            dict: Parsed YAML data, or empty dict if loading fails.
        """
        if not os.path.exists(file_path):
            if os.path.basename(file_path) == 'trajectory.yaml':
                self.get_logger().info(f"{os.path.basename(file_path)} not found at {file_path}. It will be created upon saving a trajectory.")
            else:
                self.get_logger().error(f"{os.path.basename(file_path)} not found at {file_path}. Please ensure it exists.")
            return {}

        with open(file_path, 'r') as file:
            try:
                data = yaml.safe_load(file) or {}
                self.get_logger().info(f"Loaded data from {os.path.basename(file_path)}")
                return data
            except yaml.YAMLError as e:
                self.get_logger().error(f"Error reading {os.path.basename(file_path)}: {e}")
                return {}

    def save_yaml_file(self, file_path, data):
        """
        Saves data to a YAML file at the specified hard-coded path.

        Args:
            file_path (str): Absolute path to the YAML file to save.
            data (dict): Data to save into the YAML file.
        """
        # Ensure the directory exists
        directory = os.path.dirname(file_path)
        os.makedirs(directory, exist_ok=True)

        with open(file_path, 'w') as file:
            yaml.dump(data, file, default_flow_style=False)
        self.get_logger().info(f"Saved data to {os.path.basename(file_path)}")

    def get_joint_names(self):
        """
        Extracts joint names from the first entry in joint_states.

        Returns:
            list: List of joint names.
        """
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
        """
        Callback function to handle incoming Trajectory messages for saving.

        Args:
            msg (plato2_interfaces.msg.Trajectory): The Trajectory message.
        """
        # Reload joint_states.yaml and trajectory.yaml to get the latest data
        self.joint_states = self.load_yaml_file(self.joint_states_path)
        self.trajectories = self.load_yaml_file(self.trajectories_path)

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
        self.save_yaml_file(self.trajectories_path, self.trajectories)

        self.get_logger().info(f"Trajectory '{trajectory_name}' saved successfully.")

    def execute_callback(self, msg):
        """
        Callback function to handle incoming execute commands.

        Args:
            msg (std_msgs.msg.String): The execute command message containing the trajectory name.
        """
        # Reload trajectory.yaml to get the latest trajectories
        self.trajectories = self.load_yaml_file(self.trajectories_path)

        trajectory_name = msg.data.strip()
        self.get_logger().info(f"Received command to execute trajectory: '{trajectory_name}'")

        if trajectory_name not in self.trajectories:
            self.get_logger().error(f"Trajectory '{trajectory_name}' not found in trajectory.yaml.")
            return

        trajectory_plan = self.trajectories[trajectory_name]

        with self.execution_lock:
            if self.execution_thread and self.execution_thread.is_alive():
                self.get_logger().warn("A trajectory is already being executed. Ignoring the new execute command.")
                return
            self.execution_thread = threading.Thread(target=self.execute_trajectory, args=(trajectory_plan,))
            self.execution_thread.start()

    def execute_trajectory(self, trajectory_plan):
        """
        Executes the given trajectory plan using minimum jerk trajectory planning.

        Args:
            trajectory_plan (list of dict): List of trajectory steps containing position names, motion times, and hold times.
        """
        try:
            # Reload joint_states.yaml to get the latest joint states
            self.joint_states = self.load_yaml_file(self.joint_states_path)

            # Start from zero position
            if 'zero_position' in self.joint_states:
                current_positions = self.joint_states['zero_position']
            else:
                current_positions = [0.0] * len(self.joint_names)
                self.get_logger().warning("Zero position not found in joint_states.yaml. Using zeros.")

            self.get_logger().info(f"Starting trajectory execution from zero position: {current_positions}")

            # Move to zero position first (if not already there)
            self.publish_position(current_positions)
            self.sleep(0.5)  # Non-blocking sleep

            # Iterate over the trajectory plan
            for step in trajectory_plan:
                if self.shutdown_flag:
                    self.get_logger().info("Trajectory execution interrupted by shutdown.")
                    break

                position_name = step['position_name']
                motion_time = step.get('Motion_Time', 0.0)
                hold_time = step.get('Hold_Time', 0.0)

                # Reload joint_states.yaml to get the latest target position
                self.joint_states = self.load_yaml_file(self.joint_states_path)

                if position_name not in self.joint_states:
                    self.get_logger().error(f"Position '{position_name}' not found in joint_states.yaml.")
                    return

                target_positions = self.joint_states[position_name]

                self.get_logger().info(f"Moving to '{position_name}' over {motion_time}s and holding for {hold_time}s.")

                # Generate minimum jerk trajectories
                list_t, list_x = self.minimum_jerk_trajectory(current_positions, target_positions, total_time=motion_time, dt=1.0/self.interpolation_rate)

                # Publish each step
                for pos in list_x:
                    if self.shutdown_flag:
                        self.get_logger().info("Trajectory execution interrupted by shutdown.")
                        break
                    self.publish_position(pos.tolist())
                    self.sleep(1.0/self.interpolation_rate)

                # Hold at target_positions for hold_time
                if hold_time > 0.0:
                    self.publish_position(target_positions)
                    self.get_logger().info(f"Holding position '{position_name}' for {hold_time}s.")
                    self.sleep(hold_time)

                # Update current positions
                current_positions = target_positions

            self.get_logger().info("Trajectory execution completed.")
        except Exception as e:
            self.get_logger().error(f"An error occurred during trajectory execution: {e}")

    def minimum_jerk_trajectory(self, init_pos, target_pos, total_time=0.5, dt=0.01):
        """
        Generates minimum jerk trajectories for multiple joints.

        Args:
            init_pos (list of float): Initial joint positions.
            target_pos (list of float): Target joint positions.
            total_time (float): Total time for the trajectory.
            dt (float): Time step.

        Returns:
            tuple: (list of time steps, numpy array of joint positions)
        """
        xi = np.array(init_pos)
        xf = np.array(target_pos)
        d = total_time
        list_t = []
        list_x = []
        t = 0.0
        while t < d:
            factor = 10*(t/d)**3 - 15*(t/d)**4 + 6*(t/d)**5
            x = xi + (xf - xi) * factor
            list_t.append(t)
            list_x.append(x)
            t += dt
        # Ensure the final position is exactly the target
        list_t.append(d)
        list_x.append(xf)
        return np.array(list_t), np.array(list_x)

    def publish_position(self, positions):
        """
        Publishes the given joint positions.

        Args:
            positions (list of float): Joint positions to publish.
        """
        msg = Float64MultiArray()
        msg.data = positions
        self.commands_publisher.publish(msg)
        self.get_logger().debug(f"Published positions: {positions}")

    def sleep(self, duration):
        """
        Non-blocking sleep using ROS 2's timing mechanisms.

        Args:
            duration (float): Duration to sleep in seconds.
        """
        end_time = self.get_clock().now() + rclpy.time.Duration(seconds=duration)
        while self.get_clock().now() < end_time:
            if self.shutdown_flag:
                break
            rclpy.spin_once(self, timeout_sec=0.1)

    def destroy_node(self):
        """
        Overrides the destroy_node method to ensure threads are properly terminated.
        """
        self.shutdown_flag = True
        with self.execution_lock:
            if self.execution_thread and self.execution_thread.is_alive():
                self.execution_thread.join()
        super().destroy_node()

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
