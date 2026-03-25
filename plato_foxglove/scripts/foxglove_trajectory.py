#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from plato_interfaces.msg import Trajectory, SavedNames, ImpedanceCommands, ImpedanceParams
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

        # Subscriber for trajectory deletion commands
        self.delete_subscriber = self.create_subscription(
            String,
            '/plato2/trajectory_delete',
            self.delete_callback,
            10
        )

        # Publisher for impedance commands to the controller
        self.commands_publisher = self.create_publisher(
            ImpedanceCommands,
            '/plato2/joint_impedance_controller/commands',
            10
        )

        # Publisher for SavedNames (trajectory names)
        self.saved_trajectories_publisher = self.create_publisher(
            SavedNames,
            '/plato2/saved_trajectories',
            10
        )

        # Timer to periodically publish saved trajectory names (optional)
        self.saved_trajectories_timer = self.create_timer(
            1.0,  # Publish every 1 seconds
            self.publish_saved_trajectories
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
        file_path = os.path.join(ws_dir, 'src', 'PLATO_ROS', 'plato_foxglove', 'config', file_name)

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
        file_path = os.path.join(ws_dir, 'src', 'PLATO_ROS', 'plato_foxglove', 'config', file_name)
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

        # Validate message lengths (params can be length 0/1/N)
        if not (len(position_names) == len(motion_times) == len(hold_times)):
            self.get_logger().error("Lengths of position_name, motion_time, and hold_time arrays must be equal.")
            return

        # Normalize params list
        params_list = list(msg.params) if hasattr(msg, 'params') and msg.params is not None else []
        if len(params_list) == 1:
            params_list = params_list * len(position_names)
        elif len(params_list) == 0:
            # default zeros
            zero = ImpedanceParams(stiffness=[0.0, 0.0], damping=[0.0, 0.0], effort_ff=[0.0, 0.0])
            params_list = [zero] * len(position_names)
        elif len(params_list) != len(position_names):
            self.get_logger().error("Length of params must be 0, 1, or match position_name length.")
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
            # Add ImpedanceParams for this step
            ip = params_list[i]
            step['ImpedanceParams'] = {
                'stiffness': list(ip.stiffness) if ip.stiffness is not None else [0.0, 0.0],
                'damping': list(ip.damping) if ip.damping is not None else [0.0, 0.0],
                'effort_ff': list(ip.effort_ff) if ip.effort_ff is not None else [0.0, 0.0],
            }
            trajectory_steps.append(step)

            self.get_logger().debug(f"Added step: {trajectory_steps[-1]}")

        # Save the new trajectory
        self.trajectories[trajectory_name] = trajectory_steps
        self.save_yaml_file('trajectory.yaml', self.trajectories)

        self.get_logger().info(f"Trajectory '{trajectory_name}' saved successfully.")

        # Publish updated saved trajectories
        self.publish_saved_trajectories()

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
        # Validate that there are trajectory steps to execute
        if not trajectory_plan:
            self.get_logger().error("No steps in the trajectory plan to execute.")
            return

        # Use the initial position from the first step in the trajectory plan
        current_positions = self.joint_states[trajectory_plan[0]['position_name']]

        # Iterate over the trajectory plan
        for step in trajectory_plan:
            position_name = step['position_name']
            motion_time = step.get('Motion_Time', 0.0)
            hold_time = step.get('Hold_Time', 0.0)

            if position_name not in self.joint_states:
                self.get_logger().error(f"Position '{position_name}' not found in joint_states.yaml.")
                return

            target_positions = self.joint_states[position_name]
            # Extract ImpedanceParams for this step (MCP/PIP compact)
            ip = step.get('ImpedanceParams', {'stiffness':[0.0,0.0], 'damping':[0.0,0.0], 'effort_ff':[0.0,0.0]})
            K_pair = ip.get('stiffness', [0.0, 0.0])
            B_pair = ip.get('damping', [0.0, 0.0])
            FF_pair = ip.get('effort_ff', [0.0, 0.0])

            self.get_logger().info(f"Moving to '{position_name}' over {motion_time}s and holding for {hold_time}s.")

            # Interpolate from current_positions to target_positions over motion_time using minimum jerk
            self.interpolate_and_publish(current_positions, target_positions, motion_time, K_pair, B_pair, FF_pair)

            # Hold at target_positions for hold_time
            if hold_time > 0.0:
                self.publish_command(target_positions, K_pair, B_pair, FF_pair)
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

    def interpolate_and_publish(self, start_positions, end_positions, duration, K_pair, B_pair, FF_pair):
        if duration <= 0.0:
            # Immediate move to the target position
            self.publish_command(end_positions, K_pair, B_pair, FF_pair)
            return

        dt = 0.01  # Time step (seconds)
        interpolated_positions = self.minimum_jerk_trajectory(start_positions, end_positions, total_time=duration, dt=dt)

        self.get_logger().debug(f"Interpolating using minimum jerk over {len(interpolated_positions)} steps with {dt}s between steps.")

        for pos in interpolated_positions:
            self.publish_command(pos, K_pair, B_pair, FF_pair)
            time.sleep(dt)

    def publish_command(self, positions, K_pair, B_pair, FF_pair):
        # Build full ImpedanceCommands
        NUM_JOINTS = len(positions)
        K_mcp, K_pip = (list(K_pair) + [0.0, 0.0])[:2]
        B_mcp, B_pip = (list(B_pair) + [0.0, 0.0])[:2]
        FF_mcp, FF_pip = (list(FF_pair) + [0.0, 0.0])[:2]

        K = [0.0] * NUM_JOINTS
        B = [0.0] * NUM_JOINTS
        FF = [0.0] * NUM_JOINTS

        # Fixed joints 0,1 gains/feedforward
        if NUM_JOINTS >= 2:
            K[0] = 3.0; K[1] = 3.0
            B[0] = 0.2; B[1] = 0.2
            FF[0] = 0.0; FF[1] = 0.0

        # Map MCP -> [2,4,6], PIP -> [3,5,7]
        for idx in [2, 4, 6]:
            if idx < NUM_JOINTS:
                K[idx] = K_mcp; B[idx] = B_mcp; FF[idx] = FF_mcp
        for idx in [3, 5, 7]:
            if idx < NUM_JOINTS:
                K[idx] = K_pip; B[idx] = B_pip; FF[idx] = FF_pip

        cmd = ImpedanceCommands()
        cmd.position = list(positions)
        cmd.velocity = [0.0] * NUM_JOINTS
        cmd.stiffness = K
        cmd.damping = B
        cmd.effort_ff = FF

        self.commands_publisher.publish(cmd)
        # self.get_logger().debug(f"Published ImpedanceCommands: pos={positions}")

    def publish_saved_trajectories(self):
        """
        Reads the trajectory.yaml file, extracts the trajectory names,
        and publishes them as a SavedNames message formatted as a 1xn array.
        """
        script_dir = os.path.dirname(os.path.abspath(__file__))
        ws_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_dir))))
        file_path = os.path.join(ws_dir, 'src', 'PLATO_ROS', 'plato_foxglove', 'config', 'trajectory.yaml')

        # Check if the YAML file exists
        if not os.path.exists(file_path):
            self.get_logger().error(f"trajectory.yaml file not found at {file_path}. Cannot publish saved trajectories.")
            return

        # Read the YAML file
        with open(file_path, 'r') as file:
            try:
                data = yaml.safe_load(file) or {}
            except yaml.YAMLError:
                self.get_logger().error("Error reading trajectory.yaml. Cannot publish saved trajectories.")
                return

        # Extract the trajectory names (keys)
        saved_trajectories = list(data.keys())

        # Create and populate the SavedNames message
        saved_trajectories_msg = SavedNames()
        saved_trajectories_msg.saved_names = saved_trajectories  # This is inherently a 1xn array in ROS2

        # Publish the SavedNames message
        self.saved_trajectories_publisher.publish(saved_trajectories_msg)
        # self.get_logger().info(f"Published saved trajectory names: {saved_trajectories}")

    def delete_callback(self, msg):
        """
        Callback function to delete a saved trajectory by name.
        """
        trajectory_to_delete = msg.data.strip()
        self.get_logger().info(f"Received request to delete trajectory: '{trajectory_to_delete}'")

        script_dir = os.path.dirname(os.path.abspath(__file__))
        ws_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_dir))))
        file_path = os.path.join(ws_dir, 'src', 'PLATO_ROS', 'plato_foxglove', 'config', 'trajectory.yaml')

        # Check if the YAML file exists
        if not os.path.exists(file_path):
            self.get_logger().error(f"trajectory.yaml file not found at {file_path}. Cannot delete trajectory.")
            return

        # Read the YAML file
        with open(file_path, 'r') as file:
            try:
                data = yaml.safe_load(file) or {}
            except yaml.YAMLError:
                self.get_logger().error("Error reading trajectory.yaml. Cannot delete trajectory.")
                return

        # Check if the trajectory exists
        if trajectory_to_delete in data:
            # Delete the trajectory
            del data[trajectory_to_delete]
            self.get_logger().info(f"Deleted trajectory '{trajectory_to_delete}' from trajectory.yaml.")

            # Write back the updated content
            with open(file_path, 'w') as file:
                yaml.dump(data, file, default_flow_style=False)

            self.get_logger().info(f"Updated trajectory.yaml after deletion.")

            # Publish the updated saved trajectories
            self.publish_saved_trajectories()
        else:
            self.get_logger().error(f"Trajectory '{trajectory_to_delete}' does not exist in trajectory.yaml.")

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
