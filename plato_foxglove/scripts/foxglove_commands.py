#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from plato_interfaces.msg import SavedNames, ImpedanceCommands, ImpedanceParams
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
        
        # Subscriber for joint state name to delete
        self.delete_subscriber = self.create_subscription(
            String,
            'delete_joint_state',
            self.delete_callback,
            10
        )
        
        # Publisher for impedance commands to our controller
        self.commands_publisher = self.create_publisher(
            ImpedanceCommands,
            '/plato2/joint_impedance_controller/commands',
            10
        )
        
        # Subscriber for compact impedance params (MCP/PIP) on a single topic
        self.params_subscriber = self.create_subscription(
            ImpedanceParams,
            'impedance_params',  # resolves to <ns>/impedance_params
            self.params_callback,
            10
        )
        self.get_logger().info(
            f"Listening for ImpedanceParams on: {self.get_namespace()}/impedance_params")
        
        # Publisher for SavedNames
        self.saved_names_publisher = self.create_publisher(
            SavedNames,
            'saved_joint_names',
            10
        )
        
        # Timer to periodically publish saved names (optional)
        self.saved_names_timer = self.create_timer(
            1.0,  # Publish every 1 seconds
            self.publish_saved_names
        )
        
        self.joint_state_subscriber = None
        self.joint_state_name = None
        self.joint_state_data = None

        # Cached command state
        self.NUM_JOINTS = 8
        self.cached_position = [0.0] * self.NUM_JOINTS
        # Defaults for compact params (MCP, PIP)
        self.cached_params = {
            'stiffness': [0.0, 0.0],
            'damping': [0.0, 0.0],
            'effort_ff': [0.0, 0.0],
        }
        self.get_logger().info("Joint State Recorder Node initialized.")

    def string_callback(self, msg):
        self.joint_state_name = msg.data
        self.get_logger().info(f"Received string for recording: {self.joint_state_name}")
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
        # Publish updated saved names after saving new data
        self.publish_saved_names()

    def save_to_yaml(self):
        if self.joint_state_name and self.joint_state_data is not None:
            # Prepare data to save
            data_to_save = {self.joint_state_name: list(self.joint_state_data)}

            script_dir = os.path.dirname(os.path.abspath(__file__))
            ws_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_dir))))
            file_path = os.path.join(ws_dir, 'src', 'PLATO_ROS', 'plato_foxglove', 'config', 'joint_states.yaml')
                
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
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        ws_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_dir))))
        file_path = os.path.join(ws_dir, 'src', 'PLATO_ROS', 'plato_foxglove', 'config', 'joint_states.yaml')

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
        """
        Update cached position from provided array and publish a full ImpedanceCommands
        using cached parameters for stiffness/damping/effort_ff.
        """
        positions = list(joint_state)[:self.NUM_JOINTS]
        if len(positions) < self.NUM_JOINTS:
            positions.extend([0.0] * (self.NUM_JOINTS - len(positions)))
        self.cached_position = positions
        self.publish_current_command()

    def params_callback(self, msg: ImpedanceParams):
        """Update cached compact params and publish the full command."""
        # Accept any iterable/sequence-like; pad/trim to 2
        def get_pair(arr):
            try:
                a = list(arr) if arr is not None else []
            except Exception:
                a = []
            if len(a) < 2:
                a.extend([0.0] * (2 - len(a)))
            a = a[:2]
            return [float(a[0]), float(a[1])]

        self.cached_params['stiffness'] = get_pair(msg.stiffness)
        self.cached_params['damping'] = get_pair(msg.damping)
        self.cached_params['effort_ff'] = get_pair(msg.effort_ff)

        self.get_logger().info(
            f"Received params MCP/PIP -> K={self.cached_params['stiffness']}, B={self.cached_params['damping']}, FF={self.cached_params['effort_ff']}")
        self.publish_current_command()

    def publish_current_command(self):
        """
        Build a full ImpedanceCommands from cached_position and cached_params.
        Mapping:
          MCP joints: indices [2,4,6]
          PIP joints: indices [3,5,7]
          Joints 0,1: fixed stiffness=3.0, damping=0.2; effort_ff defaults 0.0
        Velocity is zeroed.
        """
        K_mcp, K_pip = self.cached_params['stiffness']
        B_mcp, B_pip = self.cached_params['damping']
        FF_mcp, FF_pip = self.cached_params['effort_ff']

        K = [0.0] * self.NUM_JOINTS
        B = [0.0] * self.NUM_JOINTS
        FF = [0.0] * self.NUM_JOINTS

        # Fixed thumb/base gains (indices 0,1)
        K[0] = 3.0; K[1] = 3.0
        B[0] = 0.2; B[1] = 0.2
        # Effort feedforward for 0,1 remains 0.0 by default

        # MCP joints (2,4,6)
        for idx in [2, 4, 6]:
            K[idx] = K_mcp
            B[idx] = B_mcp
            FF[idx] = FF_mcp
        # PIP joints (3,5,7)
        for idx in [3, 5, 7]:
            K[idx] = K_pip
            B[idx] = B_pip
            FF[idx] = FF_pip

        cmd = ImpedanceCommands()
        cmd.position = list(self.cached_position)
        cmd.velocity = [0.0] * self.NUM_JOINTS
        cmd.stiffness = K
        cmd.damping = B
        cmd.effort_ff = FF

        self.commands_publisher.publish(cmd)
        self.get_logger().info(
            f"Published ImpedanceCommands from cache: pos={cmd.position}, K0-1=3,B0-1=0.2, K_mcp={K_mcp}, K_pip={K_pip}")

    def publish_saved_names(self):
        """
        Reads the joint_states.yaml file, extracts the joint state names,
        and publishes them as a SavedNames message formatted as a 1xn array.
        """
        script_dir = os.path.dirname(os.path.abspath(__file__))
        ws_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_dir))))
        file_path = os.path.join(ws_dir, 'src', 'PLATO_ROS', 'plato_foxglove', 'config', 'joint_states.yaml')

        # Check if the YAML file exists
        if not os.path.exists(file_path):
            self.get_logger().error(f"YAML file not found at {file_path}. Cannot publish saved names.")
            return

        # Read the YAML file
        with open(file_path, 'r') as file:
            try:
                data = yaml.safe_load(file) or {}
            except yaml.YAMLError:
                self.get_logger().error("Error reading YAML file. Cannot publish saved names.")
                return

        # Extract the joint state names (keys)
        saved_names = list(data.keys())

        # Create and populate the SavedNames message
        saved_names_msg = SavedNames()
        saved_names_msg.saved_names = saved_names  # This is inherently a 1xn array in ROS2

        # Publish the SavedNames message
        self.saved_names_publisher.publish(saved_names_msg)
        # self.get_logger().info(f"Published saved joint names: {saved_names}")

    def delete_callback(self, msg):
        """
        Callback function to delete a saved joint state by name.
        """
        name_to_delete = msg.data
        self.get_logger().info(f"Received request to delete joint state: {name_to_delete}")

        script_dir = os.path.dirname(os.path.abspath(__file__))
        ws_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_dir))))
        file_path = os.path.join(ws_dir, 'src', 'PLATO_ROS', 'plato_foxglove', 'config', 'joint_states.yaml')

        # Check if the YAML file exists
        if not os.path.exists(file_path):
            self.get_logger().error(f"YAML file not found at {file_path}. Cannot delete joint state.")
            return

        # Read the YAML file
        with open(file_path, 'r') as file:
            try:
                data = yaml.safe_load(file) or {}
            except yaml.YAMLError:
                self.get_logger().error("Error reading YAML file. Cannot delete joint state.")
                return

        # Check if the name exists
        if name_to_delete in data:
            # Delete the entry
            del data[name_to_delete]
            self.get_logger().info(f"Deleted joint state '{name_to_delete}' from YAML file.")
            
            # Write back the updated content
            with open(file_path, 'w') as file:
                yaml.dump(data, file, default_flow_style=False)
            
            self.get_logger().info(f"Updated YAML file at {file_path} after deletion.")
            
            # Publish the updated saved names
            self.publish_saved_names()
        else:
            self.get_logger().error(f"Joint state '{name_to_delete}' does not exist in YAML file.")

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
