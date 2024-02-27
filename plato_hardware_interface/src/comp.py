#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import pinocchio as pin
import numpy as np

class PlatoController(Node):
    def __init__(self):
        super().__init__('plato_controller')
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'plato/joint_states',
            self.joint_states_callback,
            10)
        self.joint_trajectory_subscription = self.create_subscription(
            JointTrajectory,
            'plato/plato_joint_controller/joint_trajectory',
            self.joint_trajectory_callback,
            10)
        self.torque_publisher = self.create_publisher(
            JointState,
            'plato/plato_effort_controller/commands',
            10)

        self.urdf_path = "/path/to/your/robot.urdf"  # Update this path
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()

        # PD Controller Gains
        self.kp = np.array([1.0] * 9)  # Proportional gains
        self.kd = np.array([0.1] * 9)  # Derivative gains

        # Initialize joint states
        self.q = np.zeros(9)  # Current positions
        self.q_prev = np.zeros(9)  # Previous positions
        self.v = np.zeros(9)  # Estimated velocities

        # Time management
        self.prev_time = self.get_clock().now()

    def joint_states_callback(self, msg):
        # Update current positions
        self.q = np.array(msg.position)

        # Calculate time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9  # Convert to seconds

        if dt > 0:
            # Estimate velocities
            self.v = (self.q - self.q_prev) / dt

        # Update previous positions and time for the next iteration
        self.q_prev = self.q
        self.prev_time = current_time

        # Compute gravity compensation
        self.tau_gravity = pin.rnea(self.model, self.data, self.q, self.v, np.zeros(9))

    def joint_trajectory_callback(self, msg):
        # Assuming the first point in the trajectory is the target
        target = msg.points[0]
        q_desired = np.array(target.positions)

        # Compute position error
        position_error = q_desired - self.q

        # PD control effort
        tau_pd = self.kp * position_error - self.kd * self.v

        # Total torque: PD control + Gravity compensation
        tau_total = tau_pd + self.tau_gravity

        # Publish the total torque as a command
        command_msg = JointState()
        command_msg.header.stamp = self.get_clock().now().to_msg()
        command_msg.effort = tau_total.tolist()
        self.torque_publisher.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    plato_controller = PlatoController()
    rclpy.spin(plato_controller)
    plato_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
