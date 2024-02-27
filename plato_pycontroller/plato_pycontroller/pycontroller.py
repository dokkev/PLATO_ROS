#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pinocchio as pin
import numpy as np
import tf2_ros
import tf_transformations

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
            Float64MultiArray,
            'plato/plato_effort_controller/commands',
            10)
    
        # TF2 listener for obtaining transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.urdf_path = '/home/optimo/workspaces/plato_ws/src/PLATO_ROS/plato_description/urdf/plato.urdf'  # Update this path
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()

        # PD Controller Gains
        self.kp = np.array([0.01, 0.0001, 0.0001, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
        self.kd = np.array([0.00, 0.001, 0.001, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
        self.ki = np.array([0.01, 0.00 ,0.00, 0.01, 0.00, 0.00, 0.01, 0.00, 0.00])

        # Initialize joint states
        self.q = np.zeros(9)  # Current positions
        self.q_prev = np.zeros(9)  # Previous positions
        self.v = np.zeros(9)  # Estimated velocities

        self.q = np.array([0, -0.9, 0.9, 0, 0.44, 0.44, 0, 0.44, 0.44])

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
        # adjust gravity vector based on the current orientation of the robot
        adjusted_gravity = self.get_gravity_vector_in_base_frame()
        self.model.gravity = pin.Motion(adjusted_gravity, np.array([0, 0, 0]))
        self.tau_gravity = pin.rnea(self.model, self.data, self.q, self.v, np.zeros(9))

        command_msg = Float64MultiArray()
        command_msg.data = self.tau_gravity.tolist()

        self.torque_publisher.publish(command_msg)

        

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
        command_msg = Float64MultiArray()
        command_msg.data = tau_total.tolist()

        self.torque_publisher.publish(command_msg)



    def get_gravity_vector_in_base_frame(self):
        try:
            # Look up the transform from 'world' frame to 'plato_base_link' of the robot
            transform = self.tf_buffer.lookup_transform('world', 'plato_base_link', rclpy.time.Time())
            
            # Extract the rotation from the transform and convert it to a rotation matrix
            quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )
            rotation_matrix = tf_transformations.quaternion_matrix(quaternion)[:3, :3]
            
            # Define the gravity vector in the 'world' frame
            gravity_world = np.array([0, 0, -9.8])
            
            # Adjust the gravity vector based on the rotation from 'world' to 'base_link'
            gravity_base = np.dot(rotation_matrix, gravity_world)
            
            return gravity_base

        except Exception as e:
            self.get_logger().error('Failed to adjust gravity vector: %s' % str(e))
            return np.array([0, 0, -10.00])  # Fallback to standard gravity if the transformation fails


def main(args=None):
    rclpy.init(args=args)
    plato_controller = PlatoController()
    rclpy.spin(plato_controller)
    plato_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
