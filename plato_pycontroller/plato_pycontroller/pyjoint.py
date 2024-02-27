#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher = self.create_publisher(JointTrajectory, 'plato/plato_joint_controller/joint_trajectory', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = ['plato_joint0', 'plato_joint1', 'plato_joint2', 'plato_joint3', 'plato_joint4', 'plato_joint5', 'plato_joint6', 'plato_joint7', 'plato_joint8']

        point = JointTrajectoryPoint()
        point.positions = [0.0, -0.9, 0.9, 0.0, 0.44, 0.44, 0.0, 0.44, 0.44]
        point.velocities = []
        point.accelerations = []
        point.effort = []
        point.time_from_start = Duration(seconds=5).to_msg()



        msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_publisher = JointTrajectoryPublisher()
    rclpy.spin(joint_trajectory_publisher)
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
