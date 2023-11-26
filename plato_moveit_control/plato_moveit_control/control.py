#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Generic ROS libraries
import rclpy
from rclpy.logging import get_logger
# RobotState is used to set joint values
from moveit.core.robot_state import RobotState

# MoveIt python library
from moveit.planning import (
    MoveItPy,
)

def main():

    # Initialize rclpy and ROS logger
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")
    # print("Hello world")

    # Instantiate MoveItPy and get planning component
    plato = MoveItPy(node_name="moveit_py")
    # optimo_arm = plato.get_planning_component("optimo_arm")

    # # Create a robot state to set the joint values
    # robot_model = plato.get_robot_model()
    # robot_state = RobotState(robot_model)

    # # Set plan start state to current state
    # optimo_arm.set_start_state_to_current_state()

    # # Set target joint values
    # joint_values = {
    #     "optimo_joint1": -1.0,
    #     "optimo_joint2": 0.7,
    #     "optimo_joint3": 0.7,
    #     "optimo_joint4": -1.5,
    #     "optimo_joint5": -0.7,
    #     "optimo_joint6": 2.0,
    #     "optimo_joint7": 0.0,
    # }

    # robot_state.joint_positions = joint_values

    # # Set the joint values as the goal state
    # optimo_arm.set_goal_state(robot_state=robot_state)

    # # Create a plan to the target pose
    # plan_result = optimo_arm.plan()

    # # If the plan is successful, get the trajectory and execute the plan
    # if plan_result:
    #     robot_trajectory = plan_result.trajectory
    #     plato.execute(robot_trajectory, blocking=True, controllers=[])

    # else:
    #     logger.error("Planning failed")

    rclpy.shutdown()
    
if __name__ == "__main__":
    main()