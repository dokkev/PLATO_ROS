#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from example_interfaces.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np

class HandTriggerController(Node):
    def __init__(self):
        super().__init__('hand_trigger_controller')

        # ——————————————————————————————
        # 1) Pre‐stored “open” and “close” joint vectors
        # ——————————————————————————————
        self.joint_positions = {
            0: np.array([  # open
                0.00286102294921875,
                0.00133514404296875,
                0.032806396484375,
                -0.1251476303239258,
                0.083707754611969,
                0.754594488293424,
                0.9632225036621094,
                1.4037198694571629
            ]),
            1: np.array([  # close
                0.00133514404296875,
                -0.00019073486328125,
                -0.202969017028808594,
                -0.3540302844949266,
                0.9857861161231995,
                0.4223854347322685,
                0.9754295349121094,
                1.418108409490415
            ]),
        }

        # Names of your 8 joints (for synchronizing with /joint_states)
        self.joint_names = [
            "joint1","joint2","joint3","joint4",
            "joint5","joint6","joint7","joint8"
        ]

        # These define the range of Float64 “trigger” values:
        self.open_val = 0.26    # when trigger = 0.26 → fully “open” (alpha=0)
        self.close_val = 0.004  # when trigger = 0.004 → fully “close” (alpha=1)

        # Keep track of the actual robot state, in case you want to sanity‐check
        self.received_joint_states = False
        self.current_position = self.joint_positions[0].copy()  # initialize as “open”

        # ——————————————————————————————
        # 2) Subscriptions & Publishers
        # ——————————————————————————————
        # Subscribe to /joint_states just to update self.current_position if needed
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/plato2/joint_states',
            self.joint_states_callback,
            10
        )

        # Subscribe to the Float64 “trigger” topic (e.g. /hand_trigger)
        self.trigger_sub = self.create_subscription(
            Float64,
            '/robot/feedback/gripper_angle',            # <— change this to your actual trigger topic
            self.trigger_callback,
            10
        )

        # Publisher for the interpolated joint angles
        self.joint_publisher = self.create_publisher(
            Float64MultiArray,
            '/plato2/plato2_position_controller/commands',
            10
        )

        self.get_logger().info('HandTriggerController initialized.')
        self.get_logger().info(f'Expect Float64 trigger ∈ [{self.close_val:.3f} .. {self.open_val:.3f}].')
        self.get_logger().info('Upon each trigger update, publishing interpolated joint commands.')

    def joint_states_callback(self, msg: JointState):
        # Update self.current_position whenever a JointState arrives.
        # This is optional; you only need it if you want to read back “where you are” for safety checks.
        position_dict = {n: p for n, p in zip(msg.name, msg.position)}
        joint_positions = []
        for jn in self.joint_names:
            if jn in position_dict:
                joint_positions.append(position_dict[jn])
            else:
                self.get_logger().warn(f"Joint '{jn}' not in /joint_states; skipping update.")
                return
        self.current_position = np.array(joint_positions)
        if not self.received_joint_states:
            self.received_joint_states = True
            self.get_logger().info("Received first /joint_states. Ready to accept trigger values.")

    def trigger_callback(self, msg: Float64):
        if not self.received_joint_states:
            # If we haven’t seen /joint_states yet, do nothing.
            return

        # 1) Read raw trigger value:
        trigger = msg.data

        # 2) Clamp to [close_val, open_val]:
        #    (In case the hardware sends a tiny bit outside.)
        trigger = float(np.clip(trigger, self.close_val, self.open_val))

        # 3) Normalize α = (open_val - trigger) / (open_val - close_val)
        #    → when trigger=open_val → α=0; trigger=close_val → α=1
        denom = (self.open_val - self.close_val)
        alpha = (self.open_val - trigger) / denom
        alpha = float(np.clip(alpha, 0.0, 1.0))

        # 4) Interpolate between joint_positions[0] and joint_positions[1]:
        q_open  = self.joint_positions[0]
        q_close = self.joint_positions[1]
        # q_des   = (1.0 - alpha)*q_open + alpha*q_close
        
        alpha_eased = self.cubic_ease_in_out(alpha)
        q_des = (1.0 - alpha_eased)*q_open + alpha_eased*q_close


        # 5) Publish q_des as a Float64MultiArray
        out_msg = Float64MultiArray()
        out_msg.data = q_des.tolist()
        self.joint_publisher.publish(out_msg)

        # Debug log at INFO or DEBUG level:
        self.get_logger().debug(
            f"Trigger={trigger:.4f}  →  α={alpha:.3f}  →  q_des={np.round(q_des, 4).tolist()}"
        )

    def cubic_ease_in_out(self, t: float) -> float:
        # (Optional) if you want to ease the interpolation
        # between 0→1.0 more naturally:
        if t < 0.5:
            return 4 * t*t*t
        else:
            p = 2*t - 2
            return 0.5 * p*p*p + 1

def main(args=None):
    rclpy.init(args=args)
    node = HandTriggerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
