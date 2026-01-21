#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from plato_teleop.spacemouse_hardware import SpaceMouseHardware

class SpaceMouseStateMachine(Node):
    def __init__(self):
        super().__init__('spacemouse_state_machine')
        
        # Initialize state (0: open, 1: close, 2: poke)
        self.state = 0
        self.target_state = 0
        self.interpolating = False
        
        # Define joint positions for each state
        self.joint_positions = {
            10: np.array([  # open
                0.00286102294921875,
                0.00133514404296875,
                0.032806396484375,
                -0.1251476303239258,
                0.083707754611969,
                0.754594488293424,
                0.9632225036621094,
                1.4037198694571629
            ]),
            11: np.array([  # close
                0.00133514404296875,
                -0.00019073486328125,
                -0.352969017028808594,
                -0.4040302844949266,
                0.9857861161231995,
                0.4223854347322685,
                0.9754295349121094,
                1.418108409490415
            ]),
            2: np.array([  # poke
                0.00286102294921875,
                -0.00019073486328125,
                -0.25024986267089844,
                -1.4372527771779176,
                0.13466107845306396,
                -0.017743468140821506,
                1.100553035736084,
                1.5071229671464579
            ]),
            
            3: np.array([  # zero
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            ]),
            

            0: np.array([  # nail_open
                0.00286102294921875,
                0.00286102294921875,
                0.6729226112365723,
                -1.2608231371814327,
                0.09384220838546753,
                1.2681183860161411,
                0.9155378341674805,
                1.387665067687152
            ]),
            
            1: np.array([  # nail_close
                0.00743865966796875,
                0.00896453857421875,
                0.31509828567504883,
                -1.2420840956154677,
                0.42267388105392456,
                1.0886013086023638,
                0.915919303894043,
                1.3876337875911986
            ])
                         
                         
                    
        }
        # Fully open : 0.26
        # Fully closed : -0.00417
        
        
        # Current joint position (will be updated from joint_states)
        self.current_position = self.joint_positions[0].copy()
        
        # Joint names to match with joint_states (modify these to match your robot's joint names)
        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
            "joint8"
        ]
        
        # Flag to indicate if we've received joint states
        self.received_joint_states = False
        
        # Interpolation parameters
        self.interpolation_duration = 0.8  # Duration in seconds
        self.interpolation_steps = 500      # Number of steps for interpolation
        self.step_time = self.interpolation_duration / self.interpolation_steps

        # Initialize SpaceMouse hardware
        self.spacemouse = SpaceMouseHardware()
        if not self.spacemouse.open():
            self.get_logger().error('Failed to open SpaceMouse device')
            raise RuntimeError('SpaceMouse initialization failed')

        self.get_logger().info('SpaceMouse initialized successfully')

        # Create a timer to poll SpaceMouse buttons
        self.spacemouse_timer = self.create_timer(0.01, self.spacemouse_callback)
        
        # Create a subscription to joint states
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/plato2/joint_states',  # Adjust this topic as needed for your robot
            self.joint_states_callback,
            10
        )
        
        # Create a publisher for joint positions
        self.joint_publisher = self.create_publisher(
            Float64MultiArray,
            '/plato2/joint_position_controller/commands',
            10
        )
        
        # Create a timer for interpolation updates
        self.interpolation_timer = self.create_timer(
            self.step_time,  # Timer interval based on interpolation steps
            self.interpolation_callback
        )
        
        # Track button states to detect clicks (transitions from not pressed to pressed)
        self.prev_left_button = 0
        self.prev_right_button = 0
        
        # Interpolation tracking
        self.start_position = None
        self.target_position = None
        self.current_step = 0
        self.total_steps = 0
        
        self.get_logger().info(f'Initialized state machine - waiting for joint states')
        self.get_logger().info('Left button: Switch between open (0) and close (1)')
        self.get_logger().info('Right button: Switch to poke (2)')
        self.get_logger().info(f'Interpolation duration: {self.interpolation_duration} seconds')

    def joint_states_callback(self, msg):
        # Extract joint positions in the correct order based on joint names
        position_dict = {name: position for name, position in zip(msg.name, msg.position)}
        
        # Check if all our joints are in the message
        joint_positions = []
        for joint_name in self.joint_names:
            if joint_name in position_dict:
                joint_positions.append(position_dict[joint_name])
            else:
                self.get_logger().warn(f"Joint {joint_name} not found in joint_states")
                return  # Skip this callback if not all joints are present
        
        # Update current position
        self.current_position = np.array(joint_positions)
        
        # Set flag that we've received joint states
        if not self.received_joint_states:
            self.received_joint_states = True
            self.get_logger().info("Received initial joint states, ready for commands")
            # Optionally publish initial position
            # self.publish_joint_position(self.current_position)

    def spacemouse_callback(self):
        # Skip processing if we're already interpolating or haven't received joint states
        if self.interpolating or not self.received_joint_states:
            return

        # Read SpaceMouse state
        state = self.spacemouse.read()
        if state is None:
            return

        # Extract button states
        left_button = 1 if state.buttons[0] else 0
        right_button = 1 if state.buttons[1] else 0

        # Check for button clicks (transition from not pressed to pressed)
        left_click = (left_button == 1 and self.prev_left_button == 0)
        right_click = (right_button == 1 and self.prev_right_button == 0)

        # Update state based on button clicks
        state_changed = False

        if left_click:
            if self.state == 0:
                self.target_state = 1  # open -> close
                state_changed = True
                self.get_logger().info('Left button clicked: Switching to state 1 (close)')
            elif self.state == 1:
                self.target_state = 0  # close -> open
                state_changed = True
                self.get_logger().info('Left button clicked: Switching to state 0 (open)')
            elif self.state == 2 or self.state == 3:
                self.target_state = 0  # poke or zero -> open
                state_changed = True
                self.get_logger().info('Left button clicked: Switching to state 0 (open)')

        if right_click:
            if self.state == 2:
                self.target_state = 3  # poke -> zero
                state_changed = True
                self.get_logger().info('Right button clicked: Switching to state 3 (zero)')
            elif self.state == 3:
                self.target_state = 2  # zero -> poke
                state_changed = True
                self.get_logger().info('Right button clicked: Switching to state 2 (poke)')
            elif self.state in [0, 1]:
                self.target_state = 2  # open or close -> poke
                state_changed = True
                self.get_logger().info('Right button clicked: Switching to state 2 (poke)')

        # Start interpolation if state changed
        if state_changed:
            self.start_interpolation(self.state, self.target_state)

        # Update previous button states
        self.prev_left_button = left_button
        self.prev_right_button = right_button


    def start_interpolation(self, from_state, to_state):
        self.interpolating = True
        self.start_position = self.current_position.copy()  # Use current actual position
        self.target_position = self.joint_positions[to_state].copy()
        self.current_step = 0
        self.total_steps = self.interpolation_steps
        self.get_logger().info(f'Starting interpolation from current position to state {to_state}')
        
        # Log the start and target positions for debugging
        self.get_logger().debug(f'Start position: {self.start_position}')
        self.get_logger().debug(f'Target position: {self.target_position}')

    def interpolation_callback(self):
        if not self.interpolating:
            return
            
        # Increment step counter
        self.current_step += 1
        
        # Calculate progress (0.0 to 1.0)
        progress = self.current_step / self.total_steps
        
        # Apply smoothing using cubic easing for more natural motion
        smoothed_progress = self.cubic_ease_in_out(progress)
        
        # Interpolate joint positions
        interpolated_position = self.start_position + smoothed_progress * (self.target_position - self.start_position)
        
        # Publish interpolated position
        self.publish_joint_position(interpolated_position)
        
        # Check if interpolation is complete
        if self.current_step >= self.total_steps:
            self.state = self.target_state
            self.current_position = interpolated_position.copy()  # Update current position
            self.interpolating = False
            self.get_logger().info(f'Interpolation complete, now in state {self.state}')

    def cubic_ease_in_out(self, t):
        """
        Cubic easing function for smooth acceleration and deceleration
        """
        if t < 0.5:
            return 4 * t * t * t
        else:
            p = 2 * t - 2
            return 0.5 * p * p * p + 1

    def publish_joint_position(self, position):
        # Create message
        msg = Float64MultiArray()
        msg.data = position.tolist()

        # Publish message
        self.joint_publisher.publish(msg)

        # For debugging (commented out to avoid log spam)
        # self.get_logger().debug(f'Published position: {position}')

    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        self.spacemouse.close()
        self.get_logger().info('SpaceMouse closed successfully')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = SpaceMouseStateMachine()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
