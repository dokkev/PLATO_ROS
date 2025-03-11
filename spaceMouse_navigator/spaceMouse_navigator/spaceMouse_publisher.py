import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pyspacemouse

class SpaceMousePublisher(Node):
    def __init__(self):
        super().__init__('space_mouse_publisher')
        self.publisher = self.create_publisher(Joy, 'spaceMouseMotion', 10)
        
        # Try to open the SpaceMouse device
        success = pyspacemouse.open()
        if not success:
            self.get_logger().error('Failed to open SpaceMouse')
            exit(1)
        
        self.get_logger().info('SpaceMouse initialized successfully')
        
        # Keep track of previous button states to detect changes
        self.prev_button_states = [False, False]
        
        # Create a timer with a callback frequency of ~900 Hz (0.0011 seconds)
        self.timer = self.create_timer(0.0011, self.publish_space_mouse_data)

    def publish_space_mouse_data(self):
        # Read the current state of the SpaceMouse
        state = pyspacemouse.read()
        if state is None:
            self.get_logger().warn('Failed to read SpaceMouse state')
            return
            
        # Create a Joy message
        joy_msg = Joy()
        
        # Set the axes values (x, y, z, pitch, roll, yaw)
        # Note: We negate pitch and yaw for a more intuitive mapping
        joy_msg.axes = [
            float(state.y), 
            float(-state.x), 
            float(state.z), 
            float(state.roll), 
            float(state.pitch), 
            float(-state.yaw)
        ]
        
        # Get button states and add them to the message
        # The Joy message expects an array of integers (0 or 1)
        joy_msg.buttons = [
            1 if state.buttons[0] else 0,  # Left button
            1 if state.buttons[1] else 0   # Right button
        ]
        
        # Log button presses (only when they change state)
        if self.prev_button_states[0] != state.buttons[0]:
            self.get_logger().info(f'Left button: {"pressed" if state.buttons[0] else "released"}')
        
        if self.prev_button_states[1] != state.buttons[1]:
            self.get_logger().info(f'Right button: {"pressed" if state.buttons[1] else "released"}')
        
        # Update previous button states
        self.prev_button_states = state.buttons.copy()
        
        # Set the timestamp
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Publish the message
        self.publisher.publish(joy_msg)
        
        # Uncomment for debugging (note: this will generate a lot of output)
        # self.get_logger().info(f'Published: axes={joy_msg.axes}, buttons={joy_msg.buttons}')

def main(args=None):
    rclpy.init(args=args)
    space_mouse_publisher = SpaceMousePublisher()
    
    try:
        # Use rclpy.spin() to keep the node running
        rclpy.spin(space_mouse_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        space_mouse_publisher.destroy_node()
        rclpy.shutdown()
        pyspacemouse.close()
        print("SpaceMouse node shut down successfully")

if __name__ == '__main__':
    main()