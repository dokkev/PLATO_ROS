import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pyspacemouse

class SpaceMousePublisher(Node):
    def __init__(self):
        super().__init__('space_mouse_publisher')
        self.publisher = self.create_publisher(Joy, 'spaceMouseMotion', 10)
        success = pyspacemouse.open()
        if not success:
            self.get_logger().error('Failed to open SpaceMouse')
            exit(1)
        # Create a timer with a callback frequency of 100 Hz
        self.timer = self.create_timer(0.01, self.publish_space_mouse_data)

    def publish_space_mouse_data(self):
        state = pyspacemouse.read()
        joy_msg = Joy()
        joy_msg.axes = [float(state.x), float(state.y), float(state.z), float(state.roll), float(state.yaw), float(state.pitch)]
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(joy_msg)
        # self.get_logger().info(f'Published: {joy_msg.axes}')

def main(args=None):
    rclpy.init(args=args)
    space_mouse_publisher = SpaceMousePublisher()
    try:
        # Use rclpy.spin() instead of a while loop
        rclpy.spin(space_mouse_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        space_mouse_publisher.destroy_node()
        rclpy.shutdown()
        pyspacemouse.close()

if __name__ == '__main__':
    main()
