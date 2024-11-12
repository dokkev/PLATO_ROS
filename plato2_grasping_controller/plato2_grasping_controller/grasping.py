import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Grasping(Node):
    def __init__(self):
        super().__init__('my_python_node')
        
        # Subscriber to "input_topic"
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.listener_callback,
            10  # QoS history depth
        )
        self.subscription  # prevent unused variable warning
        
        # Publisher to "output_topic"
        self.publisher = self.create_publisher(String, 'output_topic', 10)

        # Timer to publish messages periodically
        self.timer = self.create_timer(1.0, self.publish_message)
        
        # pre-defiend target joint positions
        self.open_positions = {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.362,
            'joint4': -0.723,
            'joint5': -0.43,
            'joint6': 1.411,
            'joint7': 1.038,
            'joint8': 0.832
        }
        self.close_positions = {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.362,
            'joint4': -0.723,
            'joint5': 0.607,
            'joint6': 0.377,
            'joint7': 1.038,
            'joint8': 0.832
        }

    def listener_callback(self, msg: String):
        self.get_logger().info(f'Received message: "{msg.data}"')

    def publish_message(self):
        msg = String()
        msg.data = 'Hello, this is a message from the publisher.'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published message: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    # node = MyNode()

    # Spin the node so it keeps running and processing callbacks
    # rclpy.spin(node)

    # Shutdown ROS 2 when done
    # node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
