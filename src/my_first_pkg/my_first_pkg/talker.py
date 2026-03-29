import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        # Declare a parameter with default value
        self.declare_parameter('publish_rate', 1.0)

        # Read parameter
        self.publish_rate = self.get_parameter('publish_rate').value

        self.pub = self.create_publisher(String, 'hello_topic', 10)

        # Timer uses parameter
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.tick
        )

        self.i = 0

    def tick(self):
        msg = String()
        msg.data = f'Hello ROS 2! {self.i}'

        # IMPORTANT: actually publish the message
        self.pub.publish(msg)

        # Log to terminal
        self.get_logger().info(f'Published: {msg.data}')

        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
