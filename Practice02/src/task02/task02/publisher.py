import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')

        # Parameters required by the task/spec
        self.declare_parameter('topic_name', '/spgc/receiver')
        self.declare_parameter('text', 'Hello, ROS2!')

        self._topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self._text = self.get_parameter('text').get_parameter_value().string_value

        self._pub = self.create_publisher(String, self._topic_name, 10)
        self._timer = self.create_timer(1.0, self._on_timer)

        self.get_logger().info(f'Publisher started. Publishing to: {self._topic_name}')
        self.get_logger().info(f'Message text: "{self._text}"')

    def _on_timer(self):
        msg = String()
        msg.data = self._text
        self._pub.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
