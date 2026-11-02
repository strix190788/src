"""Нода-подписчик: печатает всё, что приходит в топик /chat."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__("listener")
        self.sub = self.create_subscription(String, "/chat", self.on_message, 10)

    def on_message(self, msg):
        self.get_logger().info(f"Получил: {msg.data}")


def main():
    rclpy.init()
    node = Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
