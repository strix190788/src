"""Нода-издатель: два раза в секунду отправляет строку в топик /chat."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__("talker")
        self.pub = self.create_publisher(String, "/chat", 10)
        self.count = 0
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        msg = String()
        msg.data = f"Привет из talker, сообщение № {self.count}"
        self.pub.publish(msg)
        self.get_logger().info(f"Отправил: {msg.data}")
        self.count += 1


def main():
    rclpy.init()
    node = Talker()
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
