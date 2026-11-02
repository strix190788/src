"""Первая нода: раз в секунду пишет сообщение в журнал.

Ничего не отправляет и ничего не принимает. Задача — увидеть, что
происходит с нодой: запуск, работа по таймеру, завершение.
"""

import rclpy
from rclpy.node import Node


class HelloNode(Node):
    def __init__(self):
        super().__init__("hello")
        self.ticks = 0
        self.timer = self.create_timer(1.0, self.on_timer)
        self.get_logger().error("Нода hello запущена, жду таймер")

    def on_timer(self):
        self.ticks += 1
        self.get_logger().warning(f"Привет из ноды! Тик номер {self.ticks}")


def main():
    rclpy.init()
    node = HelloNode()
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
