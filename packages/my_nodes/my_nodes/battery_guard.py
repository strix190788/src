"""Сторож батареи: следит за напряжением и предупреждает о разряде."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

LOW_VOLTAGE = 7.0  # В: порог тревоги, полная батарея 8.3 В, пустая 6.0 В


class BatteryGuard(Node):
    def __init__(self):
        super().__init__("battery_guard")
        self.voltage = None

        self.battery_sub = self.create_subscription(
            Float32, "/hardware/battery", self.on_battery, 1
        )
        self.alert_pub = self.create_publisher(String, "/battery_alert", 1)
        self.timer = self.create_timer(1.0, self.check)

        self.get_logger().info(f"battery_guard запущен, порог {LOW_VOLTAGE:.1f} В")

    def on_battery(self, msg):
        # колбэк только запоминает свежее значение
        self.voltage = msg.data

    def check(self):
        if self.voltage is None:
            return
        if self.voltage < LOW_VOLTAGE:
            alert = String()
            alert.data = f"Батарея разряжена: {self.voltage:.2f} В"
            self.alert_pub.publish(alert)
            self.get_logger().warn(alert.data)


def main():
    rclpy.init()
    node = BatteryGuard()
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
