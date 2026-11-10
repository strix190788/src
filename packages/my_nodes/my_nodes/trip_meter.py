"""Одометр пробега: складывает оценку пробега из скоростей колёс.

Читает /hardware/status, вычисляет по измеренным скоростям колёс скорость
корпуса и накапливает оценку пробега. Итог отправляет в /trip_meter раз
в секунду.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

TELEMETRY_DT = 1.0 / 50.0  # с, столько проходит между сообщениями телеметрии


class TripMeter(Node):
    def __init__(self):
        super().__init__("trip_meter")

        self.declare_parameter("wheel_radius", 0.021)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)

        self.distance = 0.0  # накопленный путь, метры

        self.status_sub = self.create_subscription(
            Float32MultiArray, "/hardware/status", self.on_status, 10
        )
        self.trip_pub = self.create_publisher(Float32, "/trip_meter", 10)
        self.timer = self.create_timer(1.0, self.report)

        self.get_logger().info(
            f"trip_meter запущен, радиус колеса {self.wheel_radius} м"
        )

    def on_status(self, msg):
        # под номерами 1 и 4 лежат измеренные скорости колёс
        left_speed = msg.data[1]
        right_speed = msg.data[4]

        v = (left_speed + right_speed) * self.wheel_radius / 2.0
        self.distance += abs(v) * TELEMETRY_DT

    def report(self):
        out = Float32()
        out.data = float(self.distance)
        self.trip_pub.publish(out)
        self.get_logger().info(f"Пробег: {self.distance:.2f} м")


def main():
    rclpy.init()
    node = TripMeter()
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
