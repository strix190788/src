"""Гироскопный компас: складывает угол поворота из показаний гироскопа.

Читает /hardware/imu, накапливает угол поворота вокруг вертикальной оси
и отправляет накопленное значение в /gyro_yaw. Угол копится как есть:
после двух оборотов влево в топике будет 4 пи, а не ноль.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

TELEMETRY_DT = 1.0 / 50.0  # с, столько проходит между сообщениями телеметрии


class GyroYaw(Node):
    def __init__(self):
        super().__init__("gyro_yaw")

        self.yaw = 0.0  # накопленный угол, радианы

        self.imu_sub = self.create_subscription(
            Imu, "/hardware/imu", self.on_imu, 10
        )
        self.yaw_pub = self.create_publisher(Float32, "/gyro_yaw", 10)

        self.get_logger().info("gyro_yaw запущен, начальный угол 0.00 рад")

    def on_imu(self, msg):
        # накопление угла: скорость поворота, умноженная на время
        self.yaw += msg.angular_velocity.z * TELEMETRY_DT

        out = Float32()
        out.data = float(self.yaw)
        self.yaw_pub.publish(out)


def main():
    rclpy.init()
    node = GyroYaw()
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
