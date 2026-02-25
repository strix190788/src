import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math


class ImuReader(Node):
    """
    Узел для чтения и интерпретации данных IMU.
    """
    
    def __init__(self):
        super().__init__('imu_reader')
        
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        self.counter = 0
        self.get_logger().info('IMU Reader started')
    
    def quaternion_to_yaw(self, w, x, y, z):
        """Извлекает угол yaw из кватерниона."""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def imu_callback(self, msg):
        """Обработка данных IMU."""
        self.counter += 1
        
        # Выводим каждые 50 сообщений (~0.5 сек)
        if self.counter % 50 != 0:
            return
        
        # Извлекаем yaw из кватерниона
        yaw = self.quaternion_to_yaw(
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        )
        yaw_deg = math.degrees(yaw)
        
        # Угловая скорость
        gyro_z = msg.angular_velocity.z
        gyro_z_deg = math.degrees(gyro_z)
        
        # Ускорение
        accel_z = msg.linear_acceleration.z
        
        self.get_logger().info(
            f'IMU: yaw={yaw_deg:6.1f}°, '
            f'gyro_z={gyro_z_deg:6.2f}°/s, '
            f'accel_z={accel_z:5.2f} m/s²'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuReader()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()