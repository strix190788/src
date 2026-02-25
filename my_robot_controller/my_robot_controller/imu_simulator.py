import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import random


class ImuSimulator(Node):
    """
    Симулятор IMU MPU9250.
    Имитирует данные, которые STM32 отправляет через UART.
    """
    
    def __init__(self):
        super().__init__('imu_simulator')
        
        # Текущая угловая скорость робота
        self.angular_velocity = 0.0
        
        # Ориентация робота (угол поворота)
        self.yaw = 0.0  # радианы
        
        # Смещение гироскопа (bias) - есть у всех реальных датчиков
        self.gyro_bias = random.gauss(0, 0.01)
        
        # Publisher для данных IMU
        self.imu_publisher = self.create_publisher(
            Imu,
            '/imu/data',
            10
        )
        
        # Subscriber для команд движения
        self.cmd_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Таймер - 100 Hz (как реальный IMU)
        self.timer = self.create_timer(0.01, self.publish_imu_data)
        
        self.get_logger().info('IMU Simulator (MPU9250) started')
        self.get_logger().info('Publishing at 100 Hz on /imu/data')
    
    def cmd_vel_callback(self, msg):
        """Получаем команды движения."""
        self.angular_velocity = msg.angular.z
    
    def publish_imu_data(self):
        """Генерируем и публикуем данные IMU."""
        dt = 0.01  # период таймера
        
        # === ГИРОСКОП ===
        # Реальная угловая скорость + bias + шум
        gyro_noise = random.gauss(0, 0.005)
        gyro_z = self.angular_velocity + self.gyro_bias + gyro_noise
        
        # Обновляем ориентацию (интегрирование)
        self.yaw += gyro_z * dt
        
        # Нормализуем угол в диапазон [-π, π]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        
        # === АКСЕЛЕРОМЕТР ===
        accel_noise = random.gauss(0, 0.1)
        accel_z = 9.81 + accel_noise  # гравитация
        
        # === ОРИЕНТАЦИЯ (кватернион) ===
        # Преобразуем угол yaw в кватернион
        q_w = math.cos(self.yaw / 2.0)
        q_z = math.sin(self.yaw / 2.0)
        
        # === ПУБЛИКУЕМ ===
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Ориентация
        msg.orientation.w = q_w
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = q_z
        
        # Угловая скорость (гироскоп)
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = gyro_z
        
        # Линейное ускорение (акселерометр)
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = accel_z
        
        self.imu_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuSimulator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()