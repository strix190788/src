#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math


class SlipDetector(Node):
    """
    Детектирует проскальзывание колес, сравнивая IMU и энкодеры.
    """
    
    def __init__(self):
        super().__init__('slip_detector')
        
        # Пороги
        self.angular_threshold = math.radians(10)  # 10°/с разница
        self.linear_threshold = 0.1  # 0.1 м/с разница
        
        # Последние данные
        self.imu_angular_z = 0.0
        self.odom_angular_z = 0.0
        self.odom_linear_x = 0.0
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom/wheel', self.odom_callback, 10)
        
        # Publisher
        self.slip_pub = self.create_publisher(
            Bool, '/slip_detected', 10)
        
        # Таймер для проверки
        self.timer = self.create_timer(0.1, self.check_slip)
        
        self.get_logger().info('Slip Detector started')
        self.get_logger().info(f'Angular threshold: {math.degrees(self.angular_threshold):.1f}°/s')
    
    def imu_callback(self, msg):
        self.imu_angular_z = msg.angular_velocity.z
    
    def odom_callback(self, msg):
        self.odom_angular_z = msg.twist.twist.angular.z
        self.odom_linear_x = msg.twist.twist.linear.x
    
    def check_slip(self):
        """
        Проверяем наличие проскальзывания.
        """
        # Сравниваем угловые скорости
        angular_diff = abs(self.imu_angular_z - self.odom_angular_z)
        
        # Детектируем проскальзывание
        is_slipping = angular_diff > self.angular_threshold
        
        # Публикуем
        msg = Bool()
        msg.data = is_slipping
        self.slip_pub.publish(msg)
        
        # Логируем при проскальзывании
        if is_slipping and abs(self.odom_linear_x) > 0.05:
            self.get_logger().warn(
                f'SLIP DETECTED! IMU={math.degrees(self.imu_angular_z):.1f}°/s, '
                f'Odom={math.degrees(self.odom_angular_z):.1f}°/s, '
                f'Diff={math.degrees(angular_diff):.1f}°/s'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SlipDetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()