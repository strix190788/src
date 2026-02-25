#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math


class WheelOdometry(Node):
    """
    Вычисление одометрии робота из энкодеров.
    """
    
    def __init__(self):
        super().__init__('wheel_odometry')
        
        # Параметры
        self.counts_per_rev = 16384
        self.wheel_diameter = 0.065
        self.wheel_base = 0.25
        self.wheel_circumference = math.pi * self.wheel_diameter
        
        # Предыдущие показания
        self.prev_counts = [0, 0, 0, 0]
        self.first_reading = True
        
        # Позиция робота
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # угол поворота
        
        # Subscriber на энкодеры
        self.encoder_sub = self.create_subscription(
            Int32MultiArray, '/encoders/raw', self.encoder_callback, 10)
        
        # Publisher одометрии
        self.odom_pub = self.create_publisher(
            Odometry, '/odom/wheel', 10)
        
        self.get_logger().info('Wheel Odometry started')
    
    def encoder_callback(self, msg):
        """Вычисляем одометрию из изменений энкодеров."""
        counts = list(msg.data)
        
        # Пропускаем первое чтение
        if self.first_reading:
            self.prev_counts = counts
            self.first_reading = False
            return
        
        # Изменение counts
        delta_left = ((counts[0] - self.prev_counts[0]) + 
                      (counts[1] - self.prev_counts[1])) / 2.0
        delta_right = ((counts[2] - self.prev_counts[2]) + 
                       (counts[3] - self.prev_counts[3])) / 2.0
        
        # Преобразуем в метры
        dist_left = (delta_left / self.counts_per_rev) * self.wheel_circumference
        dist_right = (delta_right / self.counts_per_rev) * self.wheel_circumference
        
        # Центральное расстояние и угол
        dist_center = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheel_base
        
        # Обновляем позицию
        self.x += dist_center * math.cos(self.theta + delta_theta / 2.0)
        self.y += dist_center * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        
        # Нормализуем угол
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Публикуем
        self.publish_odometry()
        
        # Сохраняем для следующей итерации
        self.prev_counts = counts
        
        # Логируем
        self.get_logger().info(
            f'Odom: x={self.x:.3f}m, y={self.y:.3f}m, '
            f'θ={math.degrees(self.theta):.1f}°'
        )
    
    def publish_odometry(self):
        """Публикует сообщение одометрии."""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Позиция
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Ориентация (кватернион)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()