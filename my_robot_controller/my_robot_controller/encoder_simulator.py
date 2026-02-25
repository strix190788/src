#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
import math


class EncoderSimulator(Node):
    """
    Симулятор магнитных энкодеров MT6701.
    4 колеса: левое переднее, левое заднее, правое переднее, правое заднее.
    """
    
    def __init__(self):
        super().__init__('encoder_simulator')
        
        # Параметры робота
        self.counts_per_rev = 16384  # разрешение энкодера
        self.wheel_diameter = 0.065  # метров
        self.wheel_base = 0.25  # расстояние между колёсами
        self.wheel_circumference = math.pi * self.wheel_diameter
        
        # Счётчики энкодеров [LF, LR, RF, RR]
        self.encoder_counts = [0, 0, 0, 0]
        
        # Текущие скорости
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Subscriber на команды
        self.cmd_subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Publisher для энкодеров
        self.encoder_publisher = self.create_publisher(
            Int32MultiArray, '/encoders/raw', 10)
        
        # Таймер - 100 Hz
        self.timer = self.create_timer(0.01, self.update_encoders)
        
        self.get_logger().info('Encoder Simulator (MT6701 x4) started')
        self.get_logger().info(f'Resolution: {self.counts_per_rev} counts/rev')
    
    def cmd_vel_callback(self, msg):
        """Получаем команды скорости."""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
    
    def update_encoders(self):
        """Обновляем показания энкодеров."""
        dt = 0.01
        
        # Дифференциальная кинематика
        # Левые колёса едут медленнее при повороте направо
        v_left = self.linear_vel - (self.angular_vel * self.wheel_base / 2.0)
        v_right = self.linear_vel + (self.angular_vel * self.wheel_base / 2.0)
        
        # Пройденное расстояние за dt
        dist_left = v_left * dt
        dist_right = v_right * dt
        
        # Преобразуем в counts
        delta_left = round((dist_left / self.wheel_circumference) * self.counts_per_rev)
        delta_right = round((dist_right / self.wheel_circumference) * self.counts_per_rev)
        
        # Обновляем счётчики
        self.encoder_counts[0] += delta_left   # Left Front
        self.encoder_counts[1] += delta_left   # Left Rear
        self.encoder_counts[2] += delta_right  # Right Front
        self.encoder_counts[3] += delta_right  # Right Rear
        
        # Публикуем
        msg = Int32MultiArray()
        msg.data = self.encoder_counts.copy()
        self.encoder_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderSimulator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()