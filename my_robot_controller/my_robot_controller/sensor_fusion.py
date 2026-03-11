#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math


class SensorFusion(Node):
    """
    Объединяет данные IMU и энкодеров для улучшенной одометрии.
    Использует комплементарный фильтр.
    """
    
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Параметры фильтра
        self.alpha = 0.98  # вес гироскопа (0.95-0.99)
        
        # Состояние
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # ориентация (fused)
        
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        # Компоненты для fusion
        self.theta_gyro = 0.0  # от IMU
        self.theta_encoders = 0.0  # от энкодеров
        
        # Время
        self.last_time = self.get_clock().now()
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom/wheel', self.odom_callback, 10)
        
        # Publisher
        self.fused_odom_pub = self.create_publisher(
            Odometry, '/odom/fused', 10)
        odom
        self.get_logger().info('Sensor Fusion started')
        self.get_logger().info(f'Complementary filter alpha={self.alpha}')
        self.get_logger().info('Publishing fused odometry to /odom/fused')
    
    def imu_callback(self, msg):
        """
        Обновляем ориентацию от гироскопа.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt > 0:
            # Интегрируем гироскоп
            gyro_z = msg.angular_velocity.z
            self.theta_gyro += gyro_z * dt
            
            # Нормализуем
            self.theta_gyro = math.atan2(
                math.sin(self.theta_gyro),
                math.cos(self.theta_gyro)
            )
        
        self.last_time = current_time
    
    def odom_callback(self, msg):
        """
        Получаем данные от энкодеров и объединяем с IMU.
        """
        # Извлекаем позицию и ориентацию от энкодеров
        x_enc = msg.pose.pose.position.x
        y_enc = msg.pose.pose.position.y
        
        # Ориентация от энкодеров
        q = msg.pose.pose.orientation
        self.theta_encoders = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y**2 + q.z**2)
        )
        
        # Скорости (берем от энкодеров)
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vth = msg.twist.twist.angular.z
        
        # ═══════════════════════════════════════════
        # SENSOR FUSION: Комплементарный фильтр
        # ═══════════════════════════════════════════
        
        # Объединяем ориентации
        self.theta = (
            self.alpha * self.theta_gyro +
            (1 - self.alpha) * self.theta_encoders
        )
        
        # Нормализуем
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Позиция: берем от энкодеров, но корректируем ориентацию
        # Это упрощенный подход. В полном EKF мы бы учитывали
        # ковариации и делали более сложную коррекцию.
        self.x = x_enc
        self.y = y_enc
        
        # Публикуем объединенную одометрию
        self.publish_fused_odom()
        
        # Логируем разницу
        angle_diff = math.degrees(self.theta_gyro - self.theta_encoders)
        if abs(angle_diff) > 1.0:  # больше 1°
            self.get_logger().info(
                f'Orientation: Gyro={math.degrees(self.theta_gyro):.1f}°, '
                f'Enc={math.degrees(self.theta_encoders):.1f}°, '
                f'Fused={math.degrees(self.theta):.1f}°, '
                f'Diff={angle_diff:.1f}°'
            )
    
    def publish_fused_odom(self):
        """
        Публикуем объединенную одометрию.
        """
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Позиция
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Ориентация (fused)
        q = self.yaw_to_quaternion(self.theta)
        odom.pose.pose.orientation = q
        
        # Ковариация (уменьшена благодаря fusion)
        odom.pose.covariance[0] = 0.005   # x (лучше)
        odom.pose.covariance[7] = 0.005   # y (лучше)
        odom.pose.covariance[35] = 0.01   # yaw (значительно лучше!)
        
        # Скорости
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        
        odom.twist.covariance[0] = 0.001
        odom.twist.covariance[35] = 0.005  # улучшенная угловая скорость
        
        self.fused_odom_pub.publish(odom)
    
    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusion()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()