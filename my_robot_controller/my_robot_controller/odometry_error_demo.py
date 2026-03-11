#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math


class OdometryErrorDemo(Node):
    """
    Демонстрация накопления ошибок одометрии.
    Сравнивает "идеальную" позицию с одометрией с ошибками.
    """
    
    def __init__(self):
        super().__init__('odometry_error_demo')
        
        # Параметры "идеального" робота
        self.ideal_wheel_diameter = 0.065  # м
        self.ideal_wheel_base = 0.25  # м
        
        # Параметры "реального" робота (с ошибками)
        self.real_wheel_diameter = 0.0655  # +0.5 мм ошибка!
        self.real_wheel_base = 0.252  # +2 мм ошибка!
        
        # Идеальная позиция
        self.ideal_x = 0.0
        self.ideal_y = 0.0
        self.ideal_theta = 0.0
        
        # "Реальная" позиция (с ошибками)
        self.real_x = 0.0
        self.real_y = 0.0
        self.real_theta = 0.0
        
        # Текущие команды
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Subscriber
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # Publishers
        self.ideal_odom_pub = self.create_publisher(
            Odometry, '/odom/ideal', 10)
        
        self.real_odom_pub = self.create_publisher(
            Odometry, '/odom/with_errors', 10)
        
        # Таймер
        self.timer = self.create_timer(0.01, self.update_odometry)
        
        self.get_logger().info('Odometry Error Demo started')
        self.get_logger().info('Publishing:')
        self.get_logger().info('  /odom/ideal - perfect odometry')
        self.get_logger().info('  /odom/with_errors - with systematic errors')
    
    def cmd_callback(self, msg):
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
    
    def update_odometry(self):
        dt = 0.01
        
        # Идеальная одометрия
        self.update_pose(
            dt,
            self.ideal_wheel_diameter,
            self.ideal_wheel_base,
            is_ideal=True
        )
        
        # "Реальная" одометрия (с ошибками)
        self.update_pose(
            dt,
            self.real_wheel_diameter,
            self.real_wheel_base,
            is_ideal=False
        )
        
        # Публикуем
        self.publish_odom(self.ideal_x, self.ideal_y, self.ideal_theta, True)
        self.publish_odom(self.real_x, self.real_y, self.real_theta, False)
        
        # Вычисляем ошибку
        error_x = self.real_x - self.ideal_x
        error_y = self.real_y - self.ideal_y
        error_theta = self.real_theta - self.ideal_theta
        error_distance = math.sqrt(error_x**2 + error_y**2)
        
        # Логируем только при движении
        if abs(self.linear_vel) > 0.01 or abs(self.angular_vel) > 0.01:
            self.get_logger().info(
                f'Odometry Error: Δpos={error_distance*1000:.1f}mm, '
                f'Δangle={math.degrees(error_theta):.2f}°'
            )
    
    def update_pose(self, dt, wheel_d, wheel_b, is_ideal):
        # Вычисляем приращение позиции
        dist = self.linear_vel * dt
        dtheta = self.angular_vel * dt
        
        if is_ideal:
            # Обновляем идеальную позицию
            self.ideal_x += dist * math.cos(self.ideal_theta + dtheta/2)
            self.ideal_y += dist * math.sin(self.ideal_theta + dtheta/2)
            self.ideal_theta += dtheta
        else:
            # Обновляем "реальную" позицию с ошибками параметров
            # Масштабируем расстояние и угол из-за неточных параметров
            scale_d = self.real_wheel_diameter / self.ideal_wheel_diameter
            scale_b = self.real_wheel_base / self.ideal_wheel_base
            
            dist_scaled = dist * scale_d
            dtheta_scaled = dtheta * scale_b
            
            self.real_x += dist_scaled * math.cos(self.real_theta + dtheta_scaled/2)
            self.real_y += dist_scaled * math.sin(self.real_theta + dtheta_scaled/2)
            self.real_theta += dtheta_scaled
    
    def publish_odom(self, x, y, theta, is_ideal):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        
        # Quaternion
        odom.pose.pose.orientation.w = math.cos(theta / 2)
        odom.pose.pose.orientation.z = math.sin(theta / 2)
        
        if is_ideal:
            self.ideal_odom_pub.publish(odom)
        else:
            self.real_odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryErrorDemo()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()