#!/usr/bin/env python3
"""
Симулятор виртуальной квартиры для тестирования SLAM.
Включает несколько комнат, коридоры, мебель.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math
import random


class SlamWorldSimulator(Node):
    """
    Симулятор квартиры с лидаром и одометрией для тестирования SLAM.
    """
    
    def __init__(self):
        super().__init__('slam_world_simulator')
        
        # ═══════════════════════════════════════════════════
        # ПАРАМЕТРЫ ЛИДАРА D500
        # ═══════════════════════════════════════════════════
        self.angle_min = 0.0
        self.angle_max = 2 * math.pi
        self.angle_increment = math.radians(0.5)
        self.num_readings = 720
        self.range_min = 0.1
        self.range_max = 12.0
        self.noise_std = 0.02
        
        # ═══════════════════════════════════════════════════
        # ПОЗИЦИЯ РОБОТА
        # ═══════════════════════════════════════════════════
        self.robot_x = 2.0
        self.robot_y = 2.0
        self.robot_theta = 0.0
        self.robot_vx = 0.0
        self.robot_vtheta = 0.0
        
        # ═══════════════════════════════════════════════════
        # ГЕОМЕТРИЯ КВАРТИРЫ
        # ═══════════════════════════════════════════════════
        
        # Внешние стены (квартира 12×10 метров)
        self.walls = [
            # Внешний периметр
            (0.0, 0.0, 12.0, 0.0),    # Нижняя стена
            (12.0, 0.0, 12.0, 10.0),  # Правая стена
            (12.0, 10.0, 0.0, 10.0),  # Верхняя стена
            (0.0, 10.0, 0.0, 0.0),    # Левая стена
            
            # Комната 1 (гостиная) - левый нижний угол
            (0.0, 4.0, 4.5, 4.0),     # Стена с проёмом
            (5.5, 4.0, 6.0, 4.0),     # Продолжение стены
            
            # Комната 2 (спальня) - правый нижний угол  
            (6.0, 0.0, 6.0, 3.5),     # Вертикальная стена
            (6.0, 4.5, 6.0, 5.0),     # Стена над дверью
            
            # Комната 3 (кухня) - левый верхний угол
            (0.0, 6.0, 4.0, 6.0),     # Горизонтальная стена
            (4.0, 6.0, 4.0, 10.0),    # Вертикальная стена
            
            # Коридор и ванная - правая часть
            (8.0, 5.0, 8.0, 10.0),    # Стена коридора
            (8.0, 5.0, 12.0, 5.0),    # Горизонтальная стена
        ]
        
        # Мебель и препятствия (круги и прямоугольники)
        self.obstacles = [
            # Гостиная - диван и стол
            {'type': 'rect', 'x': 1.0, 'y': 1.0, 'w': 2.0, 'h': 0.8},  # Диван
            {'type': 'circle', 'x': 2.5, 'y': 2.5, 'r': 0.4},          # Журнальный стол
            
            # Спальня - кровать
            {'type': 'rect', 'x': 8.0, 'y': 1.0, 'w': 2.0, 'h': 1.5},  # Кровать
            
            # Кухня - стол и холодильник
            {'type': 'rect', 'x': 1.0, 'y': 7.5, 'w': 1.2, 'h': 1.2},  # Стол
            {'type': 'rect', 'x': 0.1, 'y': 9.0, 'w': 0.7, 'h': 0.7},  # Холодильник
            
            # Коридор - шкаф
            {'type': 'rect', 'x': 9.0, 'y': 6.0, 'w': 1.0, 'h': 2.0},  # Шкаф
        ]
        
        # ═══════════════════════════════════════════════════
        # ROS 2 ИНТЕРФЕЙСЫ
        # ═══════════════════════════════════════════════════
        
        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Subscriber для управления
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Таймеры
        self.scan_timer = self.create_timer(0.1, self.publish_scan)    # 10 Hz
        self.odom_timer = self.create_timer(0.02, self.update_odom)    # 50 Hz
        
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('SLAM World Simulator started')
        self.get_logger().info('Apartment size: 12×10 meters')
        self.get_logger().info(f'Robot start position: ({self.robot_x}, {self.robot_y})')
    
    def cmd_callback(self, msg):
        """Обработка команд управления."""
        self.robot_vx = msg.linear.x
        self.robot_vtheta = msg.angular.z
    
    def update_odom(self):
        """Обновление позиции робота и публикация одометрии."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Обновляем позицию
        if abs(self.robot_vtheta) < 0.001:
            # Движение по прямой
            self.robot_x += self.robot_vx * math.cos(self.robot_theta) * dt
            self.robot_y += self.robot_vx * math.sin(self.robot_theta) * dt
        else:
            # Движение по дуге
            radius = self.robot_vx / self.robot_vtheta
            self.robot_x += radius * (math.sin(self.robot_theta + self.robot_vtheta * dt) - 
                                      math.sin(self.robot_theta))
            self.robot_y += radius * (math.cos(self.robot_theta) - 
                                      math.cos(self.robot_theta + self.robot_vtheta * dt))
        
        self.robot_theta += self.robot_vtheta * dt
        
        # Нормализация угла
        while self.robot_theta > math.pi:
            self.robot_theta -= 2 * math.pi
        while self.robot_theta < -math.pi:
            self.robot_theta += 2 * math.pi
        
        # Проверка столкновений (простая)
        self.robot_x = max(0.3, min(11.7, self.robot_x))
        self.robot_y = max(0.3, min(9.7, self.robot_y))
        
        # Публикация одометрии
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.robot_x
        odom.pose.pose.position.y = self.robot_y
        odom.pose.pose.position.z = 0.0
        
        # Кватернион из угла
        odom.pose.pose.orientation.z = math.sin(self.robot_theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.robot_theta / 2)
        
        odom.twist.twist.linear.x = self.robot_vx
        odom.twist.twist.angular.z = self.robot_vtheta
        
        self.odom_pub.publish(odom)
        
        # TF: odom → base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.robot_theta / 2)
        t.transform.rotation.w = math.cos(self.robot_theta / 2)
        
        self.tf_broadcaster.sendTransform(t)
        
        # TF: base_link → laser_link (лидар в центре)
        t_laser = TransformStamped()
        t_laser.header.stamp = current_time.to_msg()
        t_laser.header.frame_id = 'base_link'
        t_laser.child_frame_id = 'laser_link'
        t_laser.transform.translation.z = 0.1
        t_laser.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t_laser)
    
    def publish_scan(self):
        """Генерация и публикация лидарного скана."""
        ranges = []
        
        for i in range(self.num_readings):
            angle = self.angle_min + i * self.angle_increment
            world_angle = self.robot_theta + angle
            
            min_dist = self.range_max
            
            # Проверяем пересечение со стенами
            for wall in self.walls:
                dist = self.ray_line_intersection(
                    self.robot_x, self.robot_y,
                    world_angle,
                    wall[0], wall[1], wall[2], wall[3]
                )
                if dist is not None and dist < min_dist:
                    min_dist = dist
            
            # Проверяем пересечение с препятствиями
            for obs in self.obstacles:
                if obs['type'] == 'circle':
                    dist = self.ray_circle_intersection(
                        self.robot_x, self.robot_y,
                        world_angle,
                        obs['x'], obs['y'], obs['r']
                    )
                elif obs['type'] == 'rect':
                    dist = self.ray_rect_intersection(
                        self.robot_x, self.robot_y,
                        world_angle,
                        obs['x'], obs['y'], obs['w'], obs['h']
                    )
                
                if dist is not None and dist < min_dist:
                    min_dist = dist
            
            # Добавляем шум
            if min_dist < self.range_max:
                min_dist += random.gauss(0, self.noise_std)
                min_dist = max(self.range_min, min(self.range_max, min_dist))
            
            # Случайные пропуски (2%)
            if random.random() < 0.02:
                min_dist = float('inf')
            
            ranges.append(min_dist)
        
        # Публикуем скан
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_link'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        
        self.scan_pub.publish(scan)
    
    def ray_line_intersection(self, rx, ry, angle, x1, y1, x2, y2):
        """Пересечение луча с отрезком."""
        dx = math.cos(angle)
        dy = math.sin(angle)
        
        x3, y3 = rx, ry
        x4, y4 = rx + dx * 100, ry + dy * 100
        
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denom) < 1e-10:
            return None
        
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom
        
        if 0 <= t <= 1 and u > 0:
            ix = x1 + t * (x2 - x1)
            iy = y1 + t * (y2 - y1)
            return math.sqrt((ix - rx)**2 + (iy - ry)**2)
        
        return None
    
    def ray_circle_intersection(self, rx, ry, angle, cx, cy, radius):
        """Пересечение луча с кругом."""
        dx = math.cos(angle)
        dy = math.sin(angle)
        
        fx = rx - cx
        fy = ry - cy
        
        a = dx * dx + dy * dy
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - radius * radius
        
        discriminant = b * b - 4 * a * c
        
        if discriminant < 0:
            return None
        
        discriminant = math.sqrt(discriminant)
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)
        
        if t1 > 0:
            return t1
        if t2 > 0:
            return t2
        
        return None
    
    def ray_rect_intersection(self, rx, ry, angle, ox, oy, w, h):
        """Пересечение луча с прямоугольником."""
        # Прямоугольник как 4 отрезка
        edges = [
            (ox, oy, ox + w, oy),         # Нижняя
            (ox + w, oy, ox + w, oy + h), # Правая
            (ox + w, oy + h, ox, oy + h), # Верхняя
            (ox, oy + h, ox, oy),         # Левая
        ]
        
        min_dist = None
        for edge in edges:
            dist = self.ray_line_intersection(rx, ry, angle, *edge)
            if dist is not None:
                if min_dist is None or dist < min_dist:
                    min_dist = dist
        
        return min_dist


def main(args=None):
    rclpy.init(args=args)
    node = SlamWorldSimulator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()