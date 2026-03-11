#!/usr/bin/env python3
"""
Улучшенный симулятор лидара с реалистичной комнатой.
Включает стены, препятствия, двери и реалистичный шум.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import random


class LidarRoomSimulator(Node):
    """
    Симулятор 2D лидара LDROBOT D500 в виртуальной комнате.
    """
    
    def __init__(self):
        super().__init__('lidar_room_simulator')
        
        # Параметры лидара D500
        self.angle_min = 0.0
        self.angle_max = 2 * math.pi
        self.angle_increment = math.radians(0.5)  # 0.5°
        self.num_readings = 720
        
        self.range_min = 0.1   # 10 см
        self.range_max = 12.0  # 12 м
        
        self.noise_std = 0.02  # Стандартный шум ±2 см
        self.miss_probability = 0.02  # 2% пропущенных точек
        
        # Позиция робота
        self.robot_x = 3.0
        self.robot_y = 2.0
        self.robot_theta = 0.0
        
        # Описание комнаты (5×4 метра)
        self.room_width = 5.0
        self.room_height = 4.0
        
        # Стены комнаты (линии: x1, y1, x2, y2)
        self.walls = [
            (0.0, 0.0, 5.0, 0.0),  # Нижняя
            (5.0, 0.0, 5.0, 4.0),  # Правая
            (5.0, 4.0, 0.0, 4.0),  # Верхняя
            (0.0, 4.0, 0.0, 0.0),  # Левая
        ]
        
        # Препятствия (круги: x, y, radius)
        self.obstacles = [
            (1.5, 1.5, 0.3),
            (3.5, 2.5, 0.4),
            (2.0, 3.0, 0.25),
        ]
        
        # ROS 2 интерфейсы
        self.scan_publisher = self.create_publisher(
            LaserScan, '/scan', 10)
        
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom/wheel', self.odom_callback, 10)
        
        self.cmd_subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # Таймер - 10 Hz
        self.timer = self.create_timer(0.1, self.publish_scan)
        
        self.get_logger().info('Lidar Room Simulator started')
        self.get_logger().info(f'Room size: {self.room_width}x{self.room_height}m')
    
    def odom_callback(self, msg):
        """Обновляем позицию робота из одометрии."""
        self.robot_x = msg.pose.pose.position.x + 3.0
        self.robot_y = msg.pose.pose.position.y + 2.0
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_theta = math.atan2(siny_cosp, cosy_cosp)
    
    def cmd_callback(self, msg):
        """Простое обновление позиции из cmd_vel."""
        dt = 0.1
        self.robot_x += msg.linear.x * math.cos(self.robot_theta) * dt
        self.robot_y += msg.linear.x * math.sin(self.robot_theta) * dt
        self.robot_theta += msg.angular.z * dt
        
        # Ограничиваем в пределах комнаты
        self.robot_x = max(0.3, min(self.room_width - 0.3, self.robot_x))
        self.robot_y = max(0.3, min(self.room_height - 0.3, self.robot_y))
    
    def ray_line_intersection(self, ray_origin, ray_angle, line_start, line_end):
        """Вычисляет пересечение луча с отрезком."""
        rx, ry = ray_origin
        dx = math.cos(ray_angle)
        dy = math.sin(ray_angle)
        
        x1, y1 = line_start
        x2, y2 = line_end
        
        denom = dx * (y2 - y1) - dy * (x2 - x1)
        
        if abs(denom) < 1e-10:
            return None
        
        t = ((x1 - rx) * (y2 - y1) - (y1 - ry) * (x2 - x1)) / denom
        u = ((x1 - rx) * dy - (y1 - ry) * dx) / denom
        
        if t > 0 and 0 <= u <= 1:
            return t
        
        return None
    
    def ray_circle_intersection(self, ray_origin, ray_angle, circle_center, radius):
        """Вычисляет пересечение луча с кругом."""
        rx, ry = ray_origin
        dx = math.cos(ray_angle)
        dy = math.sin(ray_angle)
        
        cx, cy = circle_center
        
        fx = rx - cx
        fy = ry - cy
        
        a = dx * dx + dy * dy
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - radius * radius
        
        discriminant = b * b - 4 * a * c
        
        if discriminant < 0:
            return None
        
        sqrt_disc = math.sqrt(discriminant)
        t1 = (-b - sqrt_disc) / (2 * a)
        t2 = (-b + sqrt_disc) / (2 * a)
        
        if t1 > 0:
            return t1
        if t2 > 0:
            return t2
        
        return None
    
    def cast_ray(self, angle):
        """Бросает луч под заданным углом и возвращает расстояние."""
        global_angle = self.robot_theta + angle
        ray_origin = (self.robot_x, self.robot_y)
        
        min_distance = float('inf')
        
        # Проверяем пересечение со стенами
        for wall in self.walls:
            x1, y1, x2, y2 = wall
            dist = self.ray_line_intersection(
                ray_origin, global_angle,
                (x1, y1), (x2, y2)
            )
            if dist is not None and dist < min_distance:
                min_distance = dist
        
        # Проверяем пересечение с препятствиями
        for obs in self.obstacles:
            cx, cy, radius = obs
            dist = self.ray_circle_intersection(
                ray_origin, global_angle,
                (cx, cy), radius
            )
            if dist is not None and dist < min_distance:
                min_distance = dist
        
        return min_distance
    
    def publish_scan(self):
        """Генерирует и публикует лидарный скан."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_link'
        
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.1 / self.num_readings
        scan.scan_time = 0.1
        
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        
        ranges = []
        intensities = []
        
        for i in range(self.num_readings):
            angle = self.angle_min + i * self.angle_increment
            
            distance = self.cast_ray(angle)
            
            if distance < self.range_min:
                distance = float('inf')
            elif distance > self.range_max:
                distance = float('inf')
            else:
                noise = random.gauss(0, self.noise_std)
                distance += noise
                distance = max(self.range_min, distance)
                
                if random.random() < self.miss_probability:
                    distance = float('inf')
            
            ranges.append(distance)
            
            if distance != float('inf'):
                intensity = max(50, 255 - int(distance * 20))
            else:
                intensity = 0
            intensities.append(float(intensity))
        
        scan.ranges = ranges
        scan.intensities = intensities
        
        self.scan_publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = LidarRoomSimulator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()