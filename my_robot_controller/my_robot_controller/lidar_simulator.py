import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import random


class LidarSimulator(Node):
    """
    Симулятор 2D лидара LDROBOT D500.
    """
    
    def __init__(self):
        super().__init__('lidar_simulator')
        
        # Параметры D500
        self.num_readings = 720  # точек на оборот
        self.angle_increment = math.radians(0.5)  # шаг 0.5°
        self.range_min = 0.1   # метров
        self.range_max = 12.0  # метров
        
        # Publisher
        self.scan_publisher = self.create_publisher(
            LaserScan, '/scan', 10)
        
        # Таймер - 10 Hz (как реальный лидар)
        self.timer = self.create_timer(0.1, self.publish_scan)
        
        self.get_logger().info('LDROBOT D500 Lidar Simulator started')
        self.get_logger().info('Publishing at 10 Hz on /scan')
    
    def publish_scan(self):
        """Генерируем лидарный скан."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_link'
        
        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi
        scan.angle_increment = self.angle_increment
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        
        # Генерируем данные — симуляция прямоугольной комнаты
        ranges = []
        
        for i in range(self.num_readings):
            angle = i * self.angle_increment
            
            # Базовое расстояние до стен
            base_distance = 4.0 + math.sin(angle * 4) * 0.5
            
            # "Двери" — пропуски в стенах
            if 0.3 < angle < 0.6:  # дверь спереди
                distance = float('inf')
            elif 2.5 < angle < 2.8:  # дверь сзади
                distance = float('inf')
            else:
                # Добавляем шум
                noise = random.gauss(0, 0.02)
                distance = base_distance + noise
                distance = max(self.range_min, min(self.range_max, distance))
            
            ranges.append(distance)
        
        scan.ranges = ranges
        self.scan_publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSimulator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()