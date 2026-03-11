#!/usr/bin/env python3
"""
Обработка лидарных данных: фильтрация, кластеризация, детекция препятствий.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np
from collections import deque


class LidarProcessor(Node):
    """
    Узел обработки лидарных данных.
    
    Функции:
    - Фильтрация шумов (медианный фильтр)
    - Преобразование в декартовы координаты
    - Кластеризация точек (группировка препятствий)
    - Детекция ближайших препятствий
    """
    
    def __init__(self):
        super().__init__('lidar_processor')
        
        # Параметры обработки
        self.median_filter_size = 5
        self.cluster_threshold = 0.3
        self.min_cluster_size = 5
        self.danger_distance = 0.5
        self.warning_distance = 1.0
        
        # Статистика
        self.scan_count = 0
        
        # ROS 2 интерфейсы
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.filtered_scan_pub = self.create_publisher(
            LaserScan, '/scan/filtered', 10)
        
        self.obstacles_pub = self.create_publisher(
            MarkerArray, '/obstacles', 10)
        
        self.get_logger().info('Lidar Processor started')
    
    def median_filter(self, ranges):
        """Применяет медианный фильтр к данным лидара."""
        filtered = []
        n = len(ranges)
        half_window = self.median_filter_size // 2
        
        for i in range(n):
            window = []
            for j in range(i - half_window, i + half_window + 1):
                idx = j % n
                r = ranges[idx]
                if not math.isinf(r) and not math.isnan(r):
                    window.append(r)
            
            if len(window) >= 3:
                window.sort()
                median = window[len(window) // 2]
                filtered.append(median)
            else:
                filtered.append(float('inf'))
        
        return filtered
    
    def polar_to_cartesian(self, ranges, angle_min, angle_increment):
        """Преобразует полярные координаты в декартовы."""
        points = []
        
        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            
            angle = angle_min + i * angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append((x, y, i))
        
        return points
    
    def euclidean_cluster(self, points):
        """Кластеризация точек методом Евклидова расстояния."""
        if not points:
            return []
        
        points_array = np.array([(p[0], p[1]) for p in points])
        visited = [False] * len(points)
        clusters = []
        
        for i in range(len(points)):
            if visited[i]:
                continue
            
            cluster = []
            queue = [i]
            
            while queue:
                idx = queue.pop(0)
                if visited[idx]:
                    continue
                
                visited[idx] = True
                cluster.append(idx)
                
                for j in range(len(points)):
                    if visited[j]:
                        continue
                    
                    dist = np.linalg.norm(
                        points_array[idx] - points_array[j]
                    )
                    
                    if dist < self.cluster_threshold:
                        queue.append(j)
            
            if len(cluster) >= self.min_cluster_size:
                cluster_points = [points[idx] for idx in cluster]
                clusters.append(cluster_points)
        
        return clusters
    
    def compute_cluster_properties(self, cluster_points):
        """Вычисляет свойства кластера."""
        if not cluster_points:
            return None
        
        points = np.array([(p[0], p[1]) for p in cluster_points])
        
        centroid_x = np.mean(points[:, 0])
        centroid_y = np.mean(points[:, 1])
        
        distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        min_idx = np.argmin(distances)
        closest_distance = distances[min_idx]
        
        min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
        min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])
        width = max_x - min_x
        height = max_y - min_y
        
        return {
            'centroid': (centroid_x, centroid_y),
            'closest_distance': closest_distance,
            'size': (width, height),
            'num_points': len(cluster_points)
        }
    
    def scan_callback(self, msg):
        """Главный callback обработки скана."""
        self.scan_count += 1
        
        # 1. Медианная фильтрация
        raw_ranges = list(msg.ranges)
        filtered_ranges = self.median_filter(raw_ranges)
        
        # Публикуем отфильтрованный скан
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.ranges = filtered_ranges
        filtered_msg.intensities = list(msg.intensities) if msg.intensities else []
        
        self.filtered_scan_pub.publish(filtered_msg)
        
        # 2. Преобразование в декартовы координаты
        points = self.polar_to_cartesian(
            filtered_ranges, 
            msg.angle_min, 
            msg.angle_increment
        )
        
        # 3. Кластеризация
        clusters = self.euclidean_cluster(points)
        
        # Вычисляем свойства кластеров
        cluster_props = []
        for cluster in clusters:
            props = self.compute_cluster_properties(cluster)
            if props:
                cluster_props.append(props)
        
        # 4. Публикация визуализации препятствий
        self.publish_obstacle_markers(cluster_props, msg.header)
        
        # Логирование
        if self.scan_count % 10 == 0:
            self.get_logger().info(
                f'Scan #{self.scan_count}: '
                f'{len(points)} points, '
                f'{len(clusters)} clusters'
            )
            
            if cluster_props:
                closest = min(cluster_props, key=lambda x: x['closest_distance'])
                dist = closest['closest_distance']
                
                if dist < self.danger_distance:
                    self.get_logger().warn(f'DANGER! Obstacle at {dist:.2f}m')
                elif dist < self.warning_distance:
                    self.get_logger().info(f'Warning: Obstacle at {dist:.2f}m')
    
    def publish_obstacle_markers(self, cluster_props, header):
        """Публикует маркеры препятствий для RViz."""
        marker_array = MarkerArray()
        
        for i, props in enumerate(cluster_props):
            marker = Marker()
            marker.header = header
            marker.header.frame_id = 'laser_link'
            marker.ns = 'obstacles'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = props['centroid'][0]
            marker.pose.position.y = props['centroid'][1]
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            
            size = max(props['size'][0], props['size'][1], 0.1)
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = 0.3
            
            dist = props['closest_distance']
            if dist < self.danger_distance:
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            elif dist < self.warning_distance:
                marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.6)
            else:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.4)
            
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 200000000
            
            marker_array.markers.append(marker)
            
            # Текст с расстоянием
            text_marker = Marker()
            text_marker.header = header
            text_marker.header.frame_id = 'laser_link'
            text_marker.ns = 'distances'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = props['centroid'][0]
            text_marker.pose.position.y = props['centroid'][1]
            text_marker.pose.position.z = 0.4
            
            text_marker.text = f'{dist:.2f}m'
            text_marker.scale.z = 0.15
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            
            text_marker.lifetime.sec = 0
            text_marker.lifetime.nanosec = 200000000
            
            marker_array.markers.append(text_marker)
        
        self.obstacles_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()