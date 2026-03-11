#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class CalibrationHelper(Node):
    """
    Помощник для калибровки параметров робота.
    """
    
    def __init__(self):
        super().__init__('calibration_helper')
        
        # Текущие параметры (из конфигурации)
        self.declare_parameter('wheel_diameter', 0.065)
        self.declare_parameter('wheel_base', 0.25)
        
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_base = self.get_parameter('wheel_base').value
        
        # Начальная и текущая одометрия
        self.start_odom = None
        self.current_odom = None
        
        # Режим калибровки
        self.calibration_mode = None  # 'diameter' или 'wheel_base'
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom/wheel', self.odom_callback, 10)
        
        self.cmd_sub = self.create_subscription(
            String, '/calibration/command', self.command_callback, 10)
        
        # Publisher для команд
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Calibration Helper started')
        self.get_logger().info(f'Current parameters:')
        self.get_logger().info(f'  wheel_diameter = {self.wheel_diameter} m')
        self.get_logger().info(f'  wheel_base = {self.wheel_base} m')
        self.get_logger().info('')
        self.get_logger().info('Commands (publish to /calibration/command):')
        self.get_logger().info('  "start_distance" - начать калибровку диаметра')
        self.get_logger().info('  "end_distance <real_distance_m>" - закончить')
        self.get_logger().info('  "start_rotation" - начать калибровку базы')
        self.get_logger().info('  "end_rotation <real_angle_deg>" - закончить')
    
    def odom_callback(self, msg):
        self.current_odom = msg
    
    def command_callback(self, msg):
        parts = msg.data.split()
        command = parts[0]
        
        if command == 'start_distance':
            self.start_distance_calibration()
        
        elif command == 'end_distance' and len(parts) == 2:
            real_distance = float(parts[1])
            self.end_distance_calibration(real_distance)
        
        elif command == 'start_rotation':
            self.start_rotation_calibration()
        
        elif command == 'end_rotation' and len(parts) == 2:
            real_angle = float(parts[1])
            self.end_rotation_calibration(real_angle)
        
        else:
            self.get_logger().error(f'Unknown command: {msg.data}')
    
    def start_distance_calibration(self):
        """
        Начинаем калибровку диаметра колес.
        """
        self.calibration_mode = 'diameter'
        self.start_odom = self.current_odom
        
        self.get_logger().info('═' * 70)
        self.get_logger().info('КАЛИБРОВКА ДИАМЕТРА КОЛЕС')
        self.get_logger().info('═' * 70)
        self.get_logger().info('1. Поставьте метку на текущей позиции робота')
        self.get_logger().info('2. Дайте команду двигаться вперед:')
        self.get_logger().info('   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"')
        self.get_logger().info('3. Через несколько секунд остановите робота')
        self.get_logger().info('4. Измерьте РЕАЛЬНОЕ расстояние рулеткой (в метрах)')
        self.get_logger().info('5. Введите команду:')
        self.get_logger().info('   ros2 topic pub --once /calibration/command std_msgs/msg/String "{data: \'end_distance <real_distance>\'}"')
        self.get_logger().info('')
    
    def end_distance_calibration(self, real_distance):
        """
        Заканчиваем калибровку диаметра.
        """
        if self.calibration_mode != 'diameter' or self.start_odom is None:
            self.get_logger().error('Сначала запустите start_distance!')
            return
        
        # Вычисляем пройденное расстояние по одометрии
        dx = self.current_odom.pose.pose.position.x - self.start_odom.pose.pose.position.x
        dy = self.current_odom.pose.pose.position.y - self.start_odom.pose.pose.position.y
        odom_distance = math.sqrt(dx**2 + dy**2)
        
        # Вычисляем коррекцию
        correction_factor = real_distance / odom_distance
        new_diameter = self.wheel_diameter * correction_factor
        
        self.get_logger().info('─' * 70)
        self.get_logger().info('РЕЗУЛЬТАТЫ КАЛИБРОВКИ ДИАМЕТРА:')
        self.get_logger().info(f'  Реальное расстояние:      {real_distance:.4f} м')
        self.get_logger().info(f'  Одометрия показала:       {odom_distance:.4f} м')
        self.get_logger().info(f'  Ошибка:                   {(odom_distance - real_distance)*1000:.1f} мм')
        self.get_logger().info(f'  Коэффициент коррекции:    {correction_factor:.6f}')
        self.get_logger().info('')
        self.get_logger().info(f'  Старый диаметр колес:     {self.wheel_diameter*1000:.2f} мм')
        self.get_logger().info(f'  НОВЫЙ диаметр колес:      {new_diameter*1000:.2f} мм')
        self.get_logger().info('')
        self.get_logger().info('Обновите конфигурацию:')
        self.get_logger().info(f'  wheel_diameter: {new_diameter:.6f}')
        self.get_logger().info('═' * 70)
        
        self.calibration_mode = None
        self.start_odom = None
    
    def start_rotation_calibration(self):
        """
        Начинаем калибровку колесной базы.
        """
        self.calibration_mode = 'wheel_base'
        self.start_odom = self.current_odom
        
        self.get_logger().info('═' * 70)
        self.get_logger().info('КАЛИБРОВКА КОЛЕСНОЙ БАЗЫ')
        self.get_logger().info('═' * 70)
        self.get_logger().info('1. Отметьте начальное направление робота')
        self.get_logger().info('2. Дайте команду повернуться на 360°:')
        self.get_logger().info('   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}"')
        self.get_logger().info('3. Подождите один полный оборот')
        self.get_logger().info('4. Остановите: Ctrl+C')
        self.get_logger().info('5. Измерьте РЕАЛЬНЫЙ угол поворота (градусы)')
        self.get_logger().info('   Например: 365° (перекрутил) или 355° (недокрутил)')
        self.get_logger().info('6. Введите команду:')
        self.get_logger().info('   ros2 topic pub --once /calibration/command std_msgs/msg/String "{data: \'end_rotation <real_angle>\'}"')
        self.get_logger().info('')
    
    def end_rotation_calibration(self, real_angle_deg):
        """
        Заканчиваем калибровку колесной базы.
        """
        if self.calibration_mode != 'wheel_base' or self.start_odom is None:
            self.get_logger().error('Сначала запустите start_rotation!')
            return
        
        # Извлекаем углы из quaternion
        def quat_to_yaw(q):
            return math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y**2 + q.z**2)
            )
        
        start_yaw = quat_to_yaw(self.start_odom.pose.pose.orientation)
        current_yaw = quat_to_yaw(self.current_odom.pose.pose.orientation)
        
        # Вычисляем разницу углов
        delta_yaw = current_yaw - start_yaw
        # Нормализуем
        while delta_yaw > math.pi:
            delta_yaw -= 2 * math.pi
        while delta_yaw < -math.pi:
            delta_yaw += 2 * math.pi
        
        odom_angle_deg = math.degrees(delta_yaw)
        
        # Коррекция
        correction_factor = real_angle_deg / odom_angle_deg
        new_wheel_base = self.wheel_base * correction_factor
        
        self.get_logger().info('─' * 70)
        self.get_logger().info('РЕЗУЛЬТАТЫ КАЛИБРОВКИ КОЛЕСНОЙ БАЗЫ:')
        self.get_logger().info(f'  Реальный угол:            {real_angle_deg:.2f}°')
        self.get_logger().info(f'  Одометрия показала:       {odom_angle_deg:.2f}°')
        self.get_logger().info(f'  Ошибка:                   {odom_angle_deg - real_angle_deg:.2f}°')
        self.get_logger().info(f'  Коэффициент коррекции:    {correction_factor:.6f}')
        self.get_logger().info('')
        self.get_logger().info(f'  Старая колесная база:     {self.wheel_base*1000:.2f} мм')
        self.get_logger().info(f'  НОВАЯ колесная база:      {new_wheel_base*1000:.2f} мм')
        self.get_logger().info('')
        self.get_logger().info('Обновите конфигурацию:')
        self.get_logger().info(f'  wheel_base: {new_wheel_base:.6f}')
        self.get_logger().info('═' * 70)
        
        self.calibration_mode = None
        self.start_odom = None


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationHelper()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()