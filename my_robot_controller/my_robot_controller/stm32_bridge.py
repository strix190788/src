#!/usr/bin/env python3
"""
ROS 2 мост для связи с прошивкой STM32.
Обеспечивает обмен данными между ROS 2 и микроконтроллером.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import serial
import struct
import threading
import math


class STM32Bridge(Node):
    """
    Мост между ROS 2 и микроконтроллером STM32.
    """
    
    def __init__(self):
        super().__init__('stm32_bridge')
        
        # Параметры робота
        self.wheel_radius = 0.033    # Радиус колеса (м)
        self.wheel_base = 0.14       # Колёсная база (м)
        
        # Параметры UART
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        # Открытие порта
        try:
            self.serial = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Connected to STM32 on {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.serial = None
        
        # Подписчики
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Публикаторы
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom/wheel', 10)
        self.wheels_pub = self.create_publisher(
            Float32MultiArray, '/wheel_velocities', 10)
        
        # Состояние одометрии
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_pos_left = 0.0
        self.prev_pos_right = 0.0
        self.first_reading = True
        
        # Запуск потока чтения телеметрии
        if self.serial:
            self.read_thread = threading.Thread(target=self.read_telemetry)
            self.read_thread.daemon = True
            self.read_thread.start()
        
        self.get_logger().info('STM32 Bridge initialized')
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Преобразует команды /cmd_vel в скорости колёс и отправляет на STM32.
        """
        if not self.serial:
            return
        
        # Дифференциальная кинематика
        v_linear = msg.linear.x
        v_angular = msg.angular.z
        
        # Скорости колёс (м/с → рад/с)
        v_left = (v_linear - v_angular * self.wheel_base / 2) / self.wheel_radius
        v_right = (v_linear + v_angular * self.wheel_base / 2) / self.wheel_radius
        
        # Формирование пакета
        packet = self.build_velocity_packet(v_left, v_right)
        
        try:
            self.serial.write(packet)
        except serial.SerialException as e:
            self.get_logger().error(f'UART write error: {e}')
    
    def build_velocity_packet(self, v_left: float, v_right: float) -> bytes:
        """
        Формирует пакет команды скорости для STM32.
        """
        packet = bytearray()
        packet.append(0x7E)                          # Start byte
        packet.append(0xA0)                          # Message type
        packet.extend(struct.pack('<f', v_left))     # Left velocity
        packet.extend(struct.pack('<f', v_right))    # Right velocity
        packet.append(0x00)                          # LED state
        packet.append(self.calculate_crc8(packet[2:]))  # CRC8
        
        return bytes(packet)
    
    def calculate_crc8(self, data: bytes) -> int:
        """
        Вычисляет CRC8 с полиномом 0x31.
        """
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x31) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc
    
    def read_telemetry(self):
        """
        Поток чтения телеметрии от STM32.
        """
        buffer = bytearray()
        
        while rclpy.ok():
            try:
                # Чтение доступных данных
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    buffer.extend(data)
                
                # Поиск пакета (начинается с 0x7E, длина 39 байт)
                while len(buffer) >= 39:
                    # Поиск стартового байта
                    start_idx = buffer.find(0x7E)
                    
                    if start_idx == -1:
                        buffer.clear()
                        break
                    
                    if start_idx > 0:
                        buffer = buffer[start_idx:]
                    
                    if len(buffer) < 39:
                        break
                    
                    # Извлечение пакета
                    packet = bytes(buffer[:39])
                    buffer = buffer[39:]
                    
                    # Парсинг и публикация
                    self.parse_telemetry(packet)
                    
            except serial.SerialException as e:
                self.get_logger().error(f'UART read error: {e}')
                break
    
    def parse_telemetry(self, packet: bytes):
        """
        Парсит телеметрию от STM32 и публикует в ROS 2 топики.
        """
        if len(packet) != 39 or packet[0] != 0x7E:
            return
        
        # Распаковка данных
        # IMU (сырые данные)
        accel_x = struct.unpack('<h', packet[1:3])[0]
        accel_y = struct.unpack('<h', packet[3:5])[0]
        accel_z = struct.unpack('<h', packet[5:7])[0]
        
        gyro_x = struct.unpack('<h', packet[7:9])[0]
        gyro_y = struct.unpack('<h', packet[9:11])[0]
        gyro_z = struct.unpack('<h', packet[11:13])[0]
        
        # Энкодеры
        pos_left = struct.unpack('<f', packet[19:23])[0]
        pos_right = struct.unpack('<f', packet[23:27])[0]
        vel_left = struct.unpack('<f', packet[27:31])[0]
        vel_right = struct.unpack('<f', packet[31:35])[0]
        
        # ToF лидары
        tof_left = struct.unpack('<h', packet[35:37])[0]
        tof_right = struct.unpack('<h', packet[37:39])[0]
        
        # Публикация IMU
        self.publish_imu(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
        
        # Публикация одометрии
        self.publish_odometry(pos_left, pos_right, vel_left, vel_right)
        
        # Публикация скоростей колёс
        wheels_msg = Float32MultiArray()
        wheels_msg.data = [vel_left, vel_right]
        self.wheels_pub.publish(wheels_msg)
    
    def publish_imu(self, ax, ay, az, gx, gy, gz):
        """
        Публикует данные IMU.
        """
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Преобразование сырых данных
        # Акселерометр: LSB → м/с² (при ±16G: 2048 LSB/g)
        accel_scale = 9.81 / 2048.0
        msg.linear_acceleration.x = ax * accel_scale
        msg.linear_acceleration.y = ay * accel_scale
        msg.linear_acceleration.z = az * accel_scale
        
        # Гироскоп: LSB → рад/с (при ±250°/s: 131 LSB/(°/s))
        gyro_scale = math.radians(1.0) / 131.0
        msg.angular_velocity.x = gx * gyro_scale
        msg.angular_velocity.y = gy * gyro_scale
        msg.angular_velocity.z = gz * gyro_scale
        
        self.imu_pub.publish(msg)
    
    def publish_odometry(self, pos_left, pos_right, vel_left, vel_right):
        """
        Вычисляет и публикует одометрию.
        """
        if self.first_reading:
            self.prev_pos_left = pos_left
            self.prev_pos_right = pos_right
            self.first_reading = False
            return
        
        # Изменение позиции колёс (рад → м)
        delta_left = (pos_left - self.prev_pos_left) * self.wheel_radius
        delta_right = (pos_right - self.prev_pos_right) * self.wheel_radius
        
        self.prev_pos_left = pos_left
        self.prev_pos_right = pos_right
        
        # Дифференциальная одометрия
        delta_s = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / self.wheel_base
        
        # Обновление позиции
        self.x += delta_s * math.cos(self.theta + delta_theta / 2)
        self.y += delta_s * math.sin(self.theta + delta_theta / 2)
        self.theta += delta_theta
        
        # Нормализация угла
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Линейная и угловая скорости
        v_linear = (vel_left + vel_right) * self.wheel_radius / 2.0
        v_angular = (vel_right - vel_left) * self.wheel_radius / self.wheel_base
        
        # Формирование сообщения
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        
        # Кватернион из угла
        msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        msg.pose.pose.orientation.w = math.cos(self.theta / 2)
        
        msg.twist.twist.linear.x = v_linear
        msg.twist.twist.angular.z = v_angular
        
        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = STM32Bridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial:
            node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()