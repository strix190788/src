import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String


class SystemMonitor(Node):
    """
    Мониторинг всей системы робота.
    Собирает данные со всех компонентов и публикует общий статус.
    """
    
    def __init__(self):
        super().__init__('system_monitor')
        
        # Состояние системы
        self.battery_level = 100.0
        self.is_moving = False
        
        # Subscribers для сбора данных
        self.battery_sub = self.create_subscription(
            Float32, '/battery_level', self.battery_callback, 10)
        
        self.motor_sub = self.create_subscription(
            Bool, '/motor_state', self.motor_callback, 10)
        
        # Publisher для общего статуса
        self.status_pub = self.create_publisher(
            String, '/system_status', 10)
        
        # Таймер для публикации статуса (каждые 3 секунды)
        self.timer = self.create_timer(3.0, self.publish_status)
        
        self.get_logger().info('System Monitor started')
    
    
    def battery_callback(self, msg):
        """Получаем уровень батареи."""
        self.battery_level = msg.data
    
    
    def motor_callback(self, msg):
        """Получаем состояние моторов."""
        self.is_moving = msg.data
    
    
    def publish_status(self):
        """
        Публикуем общий статус системы.
        """
        # Определяем статус на основе всех данных
        if self.battery_level < 5.0:
            status = "CRITICAL - System shutdown imminent"
        elif self.battery_level < 20.0:
            status = f"WARNING - Low battery ({self.battery_level:.1f}%)"
        elif self.is_moving:
            status = f"ACTIVE - Robot moving (Battery: {self.battery_level:.1f}%)"
        else:
            status = f"IDLE - Ready (Battery: {self.battery_level:.1f}%)"
        
        # Публикуем
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        
        self.get_logger().info(f'System Status: {status}')


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

