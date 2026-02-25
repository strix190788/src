import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool


class MotorSimulator(Node):
    """
    Симулятор моторов робота.
    Получает команды движения и публикует состояние.
    """
    
    def __init__(self):
        super().__init__('motor_simulator')
        
        # Текущее состояние
        self.current_speed = 0.0
        self.current_angular = 0.0
        self.battery_level = 100.0
        self.is_moving = False
        
        # Subscriber для команд движения (Twist)
        self.cmd_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Subscriber для уровня батареи
        self.battery_subscriber = self.create_subscription(
            Float32,
            '/battery_level',
            self.battery_callback,
            10
        )
        
        # Publisher для состояния моторов
        self.state_publisher = self.create_publisher(
            Bool,
            '/motor_state',
            10
        )
        
        # Таймер для публикации состояния (каждые 0.5 секунды)
        self.timer = self.create_timer(0.5, self.publish_state)
        
        self.get_logger().info('Motor Simulator started')
    
    
    def battery_callback(self, msg):
        """
        Получаем уровень батареи.
        """
        self.battery_level = msg.data
    
    
    def cmd_vel_callback(self, msg):
        """
        Получаем команды движения.
        """
        # Проверяем батарею — нельзя ехать без заряда!
        if self.battery_level < 5.0:
            self.get_logger().error('Battery too low! Cannot move.')
            self.current_speed = 0.0
            self.current_angular = 0.0
            self.is_moving = False
            return
        
        # Сохраняем команды
        self.current_speed = msg.linear.x
        self.current_angular = msg.angular.z
        
        # Определяем, движется ли робот
        self.is_moving = abs(self.current_speed) > 0.01 or abs(self.current_angular) > 0.01
        
        if self.is_moving:
            self.get_logger().info(
                f'Motors running: speed={self.current_speed:.2f} m/s, '
                f'angular={self.current_angular:.2f} rad/s'
            )
        else:
            self.get_logger().info('Motors stopped')
    
    
    def publish_state(self):
        """
        Публикуем текущее состояние моторов.
        """
        msg = Bool()
        msg.data = self.is_moving
        self.state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorSimulator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

