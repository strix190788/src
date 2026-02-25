import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Импортируем тип сообщения


class RobotNewsStation(Node):
    """
    Узел-издатель, который публикует новости робота.
    """
    
    def __init__(self):
        super().__init__('robot_news_station')
        
        # Создаём Publisher
        self.publisher_ = self.create_publisher(
            String,           # тип сообщения
            'robot_news',     # имя топика
            10                # размер очереди
        )
        
        # Таймер для публикации каждые 0.5 секунды
        self.timer = self.create_timer(0.5, self.publish_news)
        
        # Счётчик сообщений
        self.counter = 0
        
        self.get_logger().info('Robot News Station запущена!')
    
    def publish_news(self):
        """
        Функция публикации новостей.
        """
        # Создаём сообщение
        msg = String()
        msg.data = f'Новость #{self.counter}: Робот работает отлично!'
        
        # Публикуем сообщение
        self.publisher_.publish(msg)
        
        # Выводим в лог
        self.get_logger().info(f'Публикую: "{msg.data}"')
        
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStation()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
