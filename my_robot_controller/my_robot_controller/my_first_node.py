#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
class MyFirstNode(Node):
    """Мой первый узел ROS 2"""
    def __init__(self):
        super().__init__('my_first_node')
        self.get_logger().info('Привет из ROS 2!')
        self.counter = 0
        self.create_timer(1.0, self.timer_callback)
    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Таймер сработал {self.counter}')
def main(args=None):
    rclpy.init(args=args)
    node = MyFirstNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
