import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import math


class StaticTransformPublisher(Node):
    """
    Узел, публикующий статические трансформации между фреймами робота.
    """
    
    def __init__(self):
        super().__init__('static_transform_publisher')
        
        # Создаём StaticTransformBroadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        # Публикуем все статические трансформации
        self.publish_transforms()
        
        self.get_logger().info('Static transforms published')
    
    def publish_transforms(self):
        """
        Публикуем статические трансформации робота.
        """
        transforms = []
        
        # 1. Камера: 20 см впереди, 15 см вверх
        t_camera = TransformStamped()
        t_camera.header.stamp = self.get_clock().now().to_msg()
        t_camera.header.frame_id = 'base_link'
        t_camera.child_frame_id = 'camera_link'
        
        t_camera.transform.translation.x = 0.2
        t_camera.transform.translation.y = 0.0
        t_camera.transform.translation.z = 0.15
        
        # Без вращения (quaternion: x=0, y=0, z=0, w=1)
        t_camera.transform.rotation.x = 0.0
        t_camera.transform.rotation.y = 0.0
        t_camera.transform.rotation.z = 0.0
        t_camera.transform.rotation.w = 1.0
        
        transforms.append(t_camera)
        
        # 2. Лидар: в центре, 10 см вверх
        t_lidar = TransformStamped()
        t_lidar.header.stamp = self.get_clock().now().to_msg()
        t_lidar.header.frame_id = 'base_link'
        t_lidar.child_frame_id = 'lidar_link'
        
        t_lidar.transform.translation.x = 0.0
        t_lidar.transform.translation.y = 0.0
        t_lidar.transform.translation.z = 0.1
        
        t_lidar.transform.rotation.x = 0.0
        t_lidar.transform.rotation.y = 0.0
        t_lidar.transform.rotation.z = 0.0
        t_lidar.transform.rotation.w = 1.0
        
        transforms.append(t_lidar)
        
        # 3. IMU: в центре, 5 см вверх
        t_imu = TransformStamped()
        t_imu.header.stamp = self.get_clock().now().to_msg()
        t_imu.header.frame_id = 'base_link'
        t_imu.child_frame_id = 'imu_link'
        
        t_imu.transform.translation.x = 0.0
        t_imu.transform.translation.y = 0.0
        t_imu.transform.translation.z = 0.05
        
        t_imu.transform.rotation.x = 0.0
        t_imu.transform.rotation.y = 0.0
        t_imu.transform.rotation.z = 0.0
        t_imu.transform.rotation.w = 1.0
        
        transforms.append(t_imu)
        
        # Публикуем все трансформации одним вызовом
        self.tf_static_broadcaster.sendTransform(transforms)
        
        self.get_logger().info('Published: base_link → camera_link')
        self.get_logger().info('Published: base_link → lidar_link')
        self.get_logger().info('Published: base_link → imu_link')


def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()