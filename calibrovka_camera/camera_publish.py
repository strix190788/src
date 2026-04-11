#!/usr/bin/env python3
"""ROS 2 узел публикации CameraInfo"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import json

class CalibratedCameraNode(Node):
    def __init__(self):
        super().__init__('calibrated_camera')
        
        # Загружаем калибровку
        self.load_calibration('camera_calibration.json')
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.image_rect_pub = self.create_publisher(Image, '/camera/image_rect', 10)
        
        # cv_bridge
        self.bridge = CvBridge()
        
        # Открываем камеру
        self.cap = cv2.VideoCapture(0)
        
        # Таймер для захвата кадров
        self.timer = self.create_timer(0.033, self.capture_frame)  # ~30 FPS
        
        self.get_logger().info('Calibrated camera node started')
    
    def load_calibration(self, filename):
        """Загружаем параметры калибровки"""
        with open(filename, 'r') as f:
            data = json.load(f)
        
        self.K = np.array(data['camera_matrix'])
        self.D = np.array(data['distortion_coefficients'])
        self.image_width = data['image_width']
        self.image_height = data['image_height']
        
        # Создаём сообщение CameraInfo
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.width = self.image_width
        self.camera_info_msg.height = self.image_height
        self.camera_info_msg.distortion_model = 'plumb_bob'
        self.camera_info_msg.d = self.D.flatten().tolist()
        self.camera_info_msg.k = self.K.flatten().tolist()
        self.camera_info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Projection matrix
        P = np.zeros((3, 4))
        P[:3, :3] = self.K
        self.camera_info_msg.p = P.flatten().tolist()
        
        self.get_logger().info(f'Loaded calibration: {self.image_width}x{self.image_height}')
    
    def capture_frame(self):
        """Захватываем и публикуем кадр"""
        ret, frame = self.cap.read()
        if not ret:
            return
        
        # Время
        stamp = self.get_clock().now().to_msg()
        
        # Публикуем raw изображение
        raw_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = 'camera_link'
        self.image_pub.publish(raw_msg)
        
        # Публикуем CameraInfo
        self.camera_info_msg.header.stamp = stamp
        self.camera_info_msg.header.frame_id = 'camera_link'
        self.info_pub.publish(self.camera_info_msg)
        
        # Публикуем rectified изображение
        rectified = cv2.undistort(frame, self.K, self.D)
        rect_msg = self.bridge.cv2_to_imgmsg(rectified, 'bgr8')
        rect_msg.header.stamp = stamp
        rect_msg.header.frame_id = 'camera_link'
        self.image_rect_pub.publish(rect_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CalibratedCameraNode()
    rclpy.spin(node)
    node.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()