import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        # Subscribe to the raw image topic
        self.subscription = self.create_subscription(
            Image, '/camera_node/image_raw/compressed', self.listener_callback, 10)
        self.bridge = CvBridge()

    def listener_callback(self, data):
        # Convert ROS 2 Image message to OpenCV image
        # "bgr8" is the standard OpenCV color format
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        # Now you can use standard OpenCV functions
        height, width, channels = current_frame.shape
        print(f"Высота: {height}, Ширина: {width}, Каналы: {channels}")


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
