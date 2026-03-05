import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import numpy as np

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class RobotTFBroadcaster(Node):
    """
    Узел, публикующий динамические трансформации odom --> base_link.
    """
    
    def __init__(self):
        super().__init__('robot_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)
        self.time_start = self.get_clock().now()
                
        self.get_logger().info('Dinamic transforms published')
        
    def broadcast_timer_callback(self):
        # Вычислить позицию на круге
        t = (self.get_clock().now() - self.time_start).nanoseconds / 1e9
        angle = 2 * math.pi * t / 30.0  # один оборот за 30 сек
        
        x = 2.0 * math.cos(angle)
        y = 2.0 * math.sin(angle)
        
        transforms = []
        t_base = TransformStamped()
        t_base.header.stamp = self.get_clock().now().to_msg()
        t_base.header.frame_id = 'odom'
        t_base.child_frame_id = 'base_link'
        
        t_base.transform.translation.x = x
        t_base.transform.translation.y = y
        
        q = quaternion_from_euler(0, 0, angle + math.pi*0.5)
        t_base.transform.rotation.x = q[0]
        t_base.transform.rotation.y = q[1]
        t_base.transform.rotation.z = q[2]
        t_base.transform.rotation.w = q[3]

        transforms.append(t_base)
        self.tf_broadcaster.sendTransform(transforms)
        self.get_logger().info('Published: odom → base_link')


def main(args=None):
    rclpy.init(args=args)
    node = RobotTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()