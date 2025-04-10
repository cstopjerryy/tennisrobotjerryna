#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        # 创建发布器，发布到 /camera/image_raw 话题
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        # 读取测试图片（请确认该图片在你的 WSL 环境下存在）
        self.image = cv2.imread('/mnt/d/Python_YOLO5_forTennisDetection/data/images/val/DunTennisBack1.jpg')
        if self.image is None:
            self.get_logger().error("Test image not found at specified path!")
        # 每隔1秒发布一次
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if self.image is not None:
            msg = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info("Published test image.")

def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
