#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO  # 新的接口

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # 根据实际情况调整
            self.image_callback,
            10)
        self.bridge = CvBridge()
        # 加载你训练好的 YOLOv8 模型（确保路径使用 Linux 格式）
        self.model = YOLO('/mnt/d/Python_YOLO5_forTennisDetection/runs/detect/train12/weights/best.pt')
        self.get_logger().info("YOLOv8 detector node started")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image)  # 运行检测
        self.get_logger().info(f"Detection results: {results}")
        cv2.imshow("YOLOv8 Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
