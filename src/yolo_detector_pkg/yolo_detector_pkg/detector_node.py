#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        # ← 1) 读取参数，默认空串
        self.declare_parameter('weight_path', '')
        weight_path = self.get_parameter('weight_path').get_parameter_value().string_value

        # ← 2) 如果没有传参数，就用包内 share 的 yolov8n.pt
        if not weight_path:
            pkg_share = get_package_share_directory('yolo_detector_pkg')
            weight_path = os.path.join(pkg_share, 'yolov8n.pt')

        self.get_logger().info(f'Loading weight: {weight_path}')
        self.model = YOLO(weight_path)

    def image_cb(self, msg: Image):
        # msg → cv2
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 推理
        results = self.model(frame, imgsz=640, conf=0.25)
        annotated = results[0].plot()  # 带框的图

        # 发布
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)


def main():
    rclpy.init()
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
