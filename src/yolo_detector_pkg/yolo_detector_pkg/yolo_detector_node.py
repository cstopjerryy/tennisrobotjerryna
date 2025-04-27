#!/usr/bin/env python3
"""
yolo_detector_node.py
---------------------
订阅  : /camera/image_raw
发布  : /yolo/annotated   (sensor_msgs/Image)  → 带检测框图像
        /yolo/ball_error  (geometry_msgs/Vector3) → x ∈ [-1,1] 表示水平偏差
依赖  : ultralytics、cv_bridge、ROS 2 Humble
"""

import os
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # ---------- 读取模型权重 ----------
        self.declare_parameter('weight_path', '')
        weight_path = self.get_parameter('weight_path')\
                             .get_parameter_value().string_value
        if not weight_path:
            pkg_share = get_package_share_directory('yolo_detector_pkg')
            weight_path = os.path.join(pkg_share, 'yolov8n.pt')

        self.get_logger().info(f'Loading YOLOv8 weight: {weight_path}')
        self.model = YOLO(weight_path)
        self.bridge = CvBridge()

        # ---------- ROS 通信 ----------
        self.create_subscription(Image,
                                 '/camera/image_raw',
                                 self.image_cb,   10)
        self.pub_anno = self.create_publisher(Image,
                                              '/yolo/annotated', 10)
        self.pub_err  = self.create_publisher(Vector3,
                                              '/yolo/ball_error', 10)

        self.get_logger().info('✅ YOLO detector node ready')

    # ------------------------------------------------------------------
    def image_cb(self, msg: Image):
        """收到一帧图像就推理一次"""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO 推理
        results = self.model(frame, imgsz=640, conf=0.25)
        annotated = results[0].plot()

        # ① 发布可视化图像
        anno_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        anno_msg.header = msg.header
        self.pub_anno.publish(anno_msg)

        # ② 如果检测到球，发布水平偏差
        if len(results[0].boxes):
            # 取第 1 个 bounding-box
            box = results[0].boxes.xyxy[0]      # tensor([x1,y1,x2,y2])
            cx = float(box[0] + box[2]) / 2     # 像素中心
            dx = (cx - frame.shape[1] / 2) / (frame.shape[1] / 2)  # 归一化 [-1,1]

            err_msg = Vector3(x=dx, y=0.0, z=0.0)
            self.pub_err.publish(err_msg)


# ----------------------------------------------------------------------
def main():
    rclpy.init()
    rclpy.spin(YoloDetector())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
