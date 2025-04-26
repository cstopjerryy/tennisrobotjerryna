#!/usr/bin/env python3
"""
yolo_detector.py
================
订阅摄像头话题 `/camera/image_raw`，
使用 YOLOv8 进行检测，
把加了检测框的图像发布到 `/yolo/annotated`，
方便在 rqt_image_view 中查看。

‣ 依赖：
    - ultralytics>=8.1
    - cv_bridge
    - ROS 2 Humble

作者：Jerry :-)
"""

import os
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloDetector(Node):
    """ROS2 节点：YOLOv8 实时目标检测"""

    def __init__(self):
        super().__init__('yolo_detector')

        # ---------- 读取参数 ----------
        # 可在 launch 或 CLI 用 `weight_path:=/xx/yy.pt` 覆盖
        self.declare_parameter('weight_path', '')
        weight_path = self.get_parameter('weight_path').get_parameter_value().string_value

        # 如果没有显式设置，使用包内自带 yolov8n.pt（6 MB）
        if not weight_path:
            pkg_share = get_package_share_directory('yolo_detector_pkg')
            weight_path = os.path.join(pkg_share, 'yolov8n.pt')

        self.get_logger().info(f'Loading weight: {weight_path}')
        self.model = YOLO(weight_path)           # 加载模型
        self.bridge = CvBridge()                 # ros ↔ cv2

        # ---------- ROS 通信 ----------
        self.subscription = self.create_subscription(
            Image,                               # 消息类型
            '/camera/image_raw',                 # 订阅的话题
            self.image_cb,                       # 回调函数
            10                                   # QoS depth
        )

        self.pub = self.create_publisher(
            Image, '/yolo/annotated', 10         # 发布带框图像
        )

        self.get_logger().info('YOLO detector node ready')

    # ------------------------------------------------------------------
    # 图像回调：收到一帧就执行
    # ------------------------------------------------------------------
    def image_cb(self, msg: Image):
        # 1) ROS 图像 → OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 2) YOLOv8 推理
        results = self.model(frame, imgsz=640, conf=0.25)
        annotated = results[0].plot()            # 带框的彩色图

        # （可选）打印检测到的类别
        # names = [self.model.names[int(cls)] for cls in results[0].boxes.cls]
        # self.get_logger().info(f'Detected: {names}')

        # 3) OpenCV → ROS 图像并发布
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out_msg.header = msg.header             # 保留时间戳
        self.pub.publish(out_msg)


# ----------------------------------------------------------------------
# 主函数：启动节点
# ----------------------------------------------------------------------
def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
