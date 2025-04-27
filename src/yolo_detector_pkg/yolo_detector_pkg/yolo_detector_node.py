#!/usr/bin/env python3
"""
YOLO-v8 检测节点
订阅 : /camera/image_raw
发布 : /yolo/annotated (带框图)
      /ball_center       (第一个球 bbox 中心点像素坐标)
"""

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # ------------ 参数 ------------
        self.declare_parameter('weight_path', '')
        wpath = self.get_parameter('weight_path').get_parameter_value().string_value
        if not wpath:                       # 若未传参就用包内自带 yolov8n
            share = get_package_share_directory('yolo_detector_pkg')
            wpath = os.path.join(share, 'yolov8n.pt')

        self.get_logger().info(f'Loading YOLO weight: {wpath}')
        self.model   = YOLO(wpath)
        self.bridge  = CvBridge()

        # ------------ 通信 ------------
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.cb_img, 10)

        self.pub_img = self.create_publisher(Image,        '/yolo/annotated', 10)
        self.pub_pt  = self.create_publisher(PointStamped, '/ball_center',   10)

    # --------------------------------
    def cb_img(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        res   = self.model(frame, imgsz=640, conf=0.25)[0]

        # (1) 取第一个 bbox 中心并发布
        if len(res.boxes):
            x1, y1, x2, y2 = res.boxes.xyxy[0].cpu().numpy()
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            pt          = PointStamped()
            pt.header   = msg.header          # 保留时间戳/坐标系
            pt.point.x  = cx
            pt.point.y  = cy
            self.pub_pt.publish(pt)

        # (2) 发布带框图像
        annotated = res.plot()
        out = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        out.header = msg.header
        self.pub_img.publish(out)


def main():
    rclpy.init()
    rclpy.spin(YoloDetector())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
