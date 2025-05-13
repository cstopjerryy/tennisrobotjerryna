#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
yolo_detector_node.py
=====================
订阅 `/camera/image_raw`，用 YOLOv8 检测“tennis_ball”，

‣ 发布 2 个话题
    1) /yolo/annotated   sensor_msgs/Image     - 带框彩色图
    2) /yolo/balls       geometry_msgs/Point   - 最大的网球中心像素坐标
       （x 像素，y 像素，z = 置信度; 若当前帧无网球，则不发布）

‣ 参数
    weight_path      string  权重文件路径（必填；若空则失败退出）
    min_conf         double  置信度阈值，默认 0.25
"""
import os, sys, time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # ---------------- 参数 ----------------
        self.declare_parameter('weight_path', '/home/jerryyang/ros2_ws/src/yolo_detector_pkg/yolov8n.pt')
        self.declare_parameter('min_conf', 0.25)
        wpath = self.get_parameter('weight_path').get_parameter_value().string_value
        self.min_conf = self.get_parameter('min_conf').get_parameter_value().double_value

        if not wpath:
            self.get_logger().fatal('⚠️ 必须通过 weight_path:=... 指定自己训练好的 best.pt')
            rclpy.shutdown(); sys.exit(1)

        self.get_logger().info(f'Loading YOLO weight: {wpath}')
        self.model   = YOLO(wpath)
        self.bridge  = CvBridge()

        # ---------------- 通信 ----------------
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.cb_image, 10)
        self.pub_img = self.create_publisher(Image, '/yolo/annotated', 10)
        self.pub_pt  = self.create_publisher(Point, '/yolo/balls', 10)

        self.get_logger().info(f'✅ yolo_detector ready, min_conf={self.min_conf}')

    # ------------------------------------------------------------------
    def cb_image(self, msg: Image):
        # 1) 消息转 cv2
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # 2) YOLO 推理
        res = self.model(frame, imgsz=1280, conf=self.min_conf)[0]

        # 3) 发布最大球中心
        tennis_ids = [i for i, c in enumerate(res.boxes.cls) 
                      if self.model.names[int(c)] in [('tennis_ball', 'sports ball')]]
        if tennis_ids:
            # 取面积最大的球
            areas = res.boxes.xywh[tennis_ids][:,2] * res.boxes.xywh[tennis_ids][:,3]
            idx   = tennis_ids[int(areas.argmax())]
            x1,y1,x2,y2 = res.boxes.xyxy[idx].cpu().numpy()
            cx, cy      = (x1+x2)/2, (y1+y2)/2
            conf        = float(res.boxes.conf[idx])

            pt = Point();  pt.x, pt.y, pt.z = cx, cy, conf
            self.pub_pt.publish(pt)

        # 4) 带框图
        annotated = res.plot()
        out = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        out.header = msg.header
        self.pub_img.publish(out)


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node();  rclpy.shutdown()


if __name__ == '__main__':
    main()
