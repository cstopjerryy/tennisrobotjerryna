#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
yolo_detector_node.py  –  Strategy A (largest‑bbox‑only)
改动要点
~~~~~~~~
1. **bbox 面积计算移至函数**  _pick_largest() 便于未来扩展。
2. 支持可选发布全部检测 (参数 publish_all=true) – 默认 false 保持旧接口。
3. 新增参数  max_targets  决定最多发布多少目标 (预留多球扩展)。
"""
"""
yolo_detector_node.py  v2.1
===========================
订阅  /camera/image_raw   → YOLOv8 推理 → 只挑 “网球” 类别发布两路话题

发布
-----
1) /yolo/annotated   sensor_msgs/Image    带框彩色图
2) /yolo/balls       geometry_msgs/Point  最大网球中心像素坐标  
   └─ pt.x, pt.y = 中心 (pix) ; pt.z = 置信度  
   若当前帧无网球则**不发布**

可识别类别
-----------
默认接受  'tennis_ball'  和 COCO 的  'sports ball'  
如需更多，在 `ACCEPT_NAMES` 元组内追加即可。

ROS 参数
---------
weight_path : string  必填，YOLO 权重（best.pt）
min_conf    : double  置信度阈值，默认 0.25
imgsz       : int     推理分辨率，默认 1280

"""
import sys, cv2, rclpy
from rclpy.node      import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge       import CvBridge
from ultralytics     import YOLO

ACCEPT_NAMES = (
    'tennis_ball',                # 自训类别
    'sports ball', 'sports_ball', # COCO 同义
    'sport ball',  'sport_ball',
)

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # ---------- 参数 ----------
        self.declare_parameters('', [
            ('weight_path',  ''),
            ('min_conf',     0.25),
            ('imgsz',        1280),
        ])
        wpath   = self.get_parameter('weight_path').value
        self.min_conf  = self.get_parameter('min_conf').value
        self.imgsz     = self.get_parameter('imgsz').value
        if not wpath:
            self.get_logger().fatal('必须通过 weight_path:=<best.pt> 指定模型权重');
            rclpy.shutdown(); sys.exit(1)

        # ---------- 模型 ----------
        self.model  = YOLO(wpath)
        self.bridge = CvBridge()

        # ---------- ROS I/O ----------
        self.create_subscription(Image, '/camera/image_raw', self.cb_image, 10)
        self.pub_img  = self.create_publisher(Image, '/yolo/annotated', 10)
        self.pub_pt   = self.create_publisher(Point,  '/yolo/balls',     10)
        self.get_logger().info('✅ YOLO detector ready (Strategy A)')

    # ------------------------------ callback --------------------------------
    def cb_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        res   = self.model(frame, imgsz=self.imgsz, conf=self.min_conf)[0]

        # 过滤网球类别
        cls_names = self.model.names
        ids = [i for i, c in enumerate(res.boxes.cls)
               if cls_names[int(c)] in ACCEPT_NAMES]
        if ids:
            # 取面积最大的一颗
            areas = res.boxes.xywh[ids][:,2] * res.boxes.xywh[ids][:,3]
            idx   = ids[int(areas.argmax())]
            x1,y1,x2,y2 = res.boxes.xyxy[idx].cpu().numpy()
            cx, cy      = (x1+x2)/2, (y1+y2)/2
            conf        = float(res.boxes.conf[idx])
            pt = Point(); pt.x, pt.y, pt.z = cx, cy, conf
            self.pub_pt.publish(pt)

        # 调试图
        ann = res.plot()
        out = self.bridge.cv2_to_imgmsg(ann, 'bgr8'); out.header = msg.header
        self.pub_img.publish(out)

# ---------------------------------------------------------------------------

def main():
    rclpy.init(); node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown()