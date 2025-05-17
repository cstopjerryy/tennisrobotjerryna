#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ball_follower_node.py  –  Strategy A with capped angular speed & distance‑based linear speed
改动要点
~~~~~~~~
1. 新增参数   max_wz   – 角速度绝对上限 (rad/s)。
2. 新增参数   stop_area_ratio – 当 bbox 面积 / 画面面积 ≥ ratio 时停止并触发拾球（TODO）。
3. 新增参数   v_scale_with_area – 线速度根据 bbox 面积对数下降，靠近时减速。
4. err_pix → 通过 self.img_w 更新；对 err 做限幅 → 防止因误差瞬变猛转。
"""
"""
ball_follower_node.py  v2
=========================

SEARCH  →  ALIGN  →  CHASE 三状态逻辑：

1. SEARCH：原地慢速旋转找球。
2. ALIGN ：看到球但未对准中心，原地转向直到 |err| ≤ align_pix。
3. CHASE ：角度误差足够小 → 匀速前进同时小幅修正角度。

参数一览
-------
img_width        图像宽度像素       (1280)
linear_spd       CHASE 直线速度 m/s (0.12)
ang_align_gain   ALIGN P 增益       (1.2)
ang_chase_gain   CHASE P 增益       (0.6)
search_wz        SEARCH 转速 rad/s  (0.35)
align_pix        对准阈值像素误差    (100)
lost_timeout     判定丢球时间 s      (1.0)
min_conf         最小置信度          (0.25)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point

class BallFollower(Node):
    SEARCH, TRACK = range(2)

    def __init__(self):
        super().__init__('ball_follower_simple')

        # ---------------- 参数 ----------------
        self.declare_parameters('', [
            ('img_width',     1280),   # 相机图像宽度(px)
            ('linear_spd',    0.18),   # 前进速度 (m/s)
            ('ang_gain',      0.8),    # 角速度比例增益
            ('max_wz',        0.6),    # 角速度上限 (rad/s)
            ('search_wz',     0.3),    # SEARCH 时自旋速度 (rad/s)
            ('lost_timeout',  1.0),    # 看不见球多久判定丢失 (s)
            ('min_conf',      0.25),   # YOLO 置信度阈
        ])

        self.img_w     = self.get_parameter('img_width').value
        self.v_lin     = self.get_parameter('linear_spd').value
        self.k_ang     = self.get_parameter('ang_gain').value
        self.max_wz    = self.get_parameter('max_wz').value
        self.wz_search = self.get_parameter('search_wz').value
        self.lost_t    = self.get_parameter('lost_timeout').value
        self.min_conf  = self.get_parameter('min_conf').value

        # 状态
        self.state      = self.SEARCH
        self.last_seen = 0.0
        self.err_pix    = 0.0

        # ROS I/O
        self.create_subscription(Point, '/yolo/balls', self.cb_ball, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.loop)  # 10 Hz 控制循环
        self.get_logger().info('✅ BallFollower simple ready')

    # ------------------ 接球回调 ------------------
    def cb_ball(self, pt: Point):
        if pt.z < self.min_conf:
            return
        self.last_seen = self._now()
        self.err_pix   = pt.x - self.img_w/2
        if self.state == self.SEARCH:
            self.state = self.TRACK

    # ------------------ 主循环 ------------------
    def loop(self):
        now   = self._now()
        twist = Twist()

        # 丢球
        if now - self.last_seen > self.lost_t:
            self.state = self.SEARCH

        if self.state == self.SEARCH:
            twist.angular.z = self.wz_search
            twist.linear.x  = 0.0

        elif self.state == self.TRACK:
            # 不再边走边修角 — 检测到球立即沿当前朝向前进
            twist.angular.z = 0.0               # ★ 取消角速度修正
            twist.linear.x  = self.v_lin

        self.pub.publish(twist)

    # ------------------ 工具 ------------------
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _sat(self, val):
        return max(-self.max_wz, min(self.max_wz, val))


def main():
    rclpy.init(); node = BallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()