#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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

    SEARCH, ALIGN, CHASE = range(3)

    def __init__(self):
        super().__init__('ball_follower')

        # ------------------ 参数 ------------------
        self.declare_parameters('', [
            ('img_width',        1280),
            ('linear_spd',       0.12),
            ('ang_align_gain',   1.2),
            ('ang_chase_gain',   0.6),
            ('search_wz',        0.25),
            ('align_pix',        100),
            ('lost_timeout',     1.0),
            ('min_conf',         0.25),
        ])

        self.w           = self.get_parameter('img_width').value
        self.v_lin       = self.get_parameter('linear_spd').value
        self.k_align     = self.get_parameter('ang_align_gain').value
        self.k_chase     = self.get_parameter('ang_chase_gain').value
        self.wz_search   = self.get_parameter('search_wz').value
        self.align_pix   = self.get_parameter('align_pix').value
        self.lost_t      = self.get_parameter('lost_timeout').value
        self.min_conf    = self.get_parameter('min_conf').value

        # 运行时变量
        self.state       = self.SEARCH
        self.last_seen   = 0.0
        self.err_pix     = 0.0

        # ROS 接口
        self.sub = self.create_subscription(Point, '/yolo/balls', self.cb_ball, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.loop)      # 10 Hz

        self.get_logger().info('✅ BallFollower v2 ready')

    # ---------------- 回调：收到网球 ----------------
    def cb_ball(self, pt: Point):
        if pt.z < self.min_conf:
            return
        self.last_seen = self.get_clock().now().nanoseconds / 1e9
        self.err_pix   = pt.x - self.w / 2.0

    # ---------------- 主循环 ----------------
    def loop(self):
        now   = self.get_clock().now().nanoseconds / 1e9
        twist = Twist()

        # ---- 判断是否丢球 ----
        if now - self.last_seen > self.lost_t:
            self.state = self.SEARCH

        # ---- 状态机逻辑 ----
        if self.state == self.SEARCH:
            twist.angular.z = self.wz_search

            # 若突然看到球 → 切 ALIGN
            if now - self.last_seen <= self.lost_t:
                self.state = self.ALIGN

        elif self.state == self.ALIGN:
            err = self.err_pix / (self.w / 2.0)          # 归一化 [-1,1]
            twist.angular.z = - self.k_align * err

            # 对准完成 → 切 CHASE
            if abs(self.err_pix) <= self.align_pix:
                self.state = self.CHASE

        elif self.state == self.CHASE:
            err = self.err_pix / (self.w / 2.0)
            twist.linear.x  = self.v_lin
            twist.angular.z = - self.k_chase * err

            # 若误差再次变大（球偏出中心） → 回 ALIGN
            if abs(self.err_pix) > self.align_pix:
                self.state = self.ALIGN

        self.pub.publish(twist)


# ------------------------------------------------------------------
def main():
    rclpy.init()
    node = BallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
