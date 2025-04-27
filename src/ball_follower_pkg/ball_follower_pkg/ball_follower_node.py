#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ball_follower_node.py
=====================
订阅 `/yolo/balls` 获取“最近网球”像素中心，
让 TurtleBot3  → 视野中央 → 前进。

‣ 行为
    • 若 t 秒内连续收不到 /yolo/balls → 进入原地慢速旋转搜索；
    • 一旦收到且置信度 ≥ min_conf → 角度对齐后匀速前进；
      当误差回到 0 且再次丢失球 → 说明刚“撞上”，切换搜索。

‣ 关键参数
    linear_spd    前进速度 m/s
    angular_gain  角度 P 增益
    search_wz     搜索时转速 rad/s
    lost_timeout  判定“丢球”时间 s
"""
import time, math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point

class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')
        # ---------- 参数 ----------
        self.declare_parameter('img_width',   640)
        self.declare_parameter('linear_spd',  0.10)
        self.declare_parameter('angular_gain',1.0)
        self.declare_parameter('search_wz',   0.5)
        self.declare_parameter('lost_timeout',1.0)
        self.declare_parameter('min_conf',    0.25)

        self.width       = self.get_parameter('img_width').value
        self.lin_spd     = self.get_parameter('linear_spd').value
        self.k_ang       = self.get_parameter('angular_gain').value
        self.search_wz   = self.get_parameter('search_wz').value
        self.lost_t      = self.get_parameter('lost_timeout').value
        self.min_conf    = self.get_parameter('min_conf').value

        self.last_seen   = 0.0          # 上次看到球的时间戳
        self.err_x_norm  = 0.0          # 归一化中心误差 [-1,1]

        # ---------- ROS ----------
        self.sub = self.create_subscription(Point,'/yolo/balls',self.cb_ball,10)
        self.pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.timer = self.create_timer(0.1, self.control_loop)   # 10Hz

        self.get_logger().info('✅ ball_follower ready')

    # --------------------------------------------------------------
    def cb_ball(self, pt: Point):
        if pt.z < self.min_conf:          # 低置信度忽略
            return
        self.last_seen = self.get_clock().now().nanoseconds / 1e9
        # x 像素 → [-1,1]
        self.err_x_norm = (pt.x - self.width/2) / (self.width/2)

    # --------------------------------------------------------------
    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        twist = Twist()

        if now - self.last_seen < self.lost_t:
            # --- 看到球：P 控制角度 + 前进 ---
            twist.angular.z = - self.k_ang * self.err_x_norm
            twist.linear.x  = self.lin_spd
        else:
            # --- 丢球：原地搜索 ---
            twist.angular.z = self.search_wz
            twist.linear.x  = 0.0

        self.pub.publish(twist)


def main():
    rclpy.init()
    node = BallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
