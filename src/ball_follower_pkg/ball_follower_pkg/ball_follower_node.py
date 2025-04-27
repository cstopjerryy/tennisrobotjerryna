#!/usr/bin/env python3
"""
ball_follower_node
订阅 : /ball_center
发布 : /cmd_vel
规则 :
    ● 先把球中心 (cx) 转到视野中央 (±deadband)
    ● 已基本居中后再向前走
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped


class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')

        # ---------- 可调参数 ----------
        self.declare_parameter('lin_spd',   0.15)   # 前进速度
        self.declare_parameter('ang_spd',   0.8)    # 最大角速度
        self.declare_parameter('deadband', 40.0)    # 允许误差(像素)
        self.declare_parameter('img_width', 640.0)  # 相机宽

        self.lin = self.get_parameter('lin_spd').value
        self.ang = self.get_parameter('ang_spd').value
        self.db  = self.get_parameter('deadband').value
        self.w   = self.get_parameter('img_width').value

        self.sub = self.create_subscription(
            PointStamped, '/ball_center', self.cb_center, 10)
        self.pub = self.create_publisher(
            Twist,        '/cmd_vel',      10)

        self.get_logger().info('BallFollower started')

    # --------------------------------
    def cb_center(self, msg: PointStamped):
        cx   = msg.point.x
        err  = cx - self.w / 2.0           # + 右  - 左
        twist = Twist()

        # 角速度按比例
        twist.angular.z = - self.ang * (err / (self.w/2.0))

        # 线速度：居中后再走
        if math.fabs(err) < self.db:
            twist.linear.x = self.lin
        else:
            twist.linear.x = 0.0

        self.pub.publish(twist)


def main():
    rclpy.init()
    rclpy.spin(BallFollower())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
