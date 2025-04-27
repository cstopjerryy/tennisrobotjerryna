#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist

class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/yolo/annotated', self.cb_image, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 你可以改这个速度
        self.linear_speed = 0.1
        self.angular_speed = 0.5
        self.get_logger().info('BallFollower ready')

    def cb_image(self, msg: Image):
        # 把带框图转 cv
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # 假设 YOLO 在框里画的是绿色线条，或者你可以检测 boxes
        # 这里我们就粗糙地用 HoughCircles 检个灰度圆心（仅示例）
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
            param1=50, param2=30, minRadius=10, maxRadius=50)
        twist = Twist()
        if circles is not None:
            # 取第一个圆
            x, y, r = circles[0][0]
            err = x - img.shape[1]/2
            twist.linear.x = self.linear_speed
            twist.angular.z = -float(err) / (img.shape[1]/2) * self.angular_speed
        else:
            # 找不到球就停
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        self.pub.publish(twist)

def main():
    rclpy.init()
    node = BallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
