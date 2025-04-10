#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator_node')
        # 订阅里程计
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        # 发布速度话题
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 设定若干waypoints
        self.waypoints = [
            (2.0, 0.0),
            (2.0, 2.0),
            (-2.0, 2.0),
            (-2.0, 0.0),
        ]
        self.index = 0

        # 机器人当前位姿
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # 参数：阈值 & 控制增益
        self.dist_threshold = 0.25
        self.linear_k = 0.3
        self.angular_k = 1.0

        # 定时器循环
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("WaypointNavigator Initialized!")

    def odom_callback(self, msg):
        """ 提取里程计信息，保存给 self.x, self.y, self.yaw """
        # 位置
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # 四元数 -> yaw
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        # yaw = 2*atan2(qz,qw) 也可
        self.yaw = math.atan2(2.0*qw*qz, 1-2*(qz*qz))

    def control_loop(self):
        # 如果所有waypoints都已完成
        if self.index >= len(self.waypoints):
            cmd = Twist()
            self.cmd_pub.publish(cmd)  # 停车
            return

        # 目标waypoint
        tx, ty = self.waypoints[self.index]

        # 计算与目标点的距离
        dx = tx - self.x
        dy = ty - self.y
        distance = math.sqrt(dx*dx + dy*dy)

        # 如果到达该waypoint，切换下一个
        if distance < self.dist_threshold:
            self.get_logger().info(f"Reached waypoint {self.index} at x={self.x:.2f},y={self.y:.2f}")
            self.index += 1
            return

        # 计算期望朝向
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - self.yaw
        # wrap到[-pi, pi]
        if yaw_error > math.pi:
            yaw_error -= 2.0*math.pi
        elif yaw_error < -math.pi:
            yaw_error += 2.0*math.pi

        # 简易P控制
        linear_speed = self.linear_k * distance
        angular_speed = self.angular_k * yaw_error

        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
