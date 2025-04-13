#!/usr/bin/env python3
import math, rclpy
from rclpy.node         import Node
from geometry_msgs.msg  import Twist, PointStamped
from nav_msgs.msg       import Odometry

ANGLE_THRESH = 0.10      # rad ≈ 6°
DIST_THRESH  = 0.30      # m   到球就停
K_YAW        = 1.0
K_LIN        = 0.5

class BallChaser(Node):
    def __init__(self):
        super().__init__('ball_chaser')

        # 订阅机器人里程计（拿当前位姿）
        self.create_subscription(Odometry, '/odom',
                                 self.odom_cb, 10)
        # 订阅 Ball Oracle 给的网球位置
        self.create_subscription(PointStamped, '/goal_point',
                                 self.ball_cb, 10)

        # 速度发布器
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 运行状态
        self.ball_x = None    # 网球坐标
        self.ball_y = None
        self.x = self.y = self.yaw = 0.0

        # 控制循环 20 Hz
        self.create_timer(0.05, self.loop)

    # ---------------- Callbacks ----------------
    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.yaw = math.atan2(2*qw*qz, 1 - 2*qz*qz)

    def ball_cb(self, msg: PointStamped):
        self.ball_x = msg.point.x
        self.ball_y = msg.point.y

    # ---------------- Main Loop ----------------
    def loop(self):
        if self.ball_x is None:        # 还没收到网球信息
            return

        dx = self.ball_x - self.x
        dy = self.ball_y - self.y
        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)

        yaw_err = target_yaw - self.yaw
        # wrap 到 [-π, π]
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

        cmd = Twist()

        # 阶段 1：对准
        if abs(yaw_err) > ANGLE_THRESH:
            cmd.angular.z = K_YAW * yaw_err
            # 不前进
        # 阶段 2：前进
        elif dist > DIST_THRESH:
            cmd.linear.x  = K_LIN * dist
            cmd.angular.z = K_YAW * yaw_err
        # 阶段 3：到达
        else:
            self.get_logger().info('🎾  Ball reached, stopping.')
            # cmd 默认为 0

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(BallChaser())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
