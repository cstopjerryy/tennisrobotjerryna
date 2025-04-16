#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PointStamped

BALL_NAME = 'tennis_ball_1'

class BallOracle(Node):
    def __init__(self):
        super().__init__('ball_oracle')
        self.sub = self.create_subscription(ModelStates,
                                            '/model_states',
                                            self.cb, 10)
        self.pub = self.create_publisher(PointStamped,
                                         '/goal_point', 10)

def cb(self, msg):
# 获取所有网球的 index 和位置
    ball_indices = [i for i, name in enumerate(msg.name) if name.startswith("tennis_ball_")]
    if not ball_indices:
        self.get_logger().warn('no tennis balls found!')
        return

    # 获取机器人自己位置
    try:
        robot_idx = msg.name.index("turtlebot3_waffle_pi")
        robot_pose = msg.pose[robot_idx]
        rx, ry = robot_pose.position.x, robot_pose.position.y
    except ValueError:
        self.get_logger().warn('robot model not found!')
        return

    # 找最近的球
    closest_idx = min(ball_indices, key=lambda i: self.dist(msg.pose[i].position.x, msg.pose[i].position.y, rx, ry))
    ball_pose = msg.pose[closest_idx]

    # 发布 goal_point
    goal = PointStamped()
    goal.header.frame_id = 'map'
    goal.point.x = ball_pose.position.x
    goal.point.y = ball_pose.position.y
    self.pub.publish(goal)

    self.get_logger().info(f"Goal: tennis_ball_{closest_idx} at ({goal.point.x:.2f}, {goal.point.y:.2f})")

def dist(self, x1, y1, x2, y2):
    return (x1 - x2) ** 2 + (y1 - y2) ** 2


def main():
    rclpy.init()
    rclpy.spin(BallOracle())
