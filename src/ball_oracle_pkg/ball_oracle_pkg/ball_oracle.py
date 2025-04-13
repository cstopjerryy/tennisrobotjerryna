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
        if BALL_NAME not in msg.name:
            self.get_logger().warn('ball not in model_states!')
            return
        idx = msg.name.index(BALL_NAME)
        pose = msg.pose[idx]

        goal = PointStamped()
        goal.header.frame_id = 'map'     # 直接用 map/world 坐标
        goal.point.x = pose.position.x
        goal.point.y = pose.position.y
        self.pub.publish(goal)

def main():
    rclpy.init()
    rclpy.spin(BallOracle())
