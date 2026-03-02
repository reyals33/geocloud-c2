#!/usr/bin/env python3
"""robot2_sim — driveable ground robot, figure-8 autonomous path."""

import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist

SCALE = 3.5
OMEGA = 0.14
HZ    = 10

class Robot2Sim(Node):
    def __init__(self):
        super().__init__('robot2_sim')
        self.pub = self.create_publisher(Odometry, '/robot2/odom', 10)
        self.sub = self.create_subscription(Twist, '/robot2/cmd_vel', self.on_cmd, 10)
        self.create_timer(1.0 / HZ, self.tick)

        self.t = 0.0
        self.manual_x   = SCALE
        self.manual_y   = 0.0
        self.manual_yaw = 0.0
        self.cmd_vx     = 0.0
        self.cmd_wz     = 0.0
        self.last_cmd   = 0.0

        self.get_logger().info('robot2_sim ready')

    def on_cmd(self, msg: Twist):
        self.cmd_vx   = msg.linear.x
        self.cmd_wz   = msg.angular.z
        self.last_cmd = time.time()

    def tick(self):
        dt = 1.0 / HZ
        self.t += dt

        manual = (time.time() - self.last_cmd) < 1.0

        if manual:
            self.manual_yaw += self.cmd_wz * dt
            self.manual_x   += self.cmd_vx * math.cos(self.manual_yaw) * dt
            self.manual_y   += self.cmd_vx * math.sin(self.manual_yaw) * dt
            x, y, yaw = self.manual_x, self.manual_y, self.manual_yaw
        else:
            # Lemniscate figure-8
            denom = 1 + math.sin(OMEGA * self.t) ** 2
            x = SCALE * math.cos(OMEGA * self.t) / denom
            y = SCALE * math.sin(OMEGA * self.t) * math.cos(OMEGA * self.t) / denom
            t2 = self.t + 0.01
            d2 = 1 + math.sin(OMEGA * t2) ** 2
            xn = SCALE * math.cos(OMEGA * t2) / d2
            yn = SCALE * math.sin(OMEGA * t2) * math.cos(OMEGA * t2) / d2
            yaw = math.atan2(yn - y, xn - x)
            self.manual_x, self.manual_y, self.manual_yaw = x, y, yaw

        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'robot2/base_link'
        msg.pose.pose.position.x    = x
        msg.pose.pose.position.y    = y
        msg.pose.pose.position.z    = 0.0
        msg.pose.pose.orientation   = yaw_to_quat(yaw)
        self.pub.publish(msg)


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2)
    q.w = math.cos(yaw / 2)
    return q

def main():
    rclpy.init()
    node = Robot2Sim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
