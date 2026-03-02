#!/usr/bin/env python3
"""drone1_sim — driveable drone, circular orbit with altitude variation."""

import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist

RADIUS   = 5.0
OMEGA    = 0.12
ALT_BASE = 3.0
ALT_AMP  = 1.2
ALT_FREQ = 0.07
HZ       = 10

class Drone1Sim(Node):
    def __init__(self):
        super().__init__('drone1_sim')
        self.pub = self.create_publisher(Odometry, '/drone1/odom', 10)
        self.sub = self.create_subscription(Twist, '/drone1/cmd_vel', self.on_cmd, 10)
        self.create_timer(1.0 / HZ, self.tick)

        self.t = 0.0
        self.manual_x   = RADIUS
        self.manual_y   = 0.0
        self.manual_z   = ALT_BASE
        self.manual_yaw = math.pi / 2
        self.cmd_vx     = 0.0
        self.cmd_vz     = 0.0
        self.cmd_wz     = 0.0
        self.last_cmd   = 0.0

        self.get_logger().info('drone1_sim ready')

    def on_cmd(self, msg: Twist):
        self.cmd_vx   = msg.linear.x
        self.cmd_vz   = msg.linear.z    # vertical throttle
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
            self.manual_z    = max(0.5, self.manual_z + self.cmd_vz * dt)
            x, y, z, yaw = self.manual_x, self.manual_y, self.manual_z, self.manual_yaw
        else:
            x   = RADIUS * math.cos(OMEGA * self.t)
            y   = RADIUS * math.sin(OMEGA * self.t)
            z   = ALT_BASE + ALT_AMP * math.sin(2 * math.pi * ALT_FREQ * self.t)
            yaw = OMEGA * self.t + math.pi / 2
            self.manual_x, self.manual_y, self.manual_z, self.manual_yaw = x, y, z, yaw

        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'drone1/base_link'
        msg.pose.pose.position.x    = x
        msg.pose.pose.position.y    = y
        msg.pose.pose.position.z    = z
        msg.pose.pose.orientation   = yaw_to_quat(yaw)
        self.pub.publish(msg)


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2)
    q.w = math.cos(yaw / 2)
    return q

def main():
    rclpy.init()
    node = Drone1Sim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
