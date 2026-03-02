#!/usr/bin/env python3
"""drone2_sim — driveable drone, elliptical orbit, opposite phase from drone1."""

import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist

RAD_X    = 6.0
RAD_Y    = 3.5
OMEGA    = 0.09
PHASE    = math.pi
ALT_BASE = 4.5
ALT_AMP  = 1.5
ALT_FREQ = 0.05
HZ       = 10

class Drone2Sim(Node):
    def __init__(self):
        super().__init__('drone2_sim')
        self.pub = self.create_publisher(Odometry, '/drone2/odom', 10)
        self.sub = self.create_subscription(Twist, '/drone2/cmd_vel', self.on_cmd, 10)
        self.create_timer(1.0 / HZ, self.tick)

        self.t = 0.0
        self.manual_x   = RAD_X * math.cos(PHASE)
        self.manual_y   = RAD_Y * math.sin(PHASE)
        self.manual_z   = ALT_BASE
        self.manual_yaw = 0.0
        self.cmd_vx     = 0.0
        self.cmd_vz     = 0.0
        self.cmd_wz     = 0.0
        self.last_cmd   = 0.0

        self.get_logger().info('drone2_sim ready')

    def on_cmd(self, msg: Twist):
        self.cmd_vx   = msg.linear.x
        self.cmd_vz   = msg.linear.z
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
            angle = OMEGA * self.t + PHASE
            x   = RAD_X * math.cos(angle)
            y   = RAD_Y * math.sin(angle)
            z   = ALT_BASE + ALT_AMP * math.sin(2 * math.pi * ALT_FREQ * self.t + 1.0)
            yaw = math.atan2(-RAD_Y * math.cos(angle) * OMEGA,
                             -RAD_X * math.sin(angle) * OMEGA)
            self.manual_x, self.manual_y, self.manual_z, self.manual_yaw = x, y, z, yaw

        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'drone2/base_link'
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
    node = Drone2Sim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
