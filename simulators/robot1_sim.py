#!/usr/bin/env python3
"""
robot1_sim — driveable ground robot simulator.

Manual control: publishes cmd_vel → robot drives that direction.
After 1s with no cmd_vel input, resumes autonomous circular path.
"""

import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist

RADIUS = 3.5    # autonomous orbit radius (m)
OMEGA  = 0.18   # autonomous orbit speed (rad/s)
HZ     = 10

class Robot1Sim(Node):
    def __init__(self):
        super().__init__('robot1_sim')
        self.pub = self.create_publisher(Odometry, '/robot1/odom', 10)
        self.sub = self.create_subscription(Twist, '/robot1/cmd_vel', self.on_cmd, 10)
        self.create_timer(1.0 / HZ, self.tick)

        self.t = 0.0

        # Manual control state
        self.manual_x   = RADIUS
        self.manual_y   = 0.0
        self.manual_yaw = math.pi / 2
        self.cmd_vx     = 0.0
        self.cmd_wz     = 0.0
        self.last_cmd   = 0.0

        self.get_logger().info('robot1_sim ready')

    def on_cmd(self, msg: Twist):
        self.cmd_vx   = msg.linear.x
        self.cmd_wz   = msg.angular.z
        self.last_cmd = time.time()

    def tick(self):
        dt = 1.0 / HZ
        self.t += dt

        manual = (time.time() - self.last_cmd) < 1.0

        if manual:
            # Integrate pose from cmd_vel
            self.manual_yaw += self.cmd_wz * dt
            self.manual_x   += self.cmd_vx * math.cos(self.manual_yaw) * dt
            self.manual_y   += self.cmd_vx * math.sin(self.manual_yaw) * dt
            x, y, yaw = self.manual_x, self.manual_y, self.manual_yaw
        else:
            # Autonomous circular orbit — keep manual state synced
            x   = RADIUS * math.cos(OMEGA * self.t)
            y   = RADIUS * math.sin(OMEGA * self.t)
            yaw = OMEGA * self.t + math.pi / 2
            self.manual_x, self.manual_y, self.manual_yaw = x, y, yaw

        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'robot1/base_link'
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
    node = Robot1Sim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
