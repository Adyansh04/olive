#!/usr/bin/env python3
"""Drift-reset demonstration: marker anchoring in action.

Reports the absolute |fused - ground_truth| error at four checkpoints:
before driving (map frame = spawn frame, large constant offset), after a
wander loop, after sighting a known marker (the trajectory snaps into the
world frame) and after driving on (the anchor holds).

Usage (sim running, fusion stack with markers enabled, maze world spawn):
    ros2 run olive marker_demo.py
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MarkerDemo(Node):
    def __init__(self):
        super().__init__('olive_marker_demo')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.fused = None
        self.gt = None
        self.create_subscription(Odometry, '/olive/odometry', self.cb_fused, 10)
        self.create_subscription(Odometry, '/ground_truth', self.cb_gt, 10)

    def cb_fused(self, m):
        p = m.pose.pose
        self.fused = (p.position.x, p.position.y, yaw_of(p.orientation))

    def cb_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))

    def drive(self, vx, wz, seconds):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def report(self, label):
        if not self.fused or not self.gt:
            print(f"{label}: no data")
            return
        dx = self.fused[0] - self.gt[0]
        dy = self.fused[1] - self.gt[1]
        dyaw = math.degrees(
            (self.fused[2] - self.gt[2] + math.pi) % (2 * math.pi) - math.pi)
        print(f"{label}: |pos err| = {math.hypot(dx, dy):.3f} m, yaw err = {dyaw:+.2f} deg  "
              f"(fused {self.fused[0]:+.2f},{self.fused[1]:+.2f} | "
              f"gt {self.gt[0]:+.2f},{self.gt[1]:+.2f})")

    def run(self):
        self.drive(0.0, 0.0, 3.0)
        self.report("start (map frame = spawn frame, expect a large constant offset)")

        self.drive(0.3, 0.0, 5.0)
        self.drive(0.0, 0.5, 3.2)
        self.drive(0.3, 0.0, 4.0)
        self.report("pre-anchor (still spawn-relative)")

        # Face the SW corner marker and approach into decoding range.
        self.drive(0.0, 0.5, 4.6)
        self.drive(0.25, 0.0, 6.0)
        self.drive(0.0, 0.0, 4.0)
        self.report("post-anchor (expect well under 0.3 m absolute)")

        self.drive(0.0, 0.5, 3.2)
        self.drive(0.3, 0.0, 4.0)
        self.drive(0.0, 0.0, 2.0)
        self.report("post-anchor after more driving")


def main():
    rclpy.init()
    node = MarkerDemo()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
