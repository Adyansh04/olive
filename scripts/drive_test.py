#!/usr/bin/env python3
"""Relative-accuracy drive test for the fusion stack.

Drives a scripted pattern and compares the fused estimate against ground
truth on displacement, heading change and integrated path length. Frame
offsets cancel out, so this works before any marker anchoring.

Usage (sim running, fusion stack running):
    ros2 run olive drive_test.py [--long]
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DriveTest(Node):
    def __init__(self, fused_topic, gt_topic):
        super().__init__('olive_drive_test')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.fused = []
        self.gt = []
        self.create_subscription(Odometry, fused_topic, self.cb_fused, 10)
        self.create_subscription(Odometry, gt_topic, self.cb_gt, 10)

    def cb_fused(self, m):
        p = m.pose.pose
        self.fused.append((p.position.x, p.position.y, yaw_of(p.orientation)))

    def cb_gt(self, m):
        p = m.pose.pose
        self.gt.append((p.position.x, p.position.y, yaw_of(p.orientation)))

    def drive(self, vx, wz, seconds):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self, long_mode):
        self.drive(0.0, 0.0, 3.0)
        n_fused0, n_gt0 = len(self.fused), len(self.gt)
        if long_mode:
            self.drive(0.3, 0.0, 6.0)
            self.drive(0.0, -0.5, 3.0)
            self.drive(0.3, 0.0, 5.0)
            self.drive(0.0, 0.5, 3.0)
            self.drive(0.25, 0.3, 6.0)   # arc
            self.drive(0.3, 0.0, 4.0)
            self.drive(0.0, 0.5, 3.2)
            self.drive(0.3, 0.0, 5.0)
        else:
            self.drive(0.3, 0.0, 6.0)
            self.drive(0.0, 0.5, 3.2)
            self.drive(0.3, 0.0, 6.0)
        self.drive(0.0, 0.0, 2.0)

        if not self.fused or not self.gt or n_fused0 == 0 or n_gt0 == 0:
            print(f"FAIL: missing data fused={len(self.fused)} gt={len(self.gt)}")
            return 1

        def stats(track, n0):
            x0, y0, yaw0 = track[max(0, n0 - 1)]
            x1, y1, yaw1 = track[-1]
            disp = math.hypot(x1 - x0, y1 - y0)
            dyaw = math.degrees((yaw1 - yaw0 + math.pi) % (2 * math.pi) - math.pi)
            path = 0.0
            prev = track[max(0, n0 - 1)]
            for p in track[n0:]:
                path += math.hypot(p[0] - prev[0], p[1] - prev[1])
                prev = p
            return disp, dyaw, path

        f_disp, f_dyaw, f_path = stats(self.fused, n_fused0)
        g_disp, g_dyaw, g_path = stats(self.gt, n_gt0)
        print(f"samples: fused={len(self.fused)} gt={len(self.gt)}")
        print(f"fused: disp={f_disp:.3f} m  dyaw={f_dyaw:+.1f} deg  path={f_path:.3f} m")
        print(f"gt   : disp={g_disp:.3f} m  dyaw={g_dyaw:+.1f} deg  path={g_path:.3f} m")
        print(f"errs : disp={abs(f_disp - g_disp):.3f} m  dyaw={abs(f_dyaw - g_dyaw):.2f} deg  "
              f"path={abs(f_path - g_path):.3f} m")
        return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--long', action='store_true', help='longer multi-turn pattern with an arc')
    parser.add_argument('--fused-topic', default='/olive/odometry')
    parser.add_argument('--gt-topic', default='/ground_truth')
    args = parser.parse_args()

    rclpy.init()
    node = DriveTest(args.fused_topic, args.gt_topic)
    code = node.run(getattr(args, 'long'))
    rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
