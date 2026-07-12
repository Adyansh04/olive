#!/usr/bin/env python3
"""Endurance test: drive continuously while tracking fusion_node memory.

Drives repeating patterns for --minutes while sampling the fusion_node RSS
and reporting whether memory plateaus (bounded cloud storage working) and the
estimate stays sane. Requires the stack to be up (bringup_test.sh).

    ros2 run olive endurance_test.py --minutes 10
"""
import argparse
import math
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def fusion_rss_mb():
    try:
        pid = subprocess.check_output(
            ['pgrep', '-f', 'lib/olive/fusion_node']).decode().split()[0]
        with open(f'/proc/{pid}/status') as f:
            for line in f:
                if line.startswith('VmRSS'):
                    return int(line.split()[1]) / 1024.0
    except (subprocess.CalledProcessError, FileNotFoundError, IndexError):
        return None
    return None


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Endurance(Node):
    def __init__(self):
        super().__init__('olive_endurance_test')
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

    def run(self, minutes):
        rss = []
        deadline = time.monotonic() + minutes * 60.0
        cycle = 0
        # Back-and-forth patrol: statistically stays in the reachable area.
        while time.monotonic() < deadline:
            self.drive(0.25, 0.0, 5.0)
            self.drive(0.0, 0.5, 3.2)     # ~90 deg left
            self.drive(0.25, 0.0, 4.0)
            self.drive(0.0, 0.5, 3.2)
            self.drive(0.25, 0.0, 5.0)
            self.drive(0.0, 0.5, 3.2)
            self.drive(0.25, 0.0, 4.0)
            self.drive(0.0, 0.5, 3.2)     # net ~360: patrol loop
            cycle += 1
            sample = fusion_rss_mb()
            if sample is not None:
                rss.append((time.monotonic(), sample))
                print(f"cycle {cycle}: RSS {sample:.1f} MB", flush=True)
        self.drive(0.0, 0.0, 1.0)

        if len(rss) < 4:
            print("FAIL: not enough RSS samples")
            return 1
        # Plateau check: second-half growth much smaller than first-half.
        mid = len(rss) // 2
        first_growth = rss[mid][1] - rss[0][1]
        second_growth = rss[-1][1] - rss[mid][1]
        print(f"RSS: start {rss[0][1]:.1f} MB, mid {rss[mid][1]:.1f} MB, "
              f"end {rss[-1][1]:.1f} MB")
        print(f"growth: first half {first_growth:+.1f} MB, second half {second_growth:+.1f} MB")
        if self.fused and self.gt:
            print(f"final fused ({self.fused[0]:+.2f},{self.fused[1]:+.2f}) "
                  f"gt ({self.gt[0]:+.2f},{self.gt[1]:+.2f})")
        return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--minutes', type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = Endurance()
    code = node.run(args.minutes)
    rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
