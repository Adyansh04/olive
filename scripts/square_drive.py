#!/usr/bin/env python3
"""Closed-loop square driver for recording demo / test bags.

Drives the robot around the four corners of an axis-aligned square,
`--loops` times, using /ground_truth (world frame) feedback so every loop
overlaps cleanly and stays clear of the maze walls. Turns in place at each
corner, then drives a straight edge -> crisp, repeatable squares on video.

Publishes /cmd_vel. Sim + fusion stack must already be running (e.g. via
`ros2 run olive bringup_test.sh maze`). Used to record demo/test bags of the
robot driving repeatable overlapping loops.

    ros2 run olive square_drive.py --half 7.0 --loops 3 --speed 0.4
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


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class SquareDriver(Node):
    def __init__(self, corners, loops, speed, wz_max, pos_tol, settle):
        super().__init__('olive_square_driver')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose = None
        self.create_subscription(Odometry, '/ground_truth', self.cb, 20)
        self.corners = corners
        self.loops = loops
        self.v = speed
        self.wz_max = wz_max
        self.pos_tol = pos_tol
        self.settle = settle

    def cb(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def publish(self, vx, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.cmd.publish(t)

    def hold(self, seconds):
        """Publish zero velocity for a wall-clock duration (stationary capture)."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    def face(self, tx, ty):
        """Rotate in place until roughly aimed at the target -> crisp corners."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is None:
                continue
            x, y, yaw = self.pose
            yerr = wrap(math.atan2(ty - y, tx - x) - yaw)
            if abs(yerr) < 0.05:
                break
            self.publish(0.0, max(-self.wz_max, min(self.wz_max, 1.5 * yerr)))
        self.publish(0.0, 0.0)

    def goto(self, tx, ty):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is None:
                continue
            x, y, yaw = self.pose
            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)
            if dist < self.pos_tol:
                break
            yerr = wrap(math.atan2(dy, dx) - yaw)
            wz = max(-self.wz_max, min(self.wz_max, 1.5 * yerr))
            if abs(yerr) > 0.30:
                vx = 0.0                          # realign before moving on
            else:
                vx = self.v * max(0.2, math.cos(yerr))
                vx = min(vx, 0.6 * dist + 0.05)   # ease into the corner
            self.publish(vx, wz)
        self.publish(0.0, 0.0)

    def run(self):
        while self.pose is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        x0, y0, _ = self.pose
        print(f"[square] start pose (ground_truth/world): ({x0:+.2f}, {y0:+.2f})", flush=True)
        print(f"[square] corners={self.corners} loops={self.loops} "
              f"speed={self.v} m/s", flush=True)

        print(f"[square] settling {self.settle:.0f}s (stationary, for IMU-init capture)", flush=True)
        self.hold(self.settle)

        # Start at the corner nearest the spawn, traverse the cycle from there.
        start = min(range(4), key=lambda i: math.hypot(
            self.corners[i][0] - x0, self.corners[i][1] - y0))
        print(f"[square] approaching start corner {start} {self.corners[start]}", flush=True)
        self.face(*self.corners[start])
        self.goto(*self.corners[start])

        for lp in range(self.loops):
            t = time.monotonic()
            for k in range(4):
                idx = (start + 1 + k) % 4
                self.face(*self.corners[idx])
                self.goto(*self.corners[idx])
                px, py, _ = self.pose
                print(f"[square] loop {lp + 1}/{self.loops}  corner {idx} "
                      f"{self.corners[idx]} -> reached ({px:+.2f}, {py:+.2f})", flush=True)
            print(f"[square] --- loop {lp + 1} complete in {time.monotonic() - t:.0f}s ---",
                  flush=True)

        self.hold(2.0)
        print("[square] DONE", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--half', type=float, default=7.0, help='half-side; corners at (+-half, +-half)')
    ap.add_argument('--loops', type=int, default=3)
    ap.add_argument('--speed', type=float, default=0.4, help='straight-line speed (m/s)')
    ap.add_argument('--wz-max', type=float, default=0.9, help='max yaw rate (rad/s)')
    ap.add_argument('--pos-tol', type=float, default=0.30, help='corner arrival tolerance (m)')
    ap.add_argument('--settle', type=float, default=6.0, help='stationary hold before driving (s)')
    args = ap.parse_args()

    h = args.half
    # CCW cycle: bottom-right -> top-right -> top-left -> bottom-left
    corners = [(h, -h), (h, h), (-h, h), (-h, -h)]

    rclpy.init()
    node = SquareDriver(corners, args.loops, args.speed, args.wz_max, args.pos_tol, args.settle)
    try:
        node.run()
    finally:
        node.publish(0.0, 0.0)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
