#!/usr/bin/env python3
"""Inject wheel-odometry error: republish /odom with slip / scale / heading drift.

Emulates a slipping or miscalibrated wheel odometry (the #1 real wheel-odom
failure) WITHOUT touching sim friction -- the diff-drive robot's only traction
is its two drive wheels (the casters are frictionless), so lowering their
friction just makes it undriveable. Here the physical robot and /ground_truth
stay perfect; only the wheel *measurement* the fusion consumes is corrupted, so
this isolates one question: do the lidar / IMU / marker factors keep the fused
estimate accurate when the wheel odom lies?

Integrates a corrupted odometry from the clean one in the body frame: each step
scales the forward distance, adds a heading drift proportional to distance, and
adds a little noise -- so error accumulates like real wheel odom.

    ros2 run olive wheel_odom_relay.py --scale 1.15 --yaw-drift-deg-per-m 0.3
Then run the fusion with wheel_odom_topic:=<--out> (default /odom_noisy).
"""
import argparse
import math
import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class WheelOdomRelay(Node):
    def __init__(self, a):
        super().__init__('wheel_odom_relay')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a = a
        random.seed(a.seed)
        self.prev = None                 # (x, y, yaw) of previous clean msg
        self.cx = self.cy = self.cyaw = None   # accumulated corrupted pose
        self.n = 0
        self.pub = self.create_publisher(Odometry, a.out, 10)
        self.create_subscription(Odometry, a.in_topic, self.cb, 10)
        self.get_logger().info(
            f"corrupting {a.in_topic} -> {a.out}: scale={a.scale}, "
            f"yaw_drift={a.yaw_drift_deg_per_m} deg/m, noise_xy={a.noise_xy} m")

    def publish(self, src):
        m = Odometry()
        m.header = src.header
        m.child_frame_id = src.child_frame_id
        m.pose.pose.position.x = self.cx
        m.pose.pose.position.y = self.cy
        m.pose.pose.position.z = src.pose.pose.position.z
        m.pose.pose.orientation.z = math.sin(self.cyaw / 2.0)
        m.pose.pose.orientation.w = math.cos(self.cyaw / 2.0)
        m.twist = src.twist            # velocities left as-is
        self.pub.publish(m)

    def cb(self, m):
        x, y, yaw = m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation)
        if self.prev is None:
            self.prev = (x, y, yaw)
            self.cx, self.cy, self.cyaw = x, y, yaw
            self.publish(m)
            return
        px, py, pyaw = self.prev
        # clean body-frame increment: signed forward distance + heading change
        ds = (x - px) * math.cos(pyaw) + (y - py) * math.sin(pyaw)
        dyaw = wrap(yaw - pyaw)
        # corrupt it
        ds_c = ds * self.a.scale + random.gauss(0.0, self.a.noise_xy)
        dyaw_c = (dyaw * self.a.yaw_scale
                  + math.radians(self.a.yaw_drift_deg_per_m) * abs(ds)
                  + random.gauss(0.0, math.radians(self.a.noise_yaw_deg)))
        # midpoint-integrate the corrupted increment into the corrupted pose
        self.cx += ds_c * math.cos(self.cyaw + dyaw_c * 0.5)
        self.cy += ds_c * math.sin(self.cyaw + dyaw_c * 0.5)
        self.cyaw = wrap(self.cyaw + dyaw_c)
        self.prev = (x, y, yaw)
        self.n += 1
        self.publish(m)
        if self.n % 200 == 0:
            drift = math.hypot(self.cx - x, self.cy - y)
            self.get_logger().info(f"wheel-odom drift from truth-frame clean odom: {drift:.2f} m")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--in-topic', default='/odom')
    ap.add_argument('--out', default='/odom_noisy')
    ap.add_argument('--scale', type=float, default=1.15, help='forward-distance scale error')
    ap.add_argument('--yaw-scale', type=float, default=1.0, help='heading-change scale error')
    ap.add_argument('--yaw-drift-deg-per-m', type=float, default=0.3,
                    help='systematic heading drift per metre travelled')
    ap.add_argument('--noise-xy', type=float, default=0.003, help='per-step forward noise (m, 1-sigma)')
    ap.add_argument('--noise-yaw-deg', type=float, default=0.05, help='per-step yaw noise (deg, 1-sigma)')
    ap.add_argument('--seed', type=int, default=42)
    args = ap.parse_args()

    rclpy.init()
    node = WheelOdomRelay(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
