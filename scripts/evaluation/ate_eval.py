#!/usr/bin/env python3
"""Full-trajectory absolute accuracy evaluation for the fusion stack.

Drives a long tour that includes a marker anchoring opportunity while
continuously sampling the fused estimate and ground truth, time-associates
the two streams, and reports position / yaw error statistics per phase:

- pre-anchor: the map frame starts at the spawn pose, so the absolute error
  is the (constant) frame offset; its variation is the pure odometry drift
- post-anchor: absolute error in the world frame (mean / RMSE / max)

Usage (sim running, fusion stack with markers enabled):
    ros2 run olive ate_eval.py
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


def stamp_of(m):
    return m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec


class AteEval(Node):
    def __init__(self, fused_topic, gt_topic):
        super().__init__('olive_ate_eval')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.fused = []   # (t, x, y, yaw)
        self.gt = []
        self.create_subscription(Odometry, fused_topic, self.cb_fused, 50)
        self.create_subscription(Odometry, gt_topic, self.cb_gt, 50)

    def cb_fused(self, m):
        p = m.pose.pose
        self.fused.append((stamp_of(m), p.position.x, p.position.y, yaw_of(p.orientation)))

    def cb_gt(self, m):
        p = m.pose.pose
        self.gt.append((stamp_of(m), p.position.x, p.position.y, yaw_of(p.orientation)))

    def drive(self, vx, wz, seconds):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        # Settle, wander (spawn-relative phase), anchor on the SW marker,
        # then a long post-anchor tour. Tuned for the maze world spawn.
        self.drive(0.0, 0.0, 3.0)
        self.drive(0.3, 0.0, 5.0)
        self.drive(0.0, 0.5, 3.2)
        self.drive(0.3, 0.0, 4.0)
        self.drive(0.0, 0.5, 4.6)     # face the SW corner marker
        self.drive(0.25, 0.0, 6.0)    # approach into decoding range
        self.drive(0.0, 0.0, 4.0)     # hold: persistence gate + keyframe
        self.drive(0.0, 0.5, 3.2)
        self.drive(0.3, 0.0, 6.0)
        self.drive(0.0, -0.5, 3.0)
        self.drive(0.3, 0.0, 5.0)
        self.drive(0.25, 0.3, 6.0)    # arc
        self.drive(0.3, 0.0, 4.0)
        self.drive(0.0, 0.5, 3.0)
        self.drive(0.3, 0.0, 5.0)
        self.drive(0.0, 0.0, 2.0)
        return self.evaluate()

    def evaluate(self):
        if len(self.fused) < 10 or len(self.gt) < 10:
            print(f"FAIL: fused={len(self.fused)} gt={len(self.gt)}")
            return 1

        errors = []  # (t, pos_err, yaw_err_deg)
        gi = 0
        for t, x, y, yaw in self.fused:
            while gi + 1 < len(self.gt) and abs(self.gt[gi + 1][0] - t) <= abs(self.gt[gi][0] - t):
                gi += 1
            gt = self.gt[gi]
            if abs(gt[0] - t) > 0.05:
                continue
            pos_err = math.hypot(x - gt[1], y - gt[2])
            yaw_err = math.degrees((yaw - gt[3] + math.pi) % (2 * math.pi) - math.pi)
            errors.append((t, pos_err, yaw_err))

        anchor_i = next((i for i, e in enumerate(errors) if e[1] < 1.0), None)

        def stats(rows, label):
            pos = [r[1] for r in rows]
            yaw = [abs(r[2]) for r in rows]
            rmse = math.sqrt(sum(p * p for p in pos) / len(pos))
            yaw_rmse = math.sqrt(sum(v * v for v in yaw) / len(yaw))
            print(f"{label}: n={len(rows)}")
            print(f"   position |err|: mean={sum(pos)/len(pos):.3f}  rmse={rmse:.3f}  "
                  f"max={max(pos):.3f} m")
            print(f"   yaw      |err|: mean={sum(yaw)/len(yaw):.2f}  rmse={yaw_rmse:.2f}  "
                  f"max={max(yaw):.2f} deg")

        print(f"total samples associated: {len(errors)} "
              f"(fused {len(self.fused)}, gt {len(self.gt)})")
        if anchor_i is None:
            print("NO ANCHORING OBSERVED — all errors include the spawn-frame offset")
            stats(errors, "whole run (spawn-relative frame)")
            return 1

        pre = errors[:anchor_i]
        post = errors[anchor_i:]
        print(f"anchor event at t={errors[anchor_i][0]:.1f}s "
              f"(error {pre[-1][1]:.2f} m -> {post[0][1]:.2f} m)")
        if pre:
            drift = [abs(r[1] - pre[0][1]) for r in pre]
            yaw = [abs(r[2]) for r in pre]
            print(f"pre-anchor (spawn frame, n={len(pre)}): frame offset {pre[0][1]:.3f} m; "
                  f"drift about it: mean={sum(drift)/len(drift)*100:.1f} cm, "
                  f"max={max(drift)*100:.1f} cm; |yaw err| max={max(yaw):.2f} deg")
        stats(post, "post-anchor ABSOLUTE (world frame)")
        return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fused-topic', default='/olive/odometry')
    parser.add_argument('--gt-topic', default='/ground_truth')
    args = parser.parse_args()

    rclpy.init()
    node = AteEval(args.fused_topic, args.gt_topic)
    code = node.run()
    rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
