#!/usr/bin/env python3
"""Calibrate the WhyCode detector's range scale against known geometry.

Turns the robot (closed-loop on ground truth) to face a marker whose world
position is known, averages the detected range, compares it against the true
camera-to-marker distance and prints the corrected outer_diameter_multiplier
for the detector configuration.

Usage (sim running, detector running; defaults target the maze SW marker):
    ros2 run olive calibrate_marker_range.py \
        [--marker-id 3] [--marker-x -7.5] [--marker-y -7.5] [--marker-z 1.2] \
        [--current-multiplier 1.096]
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from whycode_vision.msg import WhyCodePoseArray

# Camera optical center relative to the ground-truth base pose (from the URDF)
CAMERA_FORWARD = 0.2
CAMERA_HEIGHT_ABOVE_GROUND = 0.12


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Calibrate(Node):
    def __init__(self, args):
        super().__init__('olive_calibrate_marker_range')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.args = args
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gt = None
        self.ranges = []
        self.create_subscription(Odometry, '/ground_truth', self.cb_gt, 10)
        self.create_subscription(WhyCodePoseArray, '/whycode/poses', self.cb_marker, 10)

    def cb_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))

    def cb_marker(self, m):
        for p in m.poses:
            if p.id_valid and p.whycode_id == self.args.marker_id:
                self.ranges.append(math.sqrt(
                    p.pose.position.x**2 + p.pose.position.y**2 + p.pose.position.z**2))

    def run(self):
        # Face the marker (closed loop on ground-truth heading).
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.gt is None:
                continue
            bearing = math.atan2(self.args.marker_y - self.gt[1],
                                 self.args.marker_x - self.gt[0])
            err = (bearing - self.gt[2] + math.pi) % (2 * math.pi) - math.pi
            if abs(err) < math.radians(1.5):
                break
            msg = Twist()
            msg.angular.z = max(-0.4, min(0.4, 1.2 * err))
            self.cmd_pub.publish(msg)
        self.cmd_pub.publish(Twist())

        # Collect detections while stationary.
        self.ranges.clear()
        end = time.monotonic() + 4.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

        if not self.ranges:
            print(f"FAIL: no valid detections of marker id {self.args.marker_id} "
                  f"(robot yaw {math.degrees(self.gt[2]):.1f} deg) — "
                  "check range/heading and that the detector is running")
            return 1

        x, y, yaw = self.gt
        cx = x + CAMERA_FORWARD * math.cos(yaw)
        cy = y + CAMERA_FORWARD * math.sin(yaw)
        true_range = math.sqrt(
            (self.args.marker_x - cx)**2 + (self.args.marker_y - cy)**2 +
            (self.args.marker_z - CAMERA_HEIGHT_ABOVE_GROUND)**2)
        detected = sum(self.ranges) / len(self.ranges)
        ratio = detected / true_range
        print(f"n={len(self.ranges)}  detected={detected:.4f} m  true={true_range:.4f} m  "
              f"ratio={ratio:.4f}")
        print(f"corrected outer_diameter_multiplier = "
              f"{self.args.current_multiplier / ratio:.4f} "
              f"(current: {self.args.current_multiplier})")
        return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--marker-id', type=int, default=3)
    parser.add_argument('--marker-x', type=float, default=-7.5)
    parser.add_argument('--marker-y', type=float, default=-7.5)
    parser.add_argument('--marker-z', type=float, default=1.2,
                        help='marker height above GROUND (not the map frame)')
    parser.add_argument('--current-multiplier', type=float, default=1.096)
    args = parser.parse_args()

    rclpy.init()
    node = Calibrate(args)
    code = node.run()
    rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
