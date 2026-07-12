#!/usr/bin/env python3
"""LiDAR outage fault injector: relay a PointCloud2 topic with a mute window.

Republishes the LiDAR stream on another topic and drops all messages during
[mute-after, mute-after + mute-duration] (relative to the first message), so
sensor-dropout coasting and recovery can be exercised in simulation:

    ros2 run olive lidar_gate_relay.py --mute-after 20 --mute-duration 15
    # point the fusion at the gated stream:
    #   fusion.yaml: points_topic: /lidar/points_gated
"""
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class LidarGateRelay(Node):
    def __init__(self, args):
        super().__init__('lidar_gate_relay')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.mute_after = args.mute_after
        self.mute_until = args.mute_after + args.mute_duration
        self.first_stamp = None
        self.muted = False
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(PointCloud2, args.out_topic, qos)
        self.create_subscription(PointCloud2, args.in_topic, self.callback, qos)
        self.get_logger().info(
            f"gating {args.in_topic} -> {args.out_topic}: mute at "
            f"t+{self.mute_after:.0f}s for {args.mute_duration:.0f}s")

    def callback(self, msg):
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if self.first_stamp is None:
            self.first_stamp = stamp
        elapsed = stamp - self.first_stamp
        if self.mute_after <= elapsed < self.mute_until:
            if not self.muted:
                self.get_logger().warn("LiDAR outage START")
                self.muted = True
            return
        if self.muted:
            self.get_logger().warn("LiDAR outage END")
            self.muted = False
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--in-topic', default='/lidar/points')
    parser.add_argument('--out-topic', default='/lidar/points_gated')
    parser.add_argument('--mute-after', type=float, default=20.0)
    parser.add_argument('--mute-duration', type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = LidarGateRelay(args)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
