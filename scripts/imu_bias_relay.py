#!/usr/bin/env python3
"""Gyro-bias fault injector: republish an IMU topic with a constant bias added.

Simulates a real MEMS gyro's turn-on bias so the fusion stack's stationary
bias estimation can be exercised in simulation:

    ros2 run olive imu_bias_relay.py --bias-z 0.02
    # point the fusion at the biased stream:
    #   fusion.yaml: imu_topic: /imu/data_biased

Usage:
    ros2 run olive imu_bias_relay.py \
        [--in-topic /imu/data] [--out-topic /imu/data_biased] \
        [--bias-x 0] [--bias-y 0] [--bias-z 0.02]
"""
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


class ImuBiasRelay(Node):
    def __init__(self, args):
        super().__init__('imu_bias_relay')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.bias = (args.bias_x, args.bias_y, args.bias_z)
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(Imu, args.out_topic, qos)
        self.create_subscription(Imu, args.in_topic, self.callback, qos)
        self.get_logger().info(
            f"relaying {args.in_topic} -> {args.out_topic} with gyro bias {self.bias} rad/s")

    def callback(self, msg):
        msg.angular_velocity.x += self.bias[0]
        msg.angular_velocity.y += self.bias[1]
        msg.angular_velocity.z += self.bias[2]
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--in-topic', default='/imu/data')
    parser.add_argument('--out-topic', default='/imu/data_biased')
    parser.add_argument('--bias-x', type=float, default=0.0)
    parser.add_argument('--bias-y', type=float, default=0.0)
    parser.add_argument('--bias-z', type=float, default=0.02)
    args = parser.parse_args()

    rclpy.init()
    node = ImuBiasRelay(args)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
