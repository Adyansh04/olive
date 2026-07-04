/**
 * @file lidar_odom_node.cpp
 * @brief Main entry point for LiDAR odometry node
 */

#include <rclcpp/rclcpp.hpp>

#include "olive/lidar_odom/lidar_odometry.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<olive::LidarOdometry>();

    // TODO: Lifecycle transitions should be managed externally via lifecycle manager
    // For standalone testing, transitions are done here
    // In final implementation, use: ros2 lifecycle set /lidar_odom_node configure
    node->configure();
    node->activate();

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}
