/**
 * @file fusion_node_main.cpp
 * @brief Entry point for the OLIVE fusion node
 */

#include <rclcpp/rclcpp.hpp>

#include "olive/fusion/fusion_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<olive::FusionNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
