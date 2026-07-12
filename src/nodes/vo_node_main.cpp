/**
 * @file vo_node_main.cpp
 * @brief Entry point for the OLIVE visual odometry node
 */

#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "olive/vo/visual_odometry_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<olive::VisualOdometryNode>();

    // Event-driven executor: no per-iteration wait-set rebuild/poll.
    rclcpp::experimental::executors::EventsExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
