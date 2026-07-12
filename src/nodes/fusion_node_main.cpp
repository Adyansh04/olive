/**
 * @file fusion_node_main.cpp
 * @brief Entry point for the OLIVE fusion node
 */

#include <pmmintrin.h>
#include <xmmintrin.h>

#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "olive/fusion/fusion_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Flush denormals to zero on the spin thread: near-zero residuals and
    // covariance terms otherwise hit the slow denormal path. Unlike
    // -ffast-math this keeps NaN/Inf semantics (degeneracy guards) intact.
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    auto node = std::make_shared<olive::FusionNode>();

    // Event-driven executor: no per-iteration wait-set rebuild/poll. Still
    // single-threaded, which the scan pipeline's reused buffers rely on.
    rclcpp::experimental::executors::EventsExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
