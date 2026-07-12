/**
 * @file visual_odometry_node.hpp
 * @brief Planar monocular visual odometry front-end
 *
 * KLT feature tracking between camera keyframes; the essential matrix gives
 * the inter-keyframe rotation and a unit-scale translation direction, and the
 * metric scale comes from wheel odometry over the same interval (monocular
 * scale is unobservable on a planar, near-constant-velocity robot). Motion is
 * reduced to the ground plane (x, y, yaw) — full 6-DoF monocular VO is not
 * trustworthy on this platform and the fusion backend pins z/roll/pitch
 * anyway. Publishes an integrated odometry stream the fusion core consumes as
 * robust between factors (default-disabled modality).
 */

#ifndef OLIVE_VO_VISUAL_ODOMETRY_NODE_HPP_
#define OLIVE_VO_VISUAL_ODOMETRY_NODE_HPP_

#include <deque>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/core.hpp>
#include <optional>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>

namespace olive
{

class VisualOdometryNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit VisualOdometryNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr& msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr& msg);
    void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr& msg);

    /// Wheel-measured planar displacement between two times, if covered
    std::optional<double> wheelDistance(double t0, double t1) const;

    void adoptKeyframe(const cv::Mat& gray, double stamp);
    void publishOdometry(double stamp);

    /// Draw the tracked features / optical flow / gate status onto the camera
    /// frame and publish it on the debug image topic (RViz Image display).
    /// @param rgb    current frame (rgb8, drawn on a copy)
    /// @param from   keyframe feature positions (flow tails); empty on adopt
    /// @param to     current tracked positions (flow heads / dots)
    /// @param inliers per-correspondence essential-matrix inlier mask, or empty
    /// @param status short HUD line explaining what VO is doing this frame
    /// @param header source image header (reused for stamp + camera frame)
    void publishDebugImage(
        const cv::Mat&                  rgb,
        const std::vector<cv::Point2f>& from,
        const std::vector<cv::Point2f>& to,
        const cv::Mat&                  inliers,
        const std::string&              status,
        const std_msgs::msg::Header&    header);

    // Topics / params
    std::string image_topic_;
    std::string camera_info_topic_;
    std::string wheel_odom_topic_;
    std::string odom_topic_;
    std::string odom_frame_;
    std::string base_frame_;
    int         max_features_     = 300;
    int         min_tracked_      = 60;
    double      min_parallax_px_  = 12.0;
    double      ransac_threshold_ = 1.0;
    double      min_wheel_motion_ = 0.03;
    double      max_keyframe_age_ = 2.0;  ///< re-adopt keyframe past this age (< wheel history)
    bool        debug_        = false;  ///< log per-frame gate values (why VO is/isn't publishing)
    bool publish_debug_image_ = false;  ///< draw features/flow overlay on /olive/debug/vo_image

    // Camera intrinsics (from camera_info)
    bool        have_intrinsics_ = false;
    double      focal_           = 0.0;
    cv::Point2d principal_point_{ 0.0, 0.0 };

    // Keyframe state
    cv::Mat                  keyframe_gray_;
    std::vector<cv::Point2f> keyframe_features_;
    double                   keyframe_stamp_ = -1.0;

    // Integrated planar pose (x, y, yaw) in the VO odom frame
    double pose_x_ = 0.0, pose_y_ = 0.0, pose_yaw_ = 0.0;

    // Wheel odometry buffer for scale: (stamp, x, y)
    mutable std::mutex                wheel_mutex_;
    std::deque<std::array<double, 3>> wheel_samples_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr                 image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr            camera_info_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                 wheel_sub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    nav_msgs::msg::Odometry                                                  odom_msg_;
    rclcpp::TimerBase::SharedPtr                                             autostart_timer_;
};

}  // namespace olive

#endif  // OLIVE_VO_VISUAL_ODOMETRY_NODE_HPP_
