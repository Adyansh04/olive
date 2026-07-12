#include "olive/vo/visual_odometry_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cv_bridge/cv_bridge.hpp>
#include <iterator>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace olive
{

VisualOdometryNode::VisualOdometryNode(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("vo_node", options)
{
    declare_parameter("autostart", true);
    declare_parameter("image_topic", "/camera/image_raw");
    declare_parameter("camera_info_topic", "/camera/camera_info");
    declare_parameter("wheel_odom_topic", "/odom");
    declare_parameter("odom_topic", "/olive/visual_odom");
    declare_parameter("odom_frame", "vo_odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("max_features", 300);
    declare_parameter("min_tracked_features", 60);
    declare_parameter("min_parallax_px", 12.0);
    declare_parameter("ransac_threshold_px", 1.0);
    declare_parameter("min_wheel_motion_m", 0.03);
    declare_parameter("max_keyframe_age_s", 2.0);
    declare_parameter("debug", false);
    declare_parameter("publish_debug_image", false);

    if (get_parameter("autostart").as_bool())
    {
        autostart_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
            autostart_timer_->cancel();
            configure();
            activate();
        });
    }
}

VisualOdometryNode::CallbackReturn
    VisualOdometryNode::on_configure(const rclcpp_lifecycle::State& /*state*/)
{
    image_topic_         = get_parameter("image_topic").as_string();
    camera_info_topic_   = get_parameter("camera_info_topic").as_string();
    wheel_odom_topic_    = get_parameter("wheel_odom_topic").as_string();
    odom_topic_          = get_parameter("odom_topic").as_string();
    odom_frame_          = get_parameter("odom_frame").as_string();
    base_frame_          = get_parameter("base_frame").as_string();
    max_features_        = static_cast<int>(get_parameter("max_features").as_int());
    min_tracked_         = static_cast<int>(get_parameter("min_tracked_features").as_int());
    min_parallax_px_     = get_parameter("min_parallax_px").as_double();
    ransac_threshold_    = get_parameter("ransac_threshold_px").as_double();
    min_wheel_motion_    = get_parameter("min_wheel_motion_m").as_double();
    max_keyframe_age_    = get_parameter("max_keyframe_age_s").as_double();
    debug_               = get_parameter("debug").as_bool();
    publish_debug_image_ = get_parameter("publish_debug_image").as_bool();

    pose_x_ = pose_y_ = pose_yaw_ = 0.0;
    keyframe_stamp_               = -1.0;
    have_intrinsics_              = false;

    // Sparse 640x480 VO gains nothing from OpenCV's thread pool, and its TBB
    // workers busy-spin between frames, wasting roughly a full core (see
    // benchmark/RESULTS.md). Single-thread the ops (goodFeaturesToTrack /
    // PyrLK are per-point independent, so the output is unchanged).
    cv::setNumThreads(1);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
    odom_msg_.header.frame_id = odom_frame_;
    odom_msg_.child_frame_id  = base_frame_;

    if (publish_debug_image_)
        debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/olive/debug/vo_image",
            rclcpp::SensorDataQoS().keep_last(2));

    RCLCPP_INFO(get_logger(), "Configured (%s -> %s)", image_topic_.c_str(), odom_topic_.c_str());
    return CallbackReturn::SUCCESS;
}

VisualOdometryNode::CallbackReturn
    VisualOdometryNode::on_activate(const rclcpp_lifecycle::State& state)
{
    LifecycleNode::on_activate(state);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_,
        rclcpp::SensorDataQoS().keep_last(2),
        [this](sensor_msgs::msg::Image::SharedPtr msg) { imageCallback(msg); });
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_,
        rclcpp::QoS(5),
        [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(msg); });
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        wheel_odom_topic_,
        rclcpp::SensorDataQoS().keep_last(50),
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { wheelOdomCallback(msg); });
    return CallbackReturn::SUCCESS;
}

VisualOdometryNode::CallbackReturn
    VisualOdometryNode::on_deactivate(const rclcpp_lifecycle::State& state)
{
    image_sub_.reset();
    camera_info_sub_.reset();
    wheel_sub_.reset();
    LifecycleNode::on_deactivate(state);
    return CallbackReturn::SUCCESS;
}

VisualOdometryNode::CallbackReturn
    VisualOdometryNode::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
{
    image_sub_.reset();
    camera_info_sub_.reset();
    wheel_sub_.reset();
    odom_pub_.reset();
    debug_image_pub_.reset();
    return CallbackReturn::SUCCESS;
}

void VisualOdometryNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr& msg)
{
    focal_           = msg->k[0];
    principal_point_ = { msg->k[2], msg->k[5] };
    have_intrinsics_ = true;
    camera_info_sub_.reset();  // intrinsics are static in this system
}

void VisualOdometryNode::wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr& msg)
{
    const double stamp =
        static_cast<double>(msg->header.stamp.sec) + 1e-9 * msg->header.stamp.nanosec;
    const std::lock_guard<std::mutex> lock(wheel_mutex_);
    wheel_samples_.push_back({ stamp, msg->pose.pose.position.x, msg->pose.pose.position.y });
    while (!wheel_samples_.empty() && wheel_samples_.front()[0] < stamp - 30.0)
        wheel_samples_.pop_front();
}

std::optional<double> VisualOdometryNode::wheelDistance(double t0, double t1) const
{
    const std::lock_guard<std::mutex> lock(wheel_mutex_);
    if (wheel_samples_.size() < 2)
        return std::nullopt;

    auto nearest = [this](double t) -> std::optional<std::array<double, 3>> {
        const auto it = std::min_element(
            wheel_samples_.begin(),
            wheel_samples_.end(),
            [t](const auto& a, const auto& b) { return std::abs(a[0] - t) < std::abs(b[0] - t); });
        if (std::abs((*it)[0] - t) > 0.15)
            return std::nullopt;
        return *it;
    };

    const auto a = nearest(t0);
    const auto b = nearest(t1);
    if (!a || !b)
        return std::nullopt;
    return std::hypot((*b)[1] - (*a)[1], (*b)[2] - (*a)[2]);
}

void VisualOdometryNode::adoptKeyframe(const cv::Mat& gray, double stamp)
{
    keyframe_features_.clear();
    cv::goodFeaturesToTrack(gray, keyframe_features_, max_features_, 0.01, 8.0);
    keyframe_gray_  = gray.clone();
    keyframe_stamp_ = stamp;
}

void VisualOdometryNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr& msg)
{
    if (!have_intrinsics_)
        return;

    const double stamp =
        static_cast<double>(msg->header.stamp.sec) + 1e-9 * msg->header.stamp.nanosec;

    const auto cv_ptr = cv_bridge::toCvShare(msg, "rgb8");
    cv::Mat    gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_RGB2GRAY);

    // Only pay the draw/encode cost when the overlay is enabled and something
    // (RViz, a recorder) is actually subscribed.
    const bool draw =
        publish_debug_image_ && debug_image_pub_ && debug_image_pub_->get_subscription_count() > 0;
    const auto emit = [&](const std::vector<cv::Point2f>& from,
                          const std::vector<cv::Point2f>& to,
                          const cv::Mat&                  inliers,
                          const std::string&              status) {
        if (draw)
            publishDebugImage(cv_ptr->image, from, to, inliers, status, msg->header);
    };

    // Bound the keyframe age: a stall (in-place turn, stationary pause) can leave
    // a keyframe lingering until it is older than the wheel buffer, after which
    // wheelDistance() can never resolve scale and VO wedges forever. Force a fresh
    // keyframe well within the wheel history so it always recovers.
    if (keyframe_stamp_ < 0.0 || std::ssize(keyframe_features_) < min_tracked_ ||
        (stamp - keyframe_stamp_) > max_keyframe_age_)
    {
        adoptKeyframe(gray, stamp);
        emit({}, keyframe_features_, cv::Mat(), "KEYFRAME adopted");
        return;
    }

    // Track keyframe features into the current image.
    std::vector<cv::Point2f> tracked;
    std::vector<uchar>       status;
    std::vector<float>       err;
    cv::calcOpticalFlowPyrLK(keyframe_gray_, gray, keyframe_features_, tracked, status, err);

    std::vector<cv::Point2f> p0;
    std::vector<cv::Point2f> p1;
    double                   parallax = 0.0;
    for (size_t i = 0; i < status.size(); ++i)
    {
        if (status[i] == 0)
            continue;
        p0.push_back(keyframe_features_[i]);
        p1.push_back(tracked[i]);
        parallax += cv::norm(tracked[i] - keyframe_features_[i]);
    }

    if (std::ssize(p0) < min_tracked_)
    {
        if (debug_)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "VO gate: tracked %zu < min %d -> re-adopt keyframe "
                "(kf had %zu features)",
                p0.size(),
                min_tracked_,
                keyframe_features_.size());
        }
        emit(p0, p1, cv::Mat(), "re-adopt: lost tracking");
        adoptKeyframe(gray, stamp);
        return;
    }
    parallax /= static_cast<double>(p0.size());
    if (parallax < min_parallax_px_)
    {
        if (debug_)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "VO gate: parallax %.1f px < min %.1f (tracked %zu) -> wait",
                parallax,
                min_parallax_px_,
                p0.size());
        }
        emit(p0, p1, cv::Mat(), "waiting: building baseline");
        return;  // not enough baseline yet
    }

    const auto wheel_motion = wheelDistance(keyframe_stamp_, stamp);
    if (!wheel_motion)
    {
        if (debug_)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "VO gate: no wheel scale for [%.2f, %.2f] -> wait",
                keyframe_stamp_,
                stamp);
        }
        emit(p0, p1, cv::Mat(), "waiting: no wheel scale");
        return;  // no scale reference yet; keep accumulating baseline
    }
    if (debug_)
    {
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "VO ok: tracked %zu  parallax %.1f px  wheel %.3f m",
            p0.size(),
            parallax,
            *wheel_motion);
    }

    if (*wheel_motion < min_wheel_motion_)
    {
        // Large image flow with no wheel translation = in-place rotation:
        // the essential-matrix translation is meaningless, recover yaw only.
        // Small flow with sub-gate wheel motion is a slow forward crawl —
        // keep the keyframe and wait for more baseline instead.
        if (parallax < 4.0 * min_parallax_px_)
        {
            emit(p0, p1, cv::Mat(), "waiting: slow crawl");
            return;
        }

        cv::Mat       inliers;
        const cv::Mat essential = cv::findEssentialMat(
            p0,
            p1,
            focal_,
            principal_point_,
            cv::RANSAC,
            0.999,
            ransac_threshold_,
            inliers);
        if (essential.empty())
            return;
        cv::Mat rotation;
        cv::Mat translation;
        cv::recoverPose(essential, p0, p1, rotation, translation, focal_, principal_point_, inliers);
        // Optical frame: yaw of the body = rotation about the camera y axis.
        pose_yaw_ += std::atan2(rotation.at<double>(0, 2), rotation.at<double>(2, 2));
        emit(p0, p1, inliers, "PUBLISH: rotation-only");
        adoptKeyframe(gray, stamp);
        publishOdometry(stamp);
        return;
    }

    cv::Mat       inliers;
    const cv::Mat essential = cv::findEssentialMat(
        p0,
        p1,
        focal_,
        principal_point_,
        cv::RANSAC,
        0.999,
        ransac_threshold_,
        inliers);
    if (essential.empty())
        return;

    cv::Mat   rotation;
    cv::Mat   translation;
    const int inlier_count =
        cv::recoverPose(essential, p0, p1, rotation, translation, focal_, principal_point_, inliers);
    if (inlier_count < min_tracked_ / 2)
    {
        emit(p0, p1, inliers, "re-adopt: degenerate pose");
        adoptKeyframe(gray, stamp);
        return;
    }

    // Planar reduction, optical convention (x right, y down, z forward):
    // body yaw is the rotation about the camera y axis; body-frame forward /
    // left displacement come from the z / -x components of the translation
    // direction, scaled by the wheel-measured distance.
    const double delta_yaw = std::atan2(rotation.at<double>(0, 2), rotation.at<double>(2, 2));
    const double tx        = translation.at<double>(0);
    const double tz        = translation.at<double>(2);
    const double norm_xz   = std::hypot(tx, tz);
    if (norm_xz < 1e-6)
        return;

    const double forward = *wheel_motion * (tz / norm_xz);
    const double left    = *wheel_motion * (-tx / norm_xz);

    // recoverPose yields the second-camera-from-first transform; the camera
    // (= body) motion in the first frame is the inverse translation.
    pose_x_ += std::cos(pose_yaw_) * forward - std::sin(pose_yaw_) * left;
    pose_y_ += std::sin(pose_yaw_) * forward + std::cos(pose_yaw_) * left;
    pose_yaw_ += delta_yaw;

    emit(p0, p1, inliers, "PUBLISH: translation");
    adoptKeyframe(gray, stamp);
    publishOdometry(stamp);
}

void VisualOdometryNode::publishOdometry(double stamp)
{
    if (!odom_pub_->is_activated())
        return;

    odom_msg_.header.stamp            = rclcpp::Time(static_cast<int64_t>(stamp * 1e9));
    odom_msg_.pose.pose.position.x    = pose_x_;
    odom_msg_.pose.pose.position.y    = pose_y_;
    odom_msg_.pose.pose.position.z    = 0.0;
    odom_msg_.pose.pose.orientation.w = std::cos(pose_yaw_ / 2.0);
    odom_msg_.pose.pose.orientation.z = std::sin(pose_yaw_ / 2.0);
    odom_pub_->publish(odom_msg_);
}

void VisualOdometryNode::publishDebugImage(
    const cv::Mat&                  rgb,
    const std::vector<cv::Point2f>& from,
    const std::vector<cv::Point2f>& to,
    const cv::Mat&                  inliers,
    const std::string&              status,
    const std_msgs::msg::Header&    header)
{
    if (!debug_image_pub_ || !debug_image_pub_->is_activated())
        return;

    cv::Mat canvas = rgb.clone();  // rgb8; scalars below are in RGB order to match

    const cv::Scalar green(40, 220, 40);    // inlier / accepted correspondence
    const cv::Scalar red(235, 70, 50);      // outlier / rejected / lost track
    const cv::Scalar yellow(250, 210, 40);  // healthy flow, still building baseline
    const cv::Scalar cyan(40, 210, 240);    // freshly detected keyframe corner

    const size_t n           = std::min(from.size(), to.size());
    const bool   have_inlier = static_cast<size_t>(inliers.rows) == to.size() && inliers.cols >= 1;

    if (from.empty())
    {
        // Keyframe adoption: nothing to track from yet — show the new corners.
        for (const auto& pt : to)
            cv::circle(canvas, pt, 3, cyan, 1, cv::LINE_AA);
    }
    else
    {
        // Optical flow: tail = keyframe corner, head = tracked position.
        for (size_t i = 0; i < n; ++i)
        {
            const bool inlier    = have_inlier ? inliers.at<uchar>(static_cast<int>(i)) != 0 : true;
            const cv::Scalar col = have_inlier ? (inlier ? green : red) : yellow;
            cv::line(canvas, from[i], to[i], col, 1, cv::LINE_AA);
            cv::circle(canvas, to[i], 3, col, -1, cv::LINE_AA);
        }
    }

    // HUD: feature count and mean flow magnitude (px).
    double flow = 0.0;
    for (size_t i = 0; i < n; ++i)
        flow += cv::norm(to[i] - from[i]);
    if (n > 0)
        flow /= static_cast<double>(n);

    char hud[96];
    if (from.empty())
        std::snprintf(hud, sizeof(hud), "VO  feat:%zu", to.size());
    else
        std::snprintf(hud, sizeof(hud), "VO  feat:%zu  flow:%.1fpx", to.size(), flow);

    cv::Scalar status_col = yellow;
    if (status.rfind("PUBLISH", 0) == 0)
        status_col = green;
    else if (status.rfind("KEYFRAME", 0) == 0)
        status_col = cyan;
    else if (status.rfind("re-adopt", 0) == 0)
        status_col = red;

    // Draw each line twice (dark halo, then colour) so it reads on any scene.
    const cv::Scalar shadow(15, 15, 15);
    cv::putText(canvas, hud, { 10, 24 }, cv::FONT_HERSHEY_SIMPLEX, 0.6, shadow, 3, cv::LINE_AA);
    cv::putText(
        canvas,
        hud,
        { 10, 24 },
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        { 235, 235, 235 },
        1,
        cv::LINE_AA);
    cv::putText(canvas, status, { 10, 50 }, cv::FONT_HERSHEY_SIMPLEX, 0.6, shadow, 3, cv::LINE_AA);
    cv::putText(canvas, status, { 10, 50 }, cv::FONT_HERSHEY_SIMPLEX, 0.6, status_col, 1, cv::LINE_AA);

    debug_image_pub_->publish(*cv_bridge::CvImage(header, "rgb8", canvas).toImageMsg());
}

}  // namespace olive
