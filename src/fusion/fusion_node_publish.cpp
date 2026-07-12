/**
 * @file fusion_node_publish.cpp
 * @brief FusionNode outputs
 *
 * Fused map-frame odometry + map->odom TF, the smooth odom-frame stream
 * (odomTick extension + LiDAR-dropout coasting), and /diagnostics.
 */

#include <format>
#include <string>

#include "olive/common/gtsam_conversions.hpp"
#include "olive/fusion/frontend/scan_matcher.hpp"
#include "olive/fusion/fusion_node.hpp"
#include "olive/fusion/graph/keyframe_map.hpp"
#include "olive/fusion/graph/pose_graph.hpp"

namespace olive
{

void FusionNode::publishDiagnostics()
{
    if (!diagnostics_pub_ || !diagnostics_pub_->is_activated())
        return;

    const double now = static_cast<double>(get_clock()->now().nanoseconds()) * 1e-9;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = get_clock()->now();

    for (const auto& status : health_monitor_.evaluate(now))
    {
        diagnostic_msgs::msg::DiagnosticStatus diag;
        diag.name        = "olive/" + status.name;
        diag.hardware_id = status.name;

        using enum SensorHealth;
        switch (status.health)
        {
            case FAILED:
                diag.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                diag.message = status.age < 0.0 ? "no data" : "timed out";
                break;
            case POOR:
            case DEGRADED:
                diag.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                diag.message = status.detail.empty() ? "degraded" : status.detail;
                break;
            default:
                diag.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
                diag.message = "ok";
        }

        diagnostic_msgs::msg::KeyValue age;
        age.key   = "age_s";
        age.value = std::to_string(status.age);
        diag.values.push_back(age);

        if (status.name == "lidar" && scan_matcher_)
        {
            diagnostic_msgs::msg::KeyValue eig;
            eig.key            = "constraint_eigenvalues";
            const auto& values = scan_matcher_->constraintEigenvalues();
            eig.value          = std::format("{} {} {}", values(0), values(1), values(2));
            diag.values.push_back(eig);
        }
        array.status.push_back(diag);
    }
    diagnostics_pub_->publish(array);
}

void FusionNode::odomTick()
{
    // Before the first scan: wheel passthrough keeps the odom TF and local
    // odometry alive during IMU init (Nav2 can look up odom->base right away).
    if (last_scan_stamp_ < 0.0)
    {
        if (publish_odom_tf_ && wheel_origin_ && wheel_buffer_.hasData())
        {
            const double wheel_now = wheel_buffer_.latestStamp();
            const auto   latest    = wheel_buffer_.poseAt(wheel_now);
            if (latest)
                publishSmoothOdometry(wheel_origin_->between(*latest), wheel_now);
        }
        return;
    }

    if (!wheel_buffer_.hasData())
        return;
    const double wheel_now = wheel_buffer_.latestStamp();
    if (wheel_now <= last_scan_stamp_)
        return;

    const auto wheel_relative = wheel_buffer_.relativePose(last_scan_stamp_, wheel_now);
    if (!wheel_relative)
        return;

    // Transient extension of the smooth pose between scans: wheel translation
    // with gyro rotation (the predictPose recipe). State is not mutated — the
    // next scan increment supersedes this extension.
    gtsam::Pose3 extension = *wheel_relative;
    if (imu_buffer_.hasData())
    {
        extension = gtsam::Pose3(
            gtsam::Rot3(imu_buffer_.relativeRotation(last_scan_stamp_, wheel_now)),
            extension.translation());
    }
    const gtsam::Pose3 smooth_now = smooth_pose_ * extension;
    if (publish_odom_tf_)
        publishSmoothOdometry(smooth_now, wheel_now);

    // LiDAR dropout: keep the map-frame output alive on wheel dead-reckoning
    // while the LiDAR is silent (tier-1); the graph itself is left untouched.
    if (!coast_on_dropout_ || wheel_now - last_scan_stamp_ < lidar_dropout_timeout_)
        return;

    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "No LiDAR for %.1f s - coasting on wheel odometry",
        wheel_now - last_scan_stamp_);
    // In odom-owning mode the coast pose follows the TF chain exactly (the
    // correction stays frozen: no new global information during an outage).
    const gtsam::Pose3 coast_pose = publish_odom_tf_ && map_odom_valid_ ?
                                        map_from_odom_ * smooth_now * child_from_base_ :
                                        last_scan_pose_ * (*wheel_relative);
    publishOdometry(coast_pose, wheel_now);

    // Tier 2 (optional): wheel-paced keyframes so marker anchors can still
    // land during the outage.
    if (dropout_keyframes_ && !keyframe_map_->empty() &&
        keyframe_map_->shouldAddKeyframe(coast_pose))
    {
        const double previous_stamp    = keyframe_map_->back().stamp;
        const auto   keyframe_relative = wheel_buffer_.relativePose(previous_stamp, wheel_now);
        if (!keyframe_relative)
            return;

        pose_graph_->addKeyframe(coast_pose, *keyframe_relative, wheel_between_sigmas_);
        if (use_planar_prior_)
            pose_graph_->addPlanarPrior(planar_prior_sigmas_);
        if (pose_graph_->optimize() == PoseGraph::OptimizeResult::FAILED)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Coast keyframe rejected by the graph");
            return;
        }
        keyframe_map_
            ->add(pose_graph_->latestPose(), nullptr, nullptr, wheel_now, /*low_quality=*/true);
        // Keep the smooth stream in step with the advanced scan state, then
        // re-derive the correction from the optimized pose.
        smooth_pose_     = smooth_pose_ * extension;
        last_scan_pose_  = pose_graph_->latestPose();
        last_scan_stamp_ = wheel_now;
        updateMapOdomCorrection();
    }
}

void FusionNode::publishOdometry(const gtsam::Pose3& pose, double stamp)
{
    if (!odom_pub_->is_activated())
        return;

    odom_msg_.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9));
    odom_msg_.pose.pose    = gtsam_conversions::toRosPose(pose);
    odom_pub_->publish(odom_msg_);

    // When this node owns odom->base, both TF edges are sent together in
    // publishSmoothOdometry so consumers always see a consistent snapshot.
    if (!publish_map_tf_ || publish_odom_tf_)
        return;

    // REP-105 split (wheel-owned odom frame): broadcast only the map->odom
    // correction. The continuous odom->base transform belongs to the wheel
    // odometry source, so a global update here can never teleport the robot
    // in the odom frame.
    const auto wheel_pose = wheel_buffer_.poseAt(stamp);
    if (!wheel_pose)
        return;

    const gtsam::Pose3 map_from_odom = pose * wheel_pose->inverse();

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp            = odom_msg_.header.stamp;
    tf.header.frame_id         = map_frame_;
    tf.child_frame_id          = odom_frame_;
    const auto pose_msg        = gtsam_conversions::toRosPose(map_from_odom);
    tf.transform.translation.x = pose_msg.position.x;
    tf.transform.translation.y = pose_msg.position.y;
    tf.transform.translation.z = pose_msg.position.z;
    tf.transform.rotation      = pose_msg.orientation;
    tf_broadcaster_->sendTransform(tf);
}

void FusionNode::updateMapOdomCorrection()
{
    if (!publish_odom_tf_)
        return;
    // map->odom = (map->base) o (odom->base)^-1 at the scan stamp, where
    // odom->base = smooth (odom->footprint) o (footprint->base). Anchor and
    // loop jumps land here and never in the smooth stream.
    map_from_odom_  = last_scan_pose_ * (smooth_pose_ * child_from_base_).inverse();
    map_odom_valid_ = true;
}

void FusionNode::publishSmoothOdometry(const gtsam::Pose3& odom_from_child, double stamp)
{
    if (!smooth_odom_pub_ || !smooth_odom_pub_->is_activated())
        return;

    smooth_odom_msg_.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9));
    smooth_odom_msg_.pose.pose    = gtsam_conversions::toRosPose(odom_from_child);
    smooth_odom_msg_.twist.twist  = last_wheel_twist_;
    if (imu_buffer_.hasData())
        smooth_odom_msg_.twist.twist.angular.z = imu_buffer_.rateNear(stamp).z();
    smooth_odom_pub_->publish(smooth_odom_msg_);

    if (!publish_odom_tf_)
        return;

    // Both TF edges in one broadcast so consumers always see a consistent
    // snapshot: odom->base_footprint (smooth, jump-free) and map->odom (the
    // correction — anchors and loop closures jump HERE, REP-105).
    tf_batch_.clear();

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp            = smooth_odom_msg_.header.stamp;
    tf.header.frame_id         = odom_frame_;
    tf.child_frame_id          = odom_child_frame_;
    auto pose_msg              = gtsam_conversions::toRosPose(odom_from_child);
    tf.transform.translation.x = pose_msg.position.x;
    tf.transform.translation.y = pose_msg.position.y;
    tf.transform.translation.z = pose_msg.position.z;
    tf.transform.rotation      = pose_msg.orientation;
    tf_batch_.push_back(tf);

    if (publish_map_tf_ && map_odom_valid_)
    {
        tf.header.frame_id         = map_frame_;
        tf.child_frame_id          = odom_frame_;
        pose_msg                   = gtsam_conversions::toRosPose(map_from_odom_);
        tf.transform.translation.x = pose_msg.position.x;
        tf.transform.translation.y = pose_msg.position.y;
        tf.transform.translation.z = pose_msg.position.z;
        tf.transform.rotation      = pose_msg.orientation;
        tf_batch_.push_back(tf);
    }
    tf_broadcaster_->sendTransform(tf_batch_);
}

}  // namespace olive
