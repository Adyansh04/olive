// FusionNode — the scan hot path: pose prediction, preprocess/deskew/match,
// keyframe decision, factor insertion (LiDAR/wheel/VO/IMU/markers), iSAM2
// optimization, correction refresh, and loop closure.
#include <gtsam/navigation/CombinedImuFactor.h>

#include <algorithm>
#include <chrono>
#include <cmath>

#include "olive/fusion/frontend/feature_extractor.hpp"
#include "olive/fusion/frontend/scan_matcher.hpp"
#include "olive/fusion/frontend/scan_preprocessor.hpp"
#include "olive/fusion/fusion_node.hpp"
#include "olive/fusion/graph/keyframe_map.hpp"
#include "olive/fusion/graph/loop_detector.hpp"
#include "olive/fusion/graph/pose_graph.hpp"
#include "olive/fusion/inputs/marker_gate.hpp"

namespace olive
{

void FusionNode::bootstrapFirstKeyframe(const FeatureClouds& features)
{
    const gtsam::Pose3 origin;
    pose_graph_->addFirstKeyframe(origin);
    if (imu_preintegration_)
    {
        // Stationary start (enforced by the IMU init gate): V(0) = 0 and
        // B(0) seeded with the measured gyro bias.
        pose_graph_->addImuPriors(imu_buffer_.gyroBias(), 0.1, 0.1, 0.01);
    }
    if (pose_graph_->optimize() == PoseGraph::OptimizeResult::FAILED)
    {
        RCLCPP_ERROR(get_logger(), "Bootstrap graph update failed - retrying on the next scan");
        return;
    }

    const Cloud::Ptr edge_copy(new Cloud(*features.edge));
    const Cloud::Ptr planar_copy(new Cloud(*features.planar));
    keyframe_map_->add(origin, edge_copy, planar_copy, features.stamp);

    last_scan_pose_  = origin;
    last_scan_stamp_ = features.stamp;

    // Seed the smooth stream with the motion accumulated since node start so
    // the odom frame stays anchored where the first wheel sample was taken.
    smooth_pose_ = gtsam::Pose3();
    if (wheel_origin_)
    {
        const auto wheel_now = wheel_buffer_.poseAt(features.stamp);
        if (wheel_now)
            smooth_pose_ = wheel_origin_->between(*wheel_now);
    }
    updateMapOdomCorrection();
}

gtsam::Pose3 FusionNode::predictPose(double scan_stamp) const
{
    // After a scan gap (sensor outage), the constant-velocity increment is
    // stale garbage — the wheel-measured motion over the gap replaces it.
    if (last_scan_stamp_ > 0.0 && scan_stamp - last_scan_stamp_ > prediction_gap_fallback_s_)
    {
        const auto wheel_relative = wheel_buffer_.relativePose(last_scan_stamp_, scan_stamp);
        if (wheel_relative)
            return last_scan_pose_ * (*wheel_relative);
    }

    // Translation prediction comes from the wheels when they cover the
    // interval — the platform's own velocity truth. A constant-velocity
    // extrapolation is only the fallback: in weakly-observable scenes
    // (corridors) the matcher cannot correct along the degenerate axis, so
    // an extrapolated guess feeds back into the estimate and pumps a
    // translation runaway. Rotation comes from the gyro either way.
    gtsam::Pose3 increment = last_increment_;
    if (last_scan_stamp_ > 0.0)
    {
        const auto wheel_relative = wheel_buffer_.relativePose(last_scan_stamp_, scan_stamp);
        if (wheel_relative)
            increment = *wheel_relative;
    }
    if (imu_buffer_.hasData() && last_scan_stamp_ > 0.0)
    {
        const Eigen::Quaterniond gyro_rotation =
            imu_buffer_.relativeRotation(last_scan_stamp_, scan_stamp);
        increment = gtsam::Pose3(gtsam::Rot3(gyro_rotation), increment.translation());
    }
    return last_scan_pose_ * increment;
}

void FusionNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
{
    const auto pipeline_start = std::chrono::steady_clock::now();

    // Hold scans until the IMU bias window resolves; a robot without an IMU
    // still starts once the grace period passes.
    if (!imu_init_done_)
    {
        const double stamp =
            static_cast<double>(msg->header.stamp.sec) + 1e-9 * msg->header.stamp.nanosec;
        if (first_scan_stamp_ < 0.0)
            first_scan_stamp_ = stamp;
        if (!imu_buffer_.hasData() && stamp - first_scan_stamp_ > imu_init_max_wait_s_)
        {
            RCLCPP_WARN(
                get_logger(),
                "No IMU data after %.1f s - running without gyro",
                imu_init_max_wait_s_);
            imu_init_done_ = true;
        }
        else
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Waiting for IMU initialization before processing scans");
            return;
        }
    }

    if (!preprocessor_->process(*msg, scan_image_))
        return;

    logSensorLatency("lidar", scan_image_.stamp);
    health_monitor_.beat("lidar", scan_image_.stamp);

    if (deskew_enabled_ && !scan_image_.rel_time.empty() && imu_buffer_.hasData())
    {
        const auto [min_it, max_it] =
            std::minmax_element(scan_image_.rel_time.begin(), scan_image_.rel_time.end());
        const double t_min = *min_it;
        const double t_max = *max_it;
        if (t_max - t_min > 1e-4)  // no-op for clouds without a time field
        {
            deskewScan(
                scan_image_,
                imu_buffer_.sampleRotations(
                    scan_image_.stamp + t_min,
                    scan_image_.stamp + t_max,
                    deskew_time_bins_),
                t_min,
                t_max);
        }
    }

    feature_extractor_->extract(scan_image_, features_);

    if (keyframe_map_->empty())
    {
        bootstrapFirstKeyframe(features_);
        publishOdometry(last_scan_pose_, features_.stamp);
        publishSmoothOdometry(smooth_pose_, features_.stamp);
        return;
    }

    const gtsam::Pose3 guess = predictPose(features_.stamp);

    Cloud::Ptr edge_map;
    Cloud::Ptr planar_map;
    keyframe_map_->buildLocalMap(guess.translation(), features_.stamp, edge_map, planar_map);
    scan_matcher_->setTarget(edge_map, planar_map);
    last_edge_map_   = edge_map;
    last_planar_map_ = planar_map;

    MatcherPose matcher_pose =
        MatcherPose::fromAffine(Eigen::Affine3f(guess.matrix().cast<float>()));
    gtsam::Pose3 scan_pose = guess;
    last_match_ok_         = scan_matcher_->align(features_, matcher_pose);
    last_match_degenerate_ = scan_matcher_->isDegenerate();
    if (last_match_ok_ && last_match_degenerate_)
    {
        health_monitor_.flagQuality("lidar", SensorHealth::DEGRADED, "degenerate geometry");
    }
    else if (!last_match_ok_)
    {
        health_monitor_.flagQuality("lidar", SensorHealth::POOR, "scan match failed");
    }
    if (last_match_ok_)
    {
        if (planar_motion_)
        {
            // Ground robot: the graph estimates x, y, yaw; z / roll / pitch
            // are structurally zero and clamping them blocks slow vertical
            // drift in scenes where the floor is barely visible.
            matcher_pose.z     = 0.0F;
            matcher_pose.roll  = 0.0F;
            matcher_pose.pitch = 0.0F;
        }
        scan_pose = gtsam::Pose3(
            gtsam::Rot3::Ypr(matcher_pose.yaw, matcher_pose.pitch, matcher_pose.roll),
            gtsam::Point3(matcher_pose.x, matcher_pose.y, matcher_pose.z));
    }
    else
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Scan matching failed; coasting on prediction");
    }

    last_increment_ = last_scan_pose_.between(scan_pose);

    // Advance the smooth odom-frame pose by the correction-independent
    // body increment. After a long LiDAR gap the scan increment carries the
    // accumulated scan-vs-wheel correction of the whole outage — that
    // correction belongs in map->odom, so the wheel increment is used there.
    gtsam::Pose3 smooth_increment = last_increment_;
    if (last_scan_stamp_ > 0.0 && features_.stamp - last_scan_stamp_ > prediction_gap_fallback_s_)
    {
        const auto wheel_gap = wheel_buffer_.relativePose(last_scan_stamp_, features_.stamp);
        if (wheel_gap)
            smooth_increment = *wheel_gap;
    }
    smooth_pose_ = smooth_pose_ * smooth_increment;

    last_scan_pose_  = scan_pose;
    last_scan_stamp_ = features_.stamp;

    if (keyframe_map_->shouldAddKeyframe(scan_pose))
    {
        const double       previous_stamp = keyframe_map_->back().stamp;
        const gtsam::Pose3 relative       = keyframe_map_->back().pose.between(scan_pose);

        // A dead-reckoned or degenerate keyframe must not carry the tight
        // scan-match confidence, or it silently corrupts the graph.
        double lidar_scale = !last_match_ok_ ?
                                 match_fail_sigma_scale_ :
                                 (last_match_degenerate_ ? degenerate_sigma_scale_ : 1.0);

        const auto wheel_relative =
            use_wheel_odom_ ? wheel_buffer_.relativePose(previous_stamp, features_.stamp) :
                              std::nullopt;

        // Eigenvalue thresholds are scene-dependent; the wheels are a direct
        // witness. A scan match claiming substantially MORE or LESS motion
        // than the wheels measured loses its authority no matter what the
        // eigenvalues said. Only the distance is compared — direction
        // differences at turn keyframes are yaw effects the wheel factor
        // already handles, and vector comparison false-fires on them.
        bool wheel_disagrees = false;
        if (wheel_relative)
        {
            const double distance_gap =
                std::abs(relative.translation().norm() - wheel_relative->translation().norm());
            if (distance_gap > wheel_lidar_disagree_m_)
            {
                wheel_disagrees = true;
                lidar_scale     = std::max(lidar_scale, degenerate_sigma_scale_);
                health_monitor_.flagQuality(
                    "lidar",
                    SensorHealth::DEGRADED,
                    "disagrees with wheel odometry");
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "Scan match distance disagrees with wheels by %.2f m - "
                    "widening the LiDAR factor",
                    distance_gap);
            }
        }

        FactorSigmas lidar_sigmas = lidar_between_sigmas_;
        for (double& sigma : lidar_sigmas)
            sigma *= lidar_scale;
        pose_graph_->addKeyframe(scan_pose, relative, lidar_sigmas);

        if (use_wheel_odom_)
        {
            if (wheel_relative)
            {
                // Slip enters through rotation: widen the wheel factor with
                // the turning and distance actually covered this interval.
                const double turn = std::abs(
                    Eigen::AngleAxisd(imu_buffer_.relativeRotation(previous_stamp, features_.stamp))
                        .angle());
                const double dist = wheel_relative->translation().norm();

                FactorSigmas wheel_sigmas = wheel_between_sigmas_;
                wheel_sigmas[5] *= 1.0 + wheel_yaw_sigma_per_rad_ * turn;
                wheel_sigmas[0] *=
                    1.0 + wheel_dist_sigma_per_m_ * dist + wheel_yaw_sigma_per_rad_ * turn;
                wheel_sigmas[1] = wheel_sigmas[0];
                pose_graph_->addOdometryFactor(*wheel_relative, wheel_sigmas);
            }
        }
        if (use_vo_)
        {
            // VO increments are wheel-scaled and only trustworthy in-plane:
            // loose z/roll/pitch, robustified against tracking failures.
            const auto vo_relative = vo_buffer_.relativePose(previous_stamp, features_.stamp);
            if (vo_relative)
            {
                pose_graph_->addOdometryFactor(*vo_relative, vo_between_sigmas_, true);
                ++vo_factors_added_;
            }
            else
            {
                ++vo_factors_skipped_;
            }
            // Keyframe rate (~1 Hz), not a sensor hot path -> throttled log is fine.
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "VO between-factors: %zu added, %zu skipped (buffer coverage)",
                vo_factors_added_,
                vo_factors_skipped_);
        }
        if (use_planar_prior_)
            pose_graph_->addPlanarPrior(planar_prior_sigmas_);

        if (imu_preintegration_)
        {
            // Preintegrate the RAW samples over the keyframe interval (kept
            // off the IMU hot path; ~200 samples at keyframe rate). Skip the
            // factor when the buffer doesn't cover the interval (long outage
            // vs. buffer history) — the lidar/wheel betweens still chain and
            // the graph re-seeds V/B at the next covered keyframe.
            // Keyframes are distance-gated, so a stationary pause stretches
            // the interval arbitrarily; preintegrating tens of seconds gives
            // a huge, poorly-linearized factor. Cap it — the chain re-seeds
            // at the next covered keyframe.
            const double interval = features_.stamp - previous_stamp;
            const auto   samples  = imu_buffer_.samplesBetween(previous_stamp, features_.stamp);
            const bool   covered  = samples.size() >= 2 && interval <= imu_preint_max_interval_ &&
                                 samples.back().timestamp - previous_stamp > interval - 0.1;
            if (covered)
            {
                pim_->resetIntegrationAndSetBias(pose_graph_->latestBias());
                double prev_t = previous_stamp;
                for (const ImuData& s : samples)
                {
                    const double dt = s.timestamp - prev_t;
                    if (dt > 0.0)
                        pim_->integrateMeasurement(s.linear_acceleration, s.angular_velocity, dt);
                    prev_t = s.timestamp;
                }
                pose_graph_->addCombinedImuFactor(*pim_, planar_motion_);
            }
        }

        int  anchors             = 0;
        bool surveyed_this_round = false;
        if (use_markers_)
        {
            for (const MarkerObservation& obs :
                 marker_gate_->collectNear(features_.stamp, marker_stamp_window_))
            {
                if (marker_landmark_mode_)
                {
                    // TagSLAM-style: the marker is a landmark variable.
                    // Surveyed ids carry a world prior (anchoring); everything
                    // else is a free landmark whose repeated sightings act as
                    // an odometry constraint.
                    const auto survey =
                        obs.decoded ? known_markers_.find(obs.marker_id) : known_markers_.end();
                    const bool surveyed = survey != known_markers_.end();
                    // Gauge guard: a free landmark initialized before the
                    // first survey anchor encodes the spawn frame and later
                    // fights the anchor snap — hold free landmarks back until
                    // the trajectory is world-anchored.
                    if (!surveyed && !world_anchored_)
                        continue;
                    surveyed_this_round = surveyed_this_round || surveyed;
                    pose_graph_->addMarkerObservation(
                        obs.landmark_key_id,
                        obs.position_in_camera,
                        base_from_camera_,
                        marker_sigma_m_,
                        surveyed ? std::optional<gtsam::Point3>(survey->second) : std::nullopt,
                        marker_survey_sigma_m_);
                }
                else
                {
                    pose_graph_->addMarkerAnchor(
                        obs.position_in_camera,
                        known_markers_.at(obs.marker_id),
                        base_from_camera_,
                        marker_sigma_m_);
                }
                anchor_event_times_[obs.landmark_key_id] = features_.stamp;
                ++anchors;
            }
        }

        const auto   optimize_start = std::chrono::steady_clock::now();
        const auto   result         = pose_graph_->optimize();
        const double optimize_ms    = std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() - optimize_start)
                                       .count();
        if (optimize_ms > optimize_budget_warn_ms_)
        {
            RCLCPP_WARN(
                get_logger(),
                "Graph update took %.1f ms (budget %.0f ms)",
                optimize_ms,
                optimize_budget_warn_ms_);
        }

        if (result == PoseGraph::OptimizeResult::FAILED)
        {
            // The keyframe was rolled back; keep publishing the scan-matched
            // pose and try again at the next keyframe trigger.
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Graph update failed - keyframe discarded, coasting on scan matching");
            updateMapOdomCorrection();
            publishOdometry(last_scan_pose_, features_.stamp);
            publishSmoothOdometry(smooth_pose_, features_.stamp);
            publishScanDebug(features_.stamp);
            return;
        }
        const bool corrected = result == PoseGraph::OptimizeResult::CORRECTED;
        if (surveyed_this_round)
            world_anchored_ = true;  // gauge fixed: free landmarks may enter

        if (imu_preintegration_)
        {
            // NOTE: the graph's bias estimate is deliberately NOT fed back
            // into the buffer (deskew/prediction). That closes a loop through
            // the scan matcher — a transient bias mis-estimate corrupts
            // deskew, which corrupts the match, which reinforces the
            // mis-estimate (observed as non-repeatable yaw excursions). The
            // buffer keeps the stationary-init bias; the graph's online
            // estimate corrects the FACTORS and is published for monitoring.
            const auto bias = pose_graph_->latestBias();

            if (debug_imu_state_ && debug_bias_pub_->is_activated())
            {
                geometry_msgs::msg::AccelStamped bias_msg;
                bias_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(features_.stamp * 1e9));
                bias_msg.header.frame_id = base_frame_;
                bias_msg.accel.linear.x  = bias.accelerometer().x();
                bias_msg.accel.linear.y  = bias.accelerometer().y();
                bias_msg.accel.linear.z  = bias.accelerometer().z();
                bias_msg.accel.angular.x = bias.gyroscope().x();
                bias_msg.accel.angular.y = bias.gyroscope().y();
                bias_msg.accel.angular.z = bias.gyroscope().z();
                debug_bias_pub_->publish(bias_msg);

                geometry_msgs::msg::Vector3Stamped vel_msg;
                vel_msg.header      = bias_msg.header;
                const auto velocity = pose_graph_->latestVelocity();
                vel_msg.vector.x    = velocity.x();
                vel_msg.vector.y    = velocity.y();
                vel_msg.vector.z    = velocity.z();
                debug_velocity_pub_->publish(vel_msg);
            }
        }

        const gtsam::Pose3 optimized = pose_graph_->latestPose();
        const Cloud::Ptr   edge_copy(new Cloud(*features_.edge));
        const Cloud::Ptr   planar_copy(new Cloud(*features_.planar));
        keyframe_map_->add(
            optimized,
            edge_copy,
            planar_copy,
            features_.stamp,
            !last_match_ok_ || last_match_degenerate_ || wheel_disagrees);

        if (corrected)
        {
            refreshAfterCorrection();
            RCLCPP_INFO(
                get_logger(),
                "Global correction applied (%d marker observation%s) - trajectory corrected",
                anchors,
                anchors == 1 ? "" : "s");
        }

        last_scan_pose_ = corrected ? pose_graph_->latestPose() : optimized;
        publishKeyframeDebug(corrected, features_.stamp);

        if (loop_closure_enabled_)
            attemptLoopClosure(features_.stamp);
    }

    // The correction is re-derived at the scan stamp where the fused pose and
    // the smooth pose describe the same instant; any anchor/loop jump from
    // this cycle lands here (map->odom) and never in the smooth stream.
    updateMapOdomCorrection();
    publishOdometry(last_scan_pose_, features_.stamp);
    publishSmoothOdometry(smooth_pose_, features_.stamp);
    publishScanDebug(features_.stamp);

    const double pipeline_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - pipeline_start)
            .count();
    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "scan: %zu pts | edges %zu planars %zu | map %zu/%zu | kf %zu | %s | eig[%.0f %.0f %.0f] "
        "| %.1f ms",
        scan_image_.points->size(),
        features_.edge->size(),
        features_.planar->size(),
        edge_map->size(),
        planar_map->size(),
        keyframe_map_->size(),
        scan_matcher_->isDegenerate() ? "DEGEN" : "ok",
        static_cast<double>(scan_matcher_->constraintEigenvalues()(0)),
        static_cast<double>(scan_matcher_->constraintEigenvalues()(1)),
        static_cast<double>(scan_matcher_->constraintEigenvalues()(2)),
        pipeline_ms);
}

void FusionNode::refreshAfterCorrection()
{
    // A global factor bent the past trajectory: refresh every stored
    // keyframe pose and drop the transformed-cloud cache.
    const auto poses = pose_graph_->allPoses();
    for (size_t i = 0; i < poses.size(); ++i)
        keyframe_map_->updatePose(i, poses[i]);
    keyframe_map_->invalidateCache();
}

void FusionNode::attemptLoopClosure(double stamp)
{
    if (keyframe_map_->size() < 10)
        return;
    if (last_loop_attempt_ >= 0.0 && stamp - last_loop_attempt_ < loop_min_interval_s_)
        return;
    last_loop_attempt_ = stamp;

    const size_t current = keyframe_map_->size() - 1;
    const auto   loop    = loop_detector_->detect(*keyframe_map_, current);
    if (!loop)
        return;

    pose_graph_->addLoopFactor(
        loop->old_index,
        current,
        loop->relative,
        std::max(loop->fitness, loop_sigma_floor_));
    if (pose_graph_->optimize() != PoseGraph::OptimizeResult::CORRECTED)
        return;

    refreshAfterCorrection();
    last_scan_pose_ = pose_graph_->latestPose();
    publishKeyframeDebug(true, stamp);
    RCLCPP_INFO(
        get_logger(),
        "Loop closed: keyframe %zu revisits %zu (fitness %.3f) - trajectory corrected",
        current,
        loop->old_index,
        loop->fitness);
}

}  // namespace olive
