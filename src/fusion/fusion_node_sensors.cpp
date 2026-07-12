// FusionNode — non-LiDAR sensor ingestion: IMU/wheel/marker callbacks, the
// stationary IMU init state machine, online gyro-bias re-estimation, and the
// marker motion gate. These fill the buffers the scan hot path consumes.
#include <algorithm>
#include <cmath>

#include "olive/common/gtsam_conversions.hpp"
#include "olive/fusion/fusion_node.hpp"

namespace olive
{

void FusionNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr& msg)
{
    ImuData sample;
    sample.timestamp = static_cast<double>(msg->header.stamp.sec) +
                       1e-9 * msg->header.stamp.nanosec + imu_time_offset_;
    logSensorLatency("imu", sample.timestamp);
    health_monitor_.beat("imu", sample.timestamp);
    sample.angular_velocity =
        Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    sample.linear_acceleration = Eigen::Vector3d(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z);
    imu_buffer_.push(sample);

    if (!imu_init_done_)
        handleImuInit(sample.timestamp);
}

void FusionNode::handleImuInit(double stamp)
{
    if (imu_init_first_stamp_ < 0.0)
        imu_init_first_stamp_ = stamp;
    if (imu_init_window_start_ < 0.0)
        imu_init_window_start_ = stamp;

    if (stamp - imu_init_window_start_ < imu_init_duration_s_)
    {
        // Give up rather than deadlock: a robot that starts moving immediately
        // still runs, just without a bias estimate.
        if (stamp - imu_init_first_stamp_ > imu_init_max_wait_s_)
        {
            RCLCPP_WARN(
                get_logger(),
                "IMU init: no stationary window within %.1f s - proceeding with zero gyro bias",
                imu_init_max_wait_s_);
            imu_init_done_ = true;
        }
        return;
    }

    const auto stats = imu_buffer_.windowStats(imu_init_window_start_, stamp);

    bool stationary = stats.count >= 10 && stats.gyro_deviation < stationary_gyro_thresh_;
    if (stationary && wheel_buffer_.hasData())
    {
        const auto wheel_motion = wheel_buffer_.relativePose(imu_init_window_start_, stamp);
        if (wheel_motion && wheel_motion->translation().norm() > stationary_wheel_thresh_)
            stationary = false;
    }

    if (!stationary)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            3000,
            "IMU init: motion detected during bias window - restarting collection");
        imu_init_window_start_ = stamp;
        return;
    }

    imu_buffer_.setGyroBias(stats.gyro_mean);
    imu_init_done_ = true;

    // Sanity checks that catch the classic driver misconfigurations.
    const double accel_norm = stats.accel_mean.norm();
    if (std::abs(accel_norm - constants::GRAVITY) > 0.2 * constants::GRAVITY)
    {
        RCLCPP_WARN(
            get_logger(),
            "IMU init: |accel| = %.2f m/s^2, expected ~9.8 - check units (g vs m/s^2)",
            accel_norm);
    }
    if (stats.gyro_mean.norm() > 0.05)
    {
        RCLCPP_WARN(
            get_logger(),
            "IMU init: gyro bias %.4f rad/s is unusually large - check units (deg/s vs rad/s)",
            stats.gyro_mean.norm());
    }
    const double tilt =
        std::acos(std::clamp(stats.accel_mean.normalized().z(), -1.0, 1.0)) * constants::RAD_TO_DEG;
    if (tilt > 5.0)
    {
        RCLCPP_WARN(
            get_logger(),
            "IMU init: gravity is %.1f deg off the base +z axis - check imu_rpy mounting "
            "rotation (or the robot is on a slope)",
            tilt);
    }

    RCLCPP_INFO(
        get_logger(),
        "IMU init complete: gyro bias [%.5f %.5f %.5f] rad/s over %zu samples "
        "(|accel| %.2f, tilt %.1f deg)",
        stats.gyro_mean.x(),
        stats.gyro_mean.y(),
        stats.gyro_mean.z(),
        stats.count,
        accel_norm,
        tilt);
}

void FusionNode::logSensorLatency(const char* sensor, double stamp)
{
    // One-shot characterization aid: the first few messages per sensor get an
    // arrival-minus-stamp report so clock offsets and driver latency are
    // visible at bring-up without extra tooling.
    int& count = latency_logged_[sensor];
    if (count >= 3)
        return;
    ++count;
    const double now = static_cast<double>(get_clock()->now().nanoseconds()) * 1e-9;
    RCLCPP_INFO(
        get_logger(),
        "%s stamp-to-arrival latency: %+.1f ms%s",
        sensor,
        (now - stamp) * 1e3,
        count == 3 ? " (final report; tune *_time_offset_s if large)" : "");
}

bool FusionNode::markerMotionGate(double stamp)
{
    // Real cameras blur under fast motion and the detector's centroid lags;
    // anchoring from such frames imprints the error into the graph.
    const double yaw_rate = std::abs(imu_buffer_.rateNear(stamp).z());
    if (yaw_rate > marker_max_yaw_rate_)
    {
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Marker detections rejected: yaw rate %.2f rad/s above limit",
            yaw_rate);
        return false;
    }
    if (wheel_buffer_.hasData())
    {
        const auto delta = wheel_buffer_.relativePose(stamp - 0.2, stamp);
        if (delta && delta->translation().norm() / 0.2 > marker_max_speed_)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Marker detections rejected: speed above limit");
            return false;
        }
    }
    return true;
}

void FusionNode::reestimateGyroBias()
{
    if (!imu_init_done_ || !imu_buffer_.hasData())
        return;

    const double now = static_cast<double>(get_clock()->now().nanoseconds()) * 1e-9;
    const double t0  = now - 1.0;

    // Only while demonstrably stationary: wheels quiet and gyro spread small.
    if (wheel_buffer_.hasData())
    {
        const auto wheel_motion = wheel_buffer_.relativePose(t0, now);
        if (!wheel_motion || wheel_motion->translation().norm() > stationary_wheel_thresh_)
            return;
    }
    const auto stats = imu_buffer_.windowStats(t0, now);
    if (stats.count < 10 || stats.gyro_deviation > stationary_gyro_thresh_)
        return;

    // Slow exponential update: tracks temperature drift without ever jumping.
    const Eigen::Vector3d updated = 0.9 * imu_buffer_.gyroBias() + 0.1 * stats.gyro_mean;
    imu_buffer_.setGyroBias(updated);
}

void FusionNode::wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr& msg)
{
    const double stamp = static_cast<double>(msg->header.stamp.sec) +
                         1e-9 * msg->header.stamp.nanosec + wheel_time_offset_;
    logSensorLatency("wheel", stamp);
    health_monitor_.beat("wheel", stamp);
    const gtsam::Pose3 wheel_pose = gtsam_conversions::toGtsamPose(msg->pose.pose);
    wheel_buffer_.push(stamp, wheel_pose);
    if (!wheel_origin_)
        wheel_origin_ = wheel_pose;  // anchors the smooth odom frame at node start
    last_wheel_twist_ = msg->twist.twist;
}

void FusionNode::markerCallback(const whycode_vision::msg::WhyCodePoseArray::SharedPtr& msg)
{
    const double stamp = static_cast<double>(msg->header.stamp.sec) +
                         1e-9 * msg->header.stamp.nanosec + camera_time_offset_;
    logSensorLatency("camera", stamp);
    health_monitor_.beat("markers", stamp);
    if (!markerMotionGate(stamp))
        return;
    for (const auto& detection : msg->poses)
    {
        marker_gate_->push(
            stamp,
            detection.whycode_id,
            detection.tracking_id,
            detection.id_valid,
            gtsam_conversions::toGtsamPoint(detection.pose.position));
    }
}

}  // namespace olive
