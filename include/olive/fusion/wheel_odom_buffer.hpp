/**
 * @file wheel_odom_buffer.hpp
 * @brief Thread-safe wheel odometry buffer with interpolated pose queries
 */

#ifndef OLIVE_FUSION_WHEEL_ODOM_BUFFER_HPP_
#define OLIVE_FUSION_WHEEL_ODOM_BUFFER_HPP_

#include <gtsam/geometry/Pose3.h>

#include <deque>
#include <mutex>
#include <optional>

namespace olive
{

/**
 * @brief Buffers timestamped wheel odometry poses (odom frame).
 *
 * Provides the interpolated pose at an arbitrary query time and the relative
 * motion between two times — the measurement a wheel between-factor needs.
 */
class WheelOdomBuffer
{
public:
    explicit WheelOdomBuffer(double history_seconds = 30.0);

    /// Max extrapolation beyond the buffered range before queries fail (s).
    /// Real wheel drivers can lag scan stamps by more than the default.
    void setInterpolationSlack(double seconds);

    /// Add a sample (called from the odometry subscription)
    void push(double timestamp, const gtsam::Pose3& pose);

    /// Interpolated pose at @p time; empty when the buffer does not cover it
    std::optional<gtsam::Pose3> poseAt(double time) const;

    /**
     * @brief Wheel-measured motion between two times
     * @return T(t0)^-1 * T(t1), or empty when either bound is uncovered
     */
    std::optional<gtsam::Pose3> relativePose(double t0, double t1) const;

    bool hasData() const;

private:
    /// Requires the lock to be held
    std::optional<gtsam::Pose3> interpolate(double time) const;

    mutable std::mutex                          mutex_;
    std::deque<std::pair<double, gtsam::Pose3>> samples_;
    double                                      history_seconds_;
    double                                      slack_seconds_ = 0.05;
};

}  // namespace olive

#endif  // OLIVE_FUSION_WHEEL_ODOM_BUFFER_HPP_
