/**
 * @file imu_buffer.hpp
 * @brief Thread-safe IMU sample buffer providing short-horizon rotation prediction
 */

#ifndef OLIVE_FUSION_IMU_BUFFER_HPP_
#define OLIVE_FUSION_IMU_BUFFER_HPP_

#include <Eigen/Geometry>
#include <deque>
#include <mutex>

#include "olive/common/types.hpp"

namespace olive
{

/**
 * @brief Buffers IMU samples and integrates gyro rates on demand.
 *
 * Used to seed the scan matcher with a rotation prediction between two scan
 * timestamps. Samples are assumed to arrive in time order; old samples are
 * pruned automatically.
 */
class ImuBuffer
{
public:
    explicit ImuBuffer(double history_seconds = 5.0);

    /// Add a sample (called from the IMU subscription; only locks and pushes)
    void push(const ImuData& sample);

    /**
     * @brief Integrate gyro rates over [t0, t1]
     * @return Rotation of the body between the two times; identity when no
     *         samples cover the interval.
     */
    Eigen::Quaterniond relativeRotation(double t0, double t1) const;

    /// True once at least one sample is buffered
    bool hasData() const;

private:
    mutable std::mutex  mutex_;
    std::deque<ImuData> samples_;
    double              history_seconds_;
};

}  // namespace olive

#endif  // OLIVE_FUSION_IMU_BUFFER_HPP_
