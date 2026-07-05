/**
 * @file imu_buffer.hpp
 * @brief Thread-safe IMU sample buffer providing short-horizon rotation prediction
 */

#ifndef OLIVE_FUSION_IMU_BUFFER_HPP_
#define OLIVE_FUSION_IMU_BUFFER_HPP_

#include <Eigen/Geometry>
#include <deque>
#include <mutex>
#include <vector>

#include "olive/common/types.hpp"

namespace olive
{

/**
 * @brief Buffers IMU samples and integrates gyro rates on demand.
 *
 * Used to seed the scan matcher with a rotation prediction between two scan
 * timestamps. Samples are assumed to arrive in time order; old samples are
 * pruned automatically.
 *
 * Real-hardware handling: a mounting rotation (base <- imu) is applied to
 * every incoming sample so all stored rates are body rates, and a gyro bias
 * (estimated externally, e.g. during a stationary startup window) is
 * subtracted at integration time — not baked into the stored samples, so it
 * can be re-estimated later from the same history.
 */
class ImuBuffer
{
public:
    explicit ImuBuffer(double history_seconds = 5.0);

    /// Fixed mounting rotation applied to incoming samples (base <- imu)
    void setMountingRotation(const Eigen::Quaterniond& base_from_imu);

    /// Gyro bias in BASE axes, subtracted at integration time
    void setGyroBias(const Eigen::Vector3d& bias);

    Eigen::Vector3d gyroBias() const;

    /// Add a sample (called from the IMU subscription; only locks and pushes)
    void push(const ImuData& sample);

    /**
     * @brief Integrate (bias-corrected) gyro rates over [t0, t1]
     * @return Rotation of the body between the two times; identity when no
     *         samples cover the interval.
     */
    Eigen::Quaterniond relativeRotation(double t0, double t1) const;

    /// Bias-corrected angular rate of the sample nearest to @p time
    Eigen::Vector3d rateNear(double time) const;

    /**
     * @brief Rotations from t0 to n+1 evenly spaced times across [t0, t1]
     *
     * result[k] = body rotation from t0 to t0 + k*(t1-t0)/n. Single pass
     * under one lock — used by scan deskew on the hot path.
     */
    std::vector<Eigen::Quaterniond> sampleRotations(double t0, double t1, int n) const;

    /**
     * @brief Statistics over the samples in [t0, t1] (raw, bias-uncorrected)
     * @return sample count; means/deviations are zero when count == 0
     */
    struct WindowStats
    {
        size_t          count = 0;
        Eigen::Vector3d gyro_mean{ 0.0, 0.0, 0.0 };
        Eigen::Vector3d accel_mean{ 0.0, 0.0, 0.0 };
        double          gyro_deviation  = 0.0;  ///< max |rate - mean| over the window
        double          accel_deviation = 0.0;  ///< max |accel - mean| over the window
    };
    WindowStats windowStats(double t0, double t1) const;

    /**
     * @brief Raw samples with t0 < timestamp <= t1 (base axes, bias NOT
     *        subtracted) — the input a bias-estimating preintegrator needs.
     *
     * Single lock, copies out. Called at keyframe rate, never per scan.
     */
    std::vector<ImuData> samplesBetween(double t0, double t1) const;

    /// True once at least one sample is buffered
    bool hasData() const;

private:
    mutable std::mutex  mutex_;
    std::deque<ImuData> samples_;
    double              history_seconds_;
    Eigen::Quaterniond  base_from_imu_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d     gyro_bias_{ 0.0, 0.0, 0.0 };
};

}  // namespace olive

#endif  // OLIVE_FUSION_IMU_BUFFER_HPP_
