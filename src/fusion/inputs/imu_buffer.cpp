#include "olive/fusion/inputs/imu_buffer.hpp"

#include <algorithm>
#include <cmath>

namespace olive
{

namespace
{

Eigen::Quaterniond rotationFromRate(const Eigen::Vector3d& rate, double dt)
{
    const double norm = rate.norm();
    if (norm < 1e-12 || dt <= 0.0)
        return Eigen::Quaterniond::Identity();
    return Eigen::Quaterniond(Eigen::AngleAxisd(norm * dt, rate / norm));
}

}  // namespace

ImuBuffer::ImuBuffer(double history_seconds)
  : history_seconds_(history_seconds)
{}

void ImuBuffer::setMountingRotation(const Eigen::Quaterniond& base_from_imu)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    base_from_imu_ = base_from_imu.normalized();
}

void ImuBuffer::setGyroBias(const Eigen::Vector3d& bias)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    gyro_bias_ = bias;
}

Eigen::Vector3d ImuBuffer::gyroBias() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return gyro_bias_;
}

void ImuBuffer::push(const ImuData& sample)
{
    const std::lock_guard<std::mutex> lock(mutex_);

    // Store in BASE axes so every consumer (prediction, deskew, stationarity
    // checks) sees body rates regardless of how the IMU is mounted.
    ImuData rotated             = sample;
    rotated.angular_velocity    = base_from_imu_ * sample.angular_velocity;
    rotated.linear_acceleration = base_from_imu_ * sample.linear_acceleration;

    samples_.push_back(rotated);
    const double cutoff = rotated.timestamp - history_seconds_;
    while (!samples_.empty() && samples_.front().timestamp < cutoff)
        samples_.pop_front();
}

Eigen::Quaterniond ImuBuffer::relativeRotation(double t0, double t1) const
{
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    if (t1 <= t0)
        return rotation;

    const std::lock_guard<std::mutex> lock(mutex_);

    double          previous_time = t0;
    Eigen::Vector3d last_rate     = Eigen::Vector3d::Zero();
    bool            any_sample    = false;
    for (const ImuData& sample : samples_)
    {
        if (sample.timestamp <= t0)
            continue;
        if (sample.timestamp > t1)
            break;
        last_rate  = sample.angular_velocity - gyro_bias_;
        any_sample = true;

        rotation      = rotation * rotationFromRate(last_rate, sample.timestamp - previous_time);
        previous_time = sample.timestamp;
    }

    // Samples may not have arrived for the very end of the window yet;
    // extrapolate the remainder with the last observed rate.
    const double tail = t1 - previous_time;
    if (any_sample && tail > 1e-4)
        rotation = rotation * rotationFromRate(last_rate, tail);

    rotation.normalize();
    return rotation;
}

Eigen::Vector3d ImuBuffer::rateNear(double time) const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    if (samples_.empty())
        return Eigen::Vector3d::Zero();

    const auto it = std::min_element(
        samples_.begin(),
        samples_.end(),
        [time](const ImuData& a, const ImuData& b) {
            return std::abs(a.timestamp - time) < std::abs(b.timestamp - time);
        });
    return it->angular_velocity - gyro_bias_;
}

std::vector<Eigen::Quaterniond> ImuBuffer::sampleRotations(double t0, double t1, int n) const
{
    std::vector<Eigen::Quaterniond> rotations;
    rotations.reserve(static_cast<size_t>(n) + 1);
    rotations.push_back(Eigen::Quaterniond::Identity());
    if (n < 1 || t1 <= t0)
        return rotations;

    const std::lock_guard<std::mutex> lock(mutex_);

    const double       dt        = (t1 - t0) / n;
    Eigen::Quaterniond rotation  = Eigen::Quaterniond::Identity();
    double             cursor    = t0;
    Eigen::Vector3d    last_rate = Eigen::Vector3d::Zero();
    auto               it        = samples_.begin();

    for (int k = 1; k <= n; ++k)
    {
        const double bin_end = t0 + k * dt;
        while (it != samples_.end() && it->timestamp <= bin_end)
        {
            if (it->timestamp > cursor)
            {
                last_rate = it->angular_velocity - gyro_bias_;
                rotation  = rotation * rotationFromRate(last_rate, it->timestamp - cursor);
                cursor    = it->timestamp;
            }
            ++it;
        }
        if (bin_end > cursor)
        {
            // No sample up to the bin edge yet: hold the last rate.
            rotation = rotation * rotationFromRate(last_rate, bin_end - cursor);
            cursor   = bin_end;
        }
        rotations.push_back(rotation.normalized());
    }
    return rotations;
}

ImuBuffer::WindowStats ImuBuffer::windowStats(double t0, double t1) const
{
    const std::lock_guard<std::mutex> lock(mutex_);

    WindowStats stats;
    for (const ImuData& sample : samples_)
    {
        if (sample.timestamp < t0 || sample.timestamp > t1)
            continue;
        stats.gyro_mean += sample.angular_velocity;
        stats.accel_mean += sample.linear_acceleration;
        ++stats.count;
    }
    if (stats.count == 0)
        return stats;
    stats.gyro_mean /= static_cast<double>(stats.count);
    stats.accel_mean /= static_cast<double>(stats.count);

    for (const ImuData& sample : samples_)
    {
        if (sample.timestamp < t0 || sample.timestamp > t1)
            continue;
        stats.gyro_deviation =
            std::max(stats.gyro_deviation, (sample.angular_velocity - stats.gyro_mean).norm());
        stats.accel_deviation =
            std::max(stats.accel_deviation, (sample.linear_acceleration - stats.accel_mean).norm());
    }
    return stats;
}

std::vector<ImuData> ImuBuffer::samplesBetween(double t0, double t1) const
{
    const std::lock_guard<std::mutex> lock(mutex_);

    std::vector<ImuData> result;
    result.reserve(static_cast<size_t>(std::max(0.0, (t1 - t0) * 250.0)));
    for (const ImuData& sample : samples_)
    {
        if (sample.timestamp <= t0)
            continue;
        if (sample.timestamp > t1)
            break;  // samples are time-ordered
        result.push_back(sample);
    }
    return result;
}

bool ImuBuffer::hasData() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return !samples_.empty();
}

}  // namespace olive
