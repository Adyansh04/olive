#include "olive/fusion/imu_buffer.hpp"

#include <algorithm>

namespace olive
{

ImuBuffer::ImuBuffer(double history_seconds)
  : history_seconds_(history_seconds)
{}

void ImuBuffer::push(const ImuData& sample)
{
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.push_back(sample);
    const double cutoff = sample.timestamp - history_seconds_;
    while (!samples_.empty() && samples_.front().timestamp < cutoff)
        samples_.pop_front();
}

Eigen::Quaterniond ImuBuffer::relativeRotation(double t0, double t1) const
{
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    if (t1 <= t0)
        return rotation;

    std::lock_guard<std::mutex> lock(mutex_);

    double          previous_time = t0;
    Eigen::Vector3d last_rate     = Eigen::Vector3d::Zero();
    for (const ImuData& sample : samples_)
    {
        if (sample.timestamp <= t0)
            continue;
        if (sample.timestamp > t1)
            break;
        last_rate = sample.angular_velocity;

        const double dt = sample.timestamp - previous_time;
        rotation        = rotation * Eigen::Quaterniond(Eigen::AngleAxisd(
                                  sample.angular_velocity.norm() * dt,
                                  sample.angular_velocity.norm() > 1e-12 ?
                                             sample.angular_velocity.normalized() :
                                             Eigen::Vector3d::UnitZ()));
        previous_time   = sample.timestamp;
    }
    // Samples may not have arrived for the very end of the window yet;
    // extrapolate the remainder with the last observed rate.
    const double tail = t1 - previous_time;
    if (tail > 1e-4 && last_rate.norm() > 1e-12)
        rotation = rotation *
                   Eigen::Quaterniond(Eigen::AngleAxisd(last_rate.norm() * tail, last_rate.normalized()));

    rotation.normalize();
    return rotation;
}

bool ImuBuffer::hasData() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return !samples_.empty();
}

}  // namespace olive
