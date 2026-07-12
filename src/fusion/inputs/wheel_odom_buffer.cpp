#include "olive/fusion/inputs/wheel_odom_buffer.hpp"

namespace olive
{

WheelOdomBuffer::WheelOdomBuffer(double history_seconds)
  : history_seconds_(history_seconds)
{}

void WheelOdomBuffer::setInterpolationSlack(double seconds)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    slack_seconds_ = seconds;
}

void WheelOdomBuffer::push(double timestamp, const gtsam::Pose3& pose)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    samples_.emplace_back(timestamp, pose);
    const double cutoff = timestamp - history_seconds_;
    while (!samples_.empty() && samples_.front().first < cutoff)
        samples_.pop_front();
}

std::optional<gtsam::Pose3> WheelOdomBuffer::poseAt(double time) const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return interpolate(time);
}

std::optional<gtsam::Pose3> WheelOdomBuffer::relativePose(double t0, double t1) const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    const auto                  pose0 = interpolate(t0);
    const auto                  pose1 = interpolate(t1);
    if (!pose0 || !pose1)
        return std::nullopt;
    return pose0->between(*pose1);
}

bool WheelOdomBuffer::hasData() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return !samples_.empty();
}

double WheelOdomBuffer::latestStamp() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return samples_.empty() ? -1.0 : samples_.back().first;
}

std::optional<gtsam::Pose3> WheelOdomBuffer::interpolate(double time) const
{
    if (samples_.empty())
        return std::nullopt;

    // Tolerate limited extrapolation at the ends (sensor latency).
    if (time < samples_.front().first - slack_seconds_ ||
        time > samples_.back().first + slack_seconds_)
        return std::nullopt;
    if (time <= samples_.front().first)
        return samples_.front().second;
    if (time >= samples_.back().first)
        return samples_.back().second;

    // Binary search for the bracketing pair.
    size_t lo = 0;
    size_t hi = samples_.size() - 1;
    while (hi - lo > 1)
    {
        const size_t mid = (lo + hi) / 2;
        if (samples_[mid].first <= time)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }

    const double span  = samples_[hi].first - samples_[lo].first;
    const double alpha = span > 1e-9 ? (time - samples_[lo].first) / span : 0.0;
    return samples_[lo].second.interpolateRt(samples_[hi].second, alpha);
}

}  // namespace olive
