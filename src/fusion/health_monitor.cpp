#include "olive/fusion/health_monitor.hpp"

namespace olive
{

void HealthMonitor::configure(const std::vector<SensorSpec>& sensors)
{
    order_.clear();
    sensors_.clear();
    for (const SensorSpec& spec : sensors)
    {
        order_.push_back(spec.name);
        sensors_[spec.name] = Entry{ spec, -1.0, SensorHealth::GOOD, {} };
    }
}

void HealthMonitor::beat(const std::string& name, double stamp)
{
    auto it = sensors_.find(name);
    if (it == sensors_.end())
        return;
    Entry& entry = it->second;
    if (stamp > entry.last_beat)
        entry.last_beat = stamp;
    entry.quality = SensorHealth::GOOD;
    entry.detail.clear();
}

void HealthMonitor::flagQuality(
    const std::string& name,
    SensorHealth       health,
    const std::string& detail)
{
    auto it = sensors_.find(name);
    if (it == sensors_.end())
        return;
    it->second.quality = health;
    it->second.detail  = detail;
}

std::vector<HealthMonitor::Status> HealthMonitor::evaluate(double now) const
{
    std::vector<Status> result;
    result.reserve(order_.size());
    for (const std::string& name : order_)
    {
        const Entry& entry = sensors_.at(name);

        Status status;
        status.name   = name;
        status.detail = entry.detail;
        status.age    = entry.last_beat < 0.0 ? -1.0 : now - entry.last_beat;

        const bool required = entry.spec.timeout > 0.0;
        if (entry.last_beat < 0.0)
            status.health = required ? SensorHealth::FAILED : SensorHealth::POOR;
        else if (required && status.age > entry.spec.timeout)
            status.health = SensorHealth::FAILED;
        else
            status.health = entry.quality;

        result.push_back(status);
    }
    return result;
}

}  // namespace olive
