/**
 * @file health_monitor.hpp
 * @brief Per-sensor liveness and quality bookkeeping
 */

#ifndef OLIVE_FUSION_HEALTH_MONITOR_HPP_
#define OLIVE_FUSION_HEALTH_MONITOR_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "olive/common/types.hpp"

namespace olive
{

/**
 * @brief Tracks when each sensor last delivered and how good its data is.
 *
 * Pure bookkeeping (no ROS): callbacks feed beat(), processing stages can
 * override quality (e.g. a degenerate scan match), and a periodic evaluate()
 * turns ages into SensorHealth states for reporting/decisions.
 */
class HealthMonitor
{
public:
    struct SensorSpec
    {
        std::string name;
        double      timeout = 0.0;  ///< s without data -> FAILED; 0 = optional (never FAILED)
    };

    struct Status
    {
        std::string  name;
        SensorHealth health = SensorHealth::FAILED;
        double       age    = -1.0;  ///< seconds since last beat; -1 = never seen
        std::string  detail;         ///< quality-override annotation
    };

    void configure(const std::vector<SensorSpec>& sensors);

    /// Record a delivery (called from sensor callbacks; cheap)
    void beat(const std::string& name, double stamp);

    /// Override the quality of a live sensor (cleared by the next beat)
    void flagQuality(const std::string& name, SensorHealth health, const std::string& detail);

    /// Evaluate all sensors at @p now
    std::vector<Status> evaluate(double now) const;

private:
    struct Entry
    {
        SensorSpec   spec;
        double       last_beat = -1.0;
        SensorHealth quality   = SensorHealth::GOOD;
        std::string  detail;
    };

    std::vector<std::string>               order_;
    std::unordered_map<std::string, Entry> sensors_;
};

}  // namespace olive

#endif  // OLIVE_FUSION_HEALTH_MONITOR_HPP_
