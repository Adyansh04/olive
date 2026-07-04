/**
 * @file marker_gate.hpp
 * @brief Gating and buffering of fiducial marker detections
 */

#ifndef OLIVE_FUSION_MARKER_GATE_HPP_
#define OLIVE_FUSION_MARKER_GATE_HPP_

#include <gtsam/geometry/Point3.h>

#include <deque>
#include <mutex>
#include <set>
#include <unordered_map>
#include <vector>

namespace olive
{

/**
 * @brief One accepted marker observation (camera-frame position)
 */
struct MarkerObservation
{
    double        stamp     = 0.0;
    int           marker_id = -1;
    gtsam::Point3 position_in_camera{ 0.0, 0.0, 0.0 };
};

/**
 * @brief Configuration for detection gating
 */
struct MarkerGateConfig
{
    std::set<int> known_ids;               ///< Only these markers anchor the graph
    double        min_range        = 0.5;  ///< Reject implausibly close detections (m)
    double        max_range        = 6.0;  ///< Reject weak distant detections (m)
    int           min_track_frames = 3;    ///< Consecutive frames before a track is trusted
    double        history          = 2.0;  ///< Observation retention (s)
};

/**
 * @brief Filters raw detector output into trustworthy anchor observations.
 *
 * The detector wire format carries no covariance or confidence, so trust is
 * established here: the decoded ID must be flagged valid and known, the range
 * must be plausible, and the same tracking ID must persist for several
 * consecutive frames before its observations pass.
 */
class MarkerGate
{
public:
    explicit MarkerGate(const MarkerGateConfig& config);

    /// Feed one raw detection; returns true when it was accepted
    bool push(
        double               stamp,
        int                  whycode_id,
        int                  tracking_id,
        bool                 id_valid,
        const gtsam::Point3& position_in_camera);

    /// Accepted observations within +-window of @p stamp (consumed)
    std::vector<MarkerObservation> collectNear(double stamp, double window);

private:
    MarkerGateConfig config_;

    mutable std::mutex            mutex_;
    std::deque<MarkerObservation> accepted_;
    std::unordered_map<int, int>  track_streak_;  ///< tracking_id -> consecutive frames
};

}  // namespace olive

#endif  // OLIVE_FUSION_MARKER_GATE_HPP_
