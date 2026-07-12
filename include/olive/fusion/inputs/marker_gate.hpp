/**
 * @file marker_gate.hpp
 * @brief Gating and buffering of fiducial marker detections
 */

#ifndef OLIVE_FUSION_INPUTS_MARKER_GATE_HPP_
#define OLIVE_FUSION_INPUTS_MARKER_GATE_HPP_

#include <gtsam/geometry/Point3.h>

#include <cstdint>
#include <deque>
#include <mutex>
#include <set>
#include <unordered_map>
#include <vector>

namespace olive
{

/// Undecoded tracks are keyed by tracking id in a disjoint landmark-id range
/// so they can never collide with decoded WhyCode ids.
constexpr int64_t UNDECODED_LANDMARK_BASE = 1'000'000;

/**
 * @brief One accepted marker observation (camera-frame position)
 */
struct MarkerObservation
{
    double  stamp           = 0.0;
    int     marker_id       = -1;   ///< decoded WhyCode id (-1 when undecoded)
    int64_t landmark_key_id = -1;   ///< stable graph key: whycode id, or
                                    ///< UNDECODED_LANDMARK_BASE + tracking id
    bool          decoded = false;  ///< id bits decoded and valid
    gtsam::Point3 position_in_camera{ 0.0, 0.0, 0.0 };
};

/**
 * @brief Configuration for detection gating
 */
struct MarkerGateConfig
{
    std::set<int> known_ids;                     ///< Surveyed markers (world priors)
    bool          accept_unknown_ids   = false;  ///< decoded but unsurveyed -> free landmark
    bool          accept_undecoded_ids = false;  ///< id_valid false -> tracking-id landmark
    double        min_range            = 0.5;    ///< Reject implausibly close detections (m)
    double        max_range            = 6.0;    ///< Reject weak distant detections (m)
    int           min_track_frames     = 3;      ///< Consecutive frames before a track is trusted
    double        history              = 2.0;    ///< Observation retention (s)
};

/**
 * @brief Filters raw detector output into trustworthy landmark observations.
 *
 * The detector wire format carries no covariance or confidence, so trust is
 * established here: the range must be plausible and the same tracking ID must
 * persist for several consecutive frames before its observations pass. Which
 * detections pass depends on the acceptance class: surveyed ids always do,
 * decoded-but-unsurveyed and undecoded tracks only when the corresponding
 * accept flag is set (they become free landmarks — odometry constraints, not
 * world anchors). The circle-fit position is good even when the ID bits fail
 * to decode, which is why undecoded tracks are usable at all.
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

#endif  // OLIVE_FUSION_INPUTS_MARKER_GATE_HPP_
