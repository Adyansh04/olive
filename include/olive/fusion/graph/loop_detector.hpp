/**
 * @file loop_detector.hpp
 * @brief ICP-based loop closure detection over the keyframe map
 */

#ifndef OLIVE_FUSION_GRAPH_LOOP_DETECTOR_HPP_
#define OLIVE_FUSION_GRAPH_LOOP_DETECTOR_HPP_

#include <gtsam/geometry/Pose3.h>

#include <optional>

#include "olive/fusion/fusion_types.hpp"
#include "olive/fusion/graph/keyframe_map.hpp"

namespace olive
{

/**
 * @brief Loop-closure parameters
 */
struct LoopDetectorConfig
{
    double search_radius     = 3.0;   ///< Candidate must lie within this of the pose (m)
    double min_time_diff     = 30.0;  ///< ... and be at least this much older (s)
    int    submap_half_width = 12;    ///< Candidate +- N cloud-bearing neighbors
    double fitness_threshold = 0.3;   ///< Max ICP mean-squared correspondence fitness
    double max_correction    = 3.0;   ///< Sanity gate: reject bigger implied jumps (m)
    float  submap_leaf_size  = 0.2F;  ///< Submap voxel size (m)
    bool   planar            = true;  ///< Project the loop relative to x/y/yaw
};

/**
 * @brief A detected loop: the current keyframe re-observes an old place
 */
struct LoopClosure
{
    size_t       old_index = 0;  ///< The revisited keyframe
    gtsam::Pose3 relative;       ///< X(old) -> X(current), ICP-refined
    double       fitness = 0.0;  ///< ICP fitness (lower is better)
};

/**
 * @brief Finds and verifies loop closures against the keyframe map.
 *
 * A candidate is the closest sufficiently-old, cloud-bearing, good-quality
 * keyframe within the search radius. The current keyframe's features are
 * ICP-aligned against a submap built from the candidate's neighborhood; the
 * result must pass a fitness gate and an implied-correction sanity gate (one
 * false loop with the loose start prior can fold the whole map).
 */
class LoopDetector
{
public:
    explicit LoopDetector(const LoopDetectorConfig& config);

    /**
     * @brief Attempt to close a loop for the newest keyframe
     * @param map            Keyframe database (newest = current)
     * @param current_index  Index of the current keyframe
     */
    std::optional<LoopClosure> detect(const KeyframeMap& map, size_t current_index) const;

private:
    std::optional<size_t> findCandidate(const KeyframeMap& map, size_t current_index) const;

    LoopDetectorConfig config_;
};

}  // namespace olive

#endif  // OLIVE_FUSION_GRAPH_LOOP_DETECTOR_HPP_
