/**
 * @file keyframe_map.hpp
 * @brief Keyframe database and local-map assembly for scan-to-map matching
 */

#ifndef OLIVE_FUSION_KEYFRAME_MAP_HPP_
#define OLIVE_FUSION_KEYFRAME_MAP_HPP_

#include <gtsam/geometry/Pose3.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <unordered_map>
#include <vector>

#include "olive/fusion/fusion_types.hpp"

namespace olive
{

/**
 * @brief One stored keyframe: optimized pose + its feature clouds (base frame)
 */
struct Keyframe
{
    gtsam::Pose3 pose;
    Cloud::Ptr   edge;
    Cloud::Ptr   planar;
    double       stamp = 0.0;
};

/**
 * @brief Owns all keyframes and assembles the local map around a query pose.
 *
 * The local map is the union of feature clouds from keyframes within a search
 * radius of the query position plus all recent keyframes, each transformed by
 * its (current) optimized pose and voxel-downsampled. Transformed clouds are
 * cached per keyframe; the cache must be invalidated whenever graph
 * optimization moves the keyframe poses.
 */
class KeyframeMap
{
public:
    explicit KeyframeMap(const KeyframeConfig& config);

    /// Number of stored keyframes
    size_t size() const { return keyframes_.size(); }

    bool empty() const { return keyframes_.empty(); }

    const Keyframe& back() const { return keyframes_.back(); }

    /// Store a new keyframe (clouds are NOT copied; ownership is shared)
    void
        add(const gtsam::Pose3& pose,
            const Cloud::Ptr&   edge,
            const Cloud::Ptr&   planar,
            double              stamp);

    /**
     * @brief True when motion since the last keyframe exceeds the thresholds
     */
    bool shouldAddKeyframe(const gtsam::Pose3& pose) const;

    /**
     * @brief Assemble the local edge/planar maps around @p position
     *
     * Outputs are owned by this class and remain valid until the next call.
     */
    void buildLocalMap(
        const gtsam::Point3& position,
        double               current_time,
        Cloud::Ptr&          edge_map,
        Cloud::Ptr&          planar_map);

    /// Overwrite a keyframe pose after graph optimization
    void updatePose(size_t index, const gtsam::Pose3& pose);

    /// Drop all cached transformed clouds (after poses changed)
    void invalidateCache();

private:
    KeyframeConfig config_;

    std::vector<Keyframe> keyframes_;
    Cloud::Ptr            keyframe_positions_;  ///< For the radius search

    // index -> feature clouds already transformed into the map frame
    std::unordered_map<size_t, std::pair<Cloud::Ptr, Cloud::Ptr>> transformed_cache_;

    Cloud::Ptr                 local_edge_;
    Cloud::Ptr                 local_planar_;
    pcl::VoxelGrid<CloudPoint> edge_filter_;
    pcl::VoxelGrid<CloudPoint> planar_filter_;
};

}  // namespace olive

#endif  // OLIVE_FUSION_KEYFRAME_MAP_HPP_
