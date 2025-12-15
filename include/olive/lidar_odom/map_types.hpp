/**
 * @file map_types.hpp
 * @brief Type definitions for local map management
 *
 * The local map is a sliding window of recent keyframes.
 * Features from keyframes are accumulated and downsampled
 * to create a target for scan-to-map registration.
 *
 * Benefits over frame-to-frame:
 * - More points = stronger geometric constraints
 * - Averaging reduces measurement noise
 * - Better rotation estimation
 */

#ifndef OLIVE_LIDAR_MAP_TYPES_HPP_
#define OLIVE_LIDAR_MAP_TYPES_HPP_

#include "olive/common/types.hpp"
#include "olive/lidar_odom/feature_types.hpp"

namespace olive
{

/**
 * @brief Configuration for local map management
 */
struct LocalMapConfig
{
    // Sliding window size
    int    max_keyframes{10};       ///< Maximum keyframes in window
    double map_radius{50.0};        ///< Radius around robot to keep (m)

    // Downsampling voxel sizes
    double edge_voxel_size{0.2};    ///< Voxel size for edge features (m)
    double planar_voxel_size{0.4};  ///< Voxel size for planar features (m)
    double full_voxel_size{0.3};    ///< Voxel size for full cloud (m)

    // Keyframe insertion thresholds
    double update_distance{0.5};    ///< Min translation for new keyframe (m)
    double update_rotation{0.2};    ///< Min rotation for new keyframe (rad)

    // Point count limits for performance
    int max_edge_points{5000};      ///< Max edge points in map
    int max_planar_points{10000};   ///< Max planar points in map
    int max_full_points{20000};     ///< Max full cloud points in map
};

/**
 * @brief A keyframe with features and associated pose
 */
struct FeatureKeyframe
{
    ExtractedFeatures features;     ///< Extracted features at this pose
    Pose3D            pose;         ///< Pose in odom frame when captured
    double            timestamp{0.0};
};

}  // namespace olive

#endif  // OLIVE_LIDAR_MAP_TYPES_HPP_