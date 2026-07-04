/**
 * @file fusion_types.hpp
 * @brief Shared types and configuration structs for the OLIVE fusion pipeline
 */

#ifndef OLIVE_FUSION_FUSION_TYPES_HPP_
#define OLIVE_FUSION_FUSION_TYPES_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace olive
{

using CloudPoint = pcl::PointXYZI;
using Cloud      = pcl::PointCloud<CloudPoint>;

/**
 * @brief A LiDAR scan unrolled into a flat, ring-major array with the
 *        per-point metadata needed for curvature-based feature extraction.
 *
 * Points are stored ring by ring in increasing column (azimuth) order and are
 * already expressed in the robot base frame.
 */
struct ScanImage
{
    Cloud::Ptr         points;    ///< Flat cloud, ring-major order
    std::vector<float> range;     ///< Range (in the sensor frame) per point
    std::vector<int>   column;    ///< Horizontal (azimuth) index per point
    std::vector<float> rel_time;  ///< Per-point time relative to the header stamp (s);
                                  ///< all zeros when the cloud has no time field
    std::vector<int> ring_start;  ///< First flat index of each ring
    std::vector<int> ring_end;    ///< One-past-last flat index of each ring
    double           stamp = 0.0;

    ScanImage()
      : points(new Cloud)
    {}

    void clear()
    {
        points->clear();
        range.clear();
        column.clear();
        rel_time.clear();
        ring_start.clear();
        ring_end.clear();
        stamp = 0.0;
    }
};

/**
 * @brief Edge + planar feature clouds extracted from one scan (base frame)
 */
struct FeatureClouds
{
    Cloud::Ptr edge;
    Cloud::Ptr planar;
    double     stamp = 0.0;

    FeatureClouds()
      : edge(new Cloud)
      , planar(new Cloud)
    {}
};

/**
 * @brief Scan preprocessing parameters
 */
struct PreprocessorConfig
{
    float           min_range       = 0.3F;   ///< Points closer than this are dropped
    float           max_range       = 12.0F;  ///< Points farther than this are dropped
    Eigen::Affine3f base_from_lidar = Eigen::Affine3f::Identity();  ///< Fixed extrinsic

    /// Per-point time field: "auto" detects time/t/timestamp, "none" disables
    std::string point_time_field = "auto";
    /// Ring source for unorganized clouds: "auto" prefers a ring field
    std::string ring_field = "auto";
};

/**
 * @brief Feature extraction parameters
 */
struct FeatureConfig
{
    int curvature_window =
        2;  ///< Curvature half-window (points); small for coarse azimuth resolution
    float edge_threshold       = 1.0F;   ///< Curvature above which a point can be an edge
    float planar_threshold     = 0.1F;   ///< Curvature below which a point can be planar
    int   sectors_per_ring     = 6;      ///< Azimuth sectors for even feature spread
    int   max_edges_per_sector = 20;     ///< Edge picks per sector
    int   column_gap           = 3;      ///< Azimuth-index gap treated as discontinuity
    float occlusion_range_gap  = 0.3F;   ///< Range step marking an occlusion boundary (m)
    float parallel_beam_ratio  = 0.02F;  ///< Neighbor range ratio marking grazing beams
    float planar_leaf_size     = 0.4F;   ///< Per-ring planar-cloud voxel size (m)
};

/**
 * @brief Scan-to-map matcher parameters
 */
struct MatcherConfig
{
    int   max_iterations             = 30;
    int   min_edge_features          = 10;   ///< Below this the edge set is ignored
    int   min_planar_features        = 100;  ///< Below this the scan is not matched
    int   min_correspondences        = 50;
    float max_correspondence_sq_dist = 1.0F;  ///< 5th-NN gate (m^2)
    float degeneracy_eigen_threshold = 100.0F;
};

/**
 * @brief Keyframe / local-map parameters
 */
struct KeyframeConfig
{
    double keyframe_translation = 0.5;   ///< New keyframe after this motion (m)
    double keyframe_rotation    = 0.21;  ///< ... or this rotation (rad)
    double search_radius        = 25.0;  ///< Keyframes within this radius join the local map
    double recent_window        = 10.0;  ///< Keyframes newer than this always join (s)
    float  edge_leaf_size       = 0.2F;  ///< Local-map voxel size, edge cloud (m)
    float  planar_leaf_size     = 0.4F;  ///< Local-map voxel size, planar cloud (m)

    /// Cloud-storage bounds (0 = unbounded). Outside the recent window at
    /// most one keyframe per cloud_voxel cell keeps its clouds; the LRU cap
    /// is a safety net on top.
    double cloud_voxel         = 0.0;  ///< m; ~2-3x keyframe spacing
    size_t max_cloud_keyframes = 0;
};

}  // namespace olive

#endif  // OLIVE_FUSION_FUSION_TYPES_HPP_
