/**
 * @file feature_types.hpp
 * @brief Type definitions for feature extraction
 *
 * Features are extracted based on local curvature analysis:
 * - Edge points: High curvature (corners, object edges)
 * - Planar points: Low curvature (walls, ground, flat surfaces)
 *
 * Edge features provide strong angular constraints for rotation estimation.
 * Planar features provide translation constraints.
 */

#ifndef OLIVE_LIDAR_FEATURE_TYPES_HPP_
#define OLIVE_LIDAR_FEATURE_TYPES_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

namespace olive
{

/**
 * @brief Configuration for feature extraction
 */
struct FeatureExtractionConfig
{
    // Scan structure (for organized point clouds)
    int num_scan_lines{ 16 };     ///< Number of scan lines/rings
    int points_per_line{ 1800 };  ///< Approximate points per scan line

    // Curvature computation
    int curvature_region{ 5 };  ///< Neighbors on each side for curvature

    // Feature selection thresholds
    double edge_threshold{ 0.1 };     ///< Min curvature for edge classification
    double planar_threshold{ 0.01 };  ///< Max curvature for planar classification

    // Feature limits per scan line
    int max_edge_features_per_line{ 20 };
    int max_planar_features_per_line{ 40 };

    // Point validity filtering
    double min_range{ 0.5 };   ///< Minimum valid range (m)
    double max_range{ 50.0 };  ///< Maximum valid range (m)

    // Ground filtering for ground robots
    bool   filter_ground{ true };      ///< Remove ground plane points
    double ground_height_min{ -0.3 };  ///< Min height relative to sensor (m)
    double ground_height_max{ 0.1 };   ///< Max height to consider as ground (m)
    double sensor_height{ 0.3 };       ///< Sensor height above ground (m)

    // Spatial distribution
    int num_sectors{ 6 };  ///< Divide scan into sectors for even distribution
};

/**
 * @brief Point with computed curvature for feature selection
 */
struct PointWithCurvature
{
    pcl::PointXYZ point;
    float         curvature{ 0.0f };
    int           scan_line{ 0 };
    int           index_in_line{ 0 };
    bool          is_valid{ true };
};

/**
 * @brief Extracted features from a single scan
 */
struct ExtractedFeatures
{
    using PointCloud    = pcl::PointCloud<pcl::PointXYZ>;
    using PointCloudPtr = PointCloud::Ptr;

    PointCloudPtr edge_points;    ///< High curvature (edge) features
    PointCloudPtr planar_points;  ///< Low curvature (planar) features
    PointCloudPtr full_cloud;     ///< All valid points (for fallback)

    double timestamp{ 0.0 };

    ExtractedFeatures()
      : edge_points(new PointCloud())
      , planar_points(new PointCloud())
      , full_cloud(new PointCloud())
    {}

    void clear()
    {
        edge_points->clear();
        planar_points->clear();
        full_cloud->clear();
    }

    bool hasFeatures() const { return !edge_points->empty() || !planar_points->empty(); }

    size_t totalFeatures() const { return edge_points->size() + planar_points->size(); }

    size_t numEdges() const { return edge_points->size(); }
    size_t numPlanars() const { return planar_points->size(); }
};

}  // namespace olive

#endif  // OLIVE_LIDAR_FEATURE_TYPES_HPP_