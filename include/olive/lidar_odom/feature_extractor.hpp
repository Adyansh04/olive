/**
 * @file feature_extractor.hpp
 * @brief Feature extraction based on local curvature analysis
 *
 * Extracts geometric features from LiDAR scans:
 * - Edge features: High curvature points (corners, object edges)
 * - Planar features: Low curvature points (walls, floors)
 *
 * Edge features provide strong angular constraints for rotation.
 * Planar features provide translation constraints.
 *
 * Design: Single-pass curvature computation with partial sort
 * for efficient top-N / bottom-N feature selection.
 */

#ifndef OLIVE_LIDAR_FEATURE_EXTRACTOR_HPP_
#define OLIVE_LIDAR_FEATURE_EXTRACTOR_HPP_

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

#include "olive/lidar_odom/feature_types.hpp"

namespace olive
{

/**
 * @brief Curvature-based feature extractor
 *
 * Computes local curvature for each point and classifies as:
 * - Edge: curvature > edge_threshold
 * - Planar: curvature < planar_threshold
 *
 * Uses partial_sort for efficient feature selection without
 * sorting the entire point cloud.
 */
class FeatureExtractor
{
public:
    using PointCloud         = pcl::PointCloud<pcl::PointXYZ>;
    using PointCloudPtr      = PointCloud::Ptr;
    using PointCloudConstPtr = PointCloud::ConstPtr;

    /**
     * @brief Construct with configuration
     */
    explicit FeatureExtractor(const FeatureExtractionConfig& config = FeatureExtractionConfig());

    /**
     * @brief Extract features from unorganized point cloud
     * @param cloud Input point cloud
     * @param timestamp Scan timestamp
     * @return Extracted features
     *
     * ?Check thisFor unorganized clouds (e.g., from Gazebo), uses a simpler
     * ?approach based on local neighborhood curvature.
     */
    ExtractedFeatures extractUnorganized(const PointCloudConstPtr& cloud, double timestamp);

    /**
     * @brief Extract features from organized point cloud (scan lines)
     * @param cloud Input organized point cloud (rows = scan lines)
     * @param timestamp Scan timestamp
     * @return Extracted features
     *
     * ?For organized clouds (e.g., from real Velodyne), uses scan line
     * ?structure for more accurate curvature computation.
     */
    ExtractedFeatures extractOrganized(const PointCloudConstPtr& cloud, double timestamp);

    /**
     * @brief Update configuration
     */
    void setConfig(const FeatureExtractionConfig& config) { config_ = config; }

    /**
     * @brief Get current configuration
     */
    const FeatureExtractionConfig& getConfig() const { return config_; }

private:
    /**
     * @brief Compute curvature for each point in unorganized cloud
     */
    std::vector<PointWithCurvature> computeCurvatureUnorganized(const PointCloudConstPtr& cloud);

    /**
     * @brief Compute curvature using scan line structure
     */
    std::vector<std::vector<PointWithCurvature>>
        computeCurvatureOrganized(const PointCloudConstPtr& cloud);

    /**
     * @brief Select features from points sorted by curvature
     */
    void selectFeatures(
        std::vector<PointWithCurvature>& points,
        PointCloudPtr&                   edge_cloud,
        PointCloudPtr&                   planar_cloud,
        int                              max_edges,
        int                              max_planars);

    /**
     * @brief Mark neighboring points as invalid (to ensure spatial distribution)
     */
    void markNeighborsInvalid(std::vector<PointWithCurvature>& points, int center_idx, int radius);

    /**
     * @brief Check if point is valid (range, neighbors, etc.)
     */
    bool isValidPoint(const pcl::PointXYZ& pt) const;

    /**
     * @brief Compute range from origin
     */
    float computeRange(const pcl::PointXYZ& pt) const;

    FeatureExtractionConfig config_;

    // KD-tree
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;
};

}  // namespace olive

#endif  // OLIVE_LIDAR_FEATURE_EXTRACTOR_HPP_
