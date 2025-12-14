/**
 * @file feature_extractor.hpp
 * @brief LOAM-style feature extraction for LiDAR point clouds
 *
 * Extracts geometric features from LiDAR scans:
 * - Edge features: High curvature points (corners, edges of objects)
 * - Planar features: Low curvature points (walls, ground, flat surfaces)
 *
 * Reference: LOAM (Lidar Odometry and Mapping in Real-time)
 * Zhang, J., & Singh, S. (2014)
 */

#ifndef OLIVE_LIDAR_FEATURE_EXTRACTOR_HPP_
#define OLIVE_LIDAR_FEATURE_EXTRACTOR_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <vector>

namespace olive
{

/**
 * @brief Configuration for feature extraction
 */
struct FeatureExtractionConfig
{
    // Scan organization (for structured LiDAR like Velodyne)
    int num_scan_lines = 16;       ///< Number of scan lines (rings)
    int points_per_line = 1800;    ///< Approximate points per scan line

    // Curvature computation
    int curvature_region = 5;      ///< Number of neighbors on each side for curvature

    // Feature selection thresholds
    double edge_threshold = 0.1;   ///< Curvature threshold for edge features
    double planar_threshold = 0.01; ///< Curvature threshold for planar features

    // Feature count limits (per scan line)
    int max_edge_features_per_line = 20;    ///< Max sharp edges per line
    int max_planar_features_per_line = 40;  ///< Max planar points per line

    // Filtering
    double min_range = 0.5;        ///< Minimum valid range (m)
    double max_range = 50.0;       ///< Maximum valid range (m)

    // Ground filtering for ground robots
    bool filter_ground = true;     ///< Remove ground plane points
    double ground_height_min = -0.3; ///< Min height relative to sensor (m)
    double ground_height_max = 0.1;  ///< Max height to consider as ground (m)
    double sensor_height = 0.3;    ///< Sensor height above ground (m)

    // Sector-based selection (ensures spatial distribution)
    int num_sectors = 6;           ///< Divide scan line into sectors
};

/**
 * @brief Point with curvature information
 */
struct PointWithCurvature
{
    pcl::PointXYZ point;
    float curvature;
    int scan_line;
    int index_in_line;
    bool is_valid;

    PointWithCurvature()
        : curvature(0.0f)
        , scan_line(0)
        , index_in_line(0)
        , is_valid(true)
    {}
};

/**
 * @brief Extracted features from a single scan
 */
struct ExtractedFeatures
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_points;      ///< High curvature (sharp) features
    pcl::PointCloud<pcl::PointXYZ>::Ptr planar_points;    ///< Low curvature (flat) features
    pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud;       ///< All valid points (downsampled)

    double timestamp;

    ExtractedFeatures()
        : edge_points(new pcl::PointCloud<pcl::PointXYZ>())
        , planar_points(new pcl::PointCloud<pcl::PointXYZ>())
        , full_cloud(new pcl::PointCloud<pcl::PointXYZ>())
        , timestamp(0.0)
    {}

    void clear()
    {
        edge_points->clear();
        planar_points->clear();
        full_cloud->clear();
    }

    bool hasFeatures() const
    {
        return !edge_points->empty() || !planar_points->empty();
    }

    size_t totalFeatures() const
    {
        return edge_points->size() + planar_points->size();
    }
};

/**
 * @brief LOAM-style feature extractor
 *
 * Extracts edge and planar features based on local curvature analysis.
 * The curvature is computed as the variance of neighboring points.
 */
class FeatureExtractor
{
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using PointCloudPtr = PointCloud::Ptr;
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
     * For unorganized clouds (e.g., from Gazebo), uses a simpler
     * approach based on local neighborhood curvature.
     */
    ExtractedFeatures extractUnorganized(const PointCloudConstPtr& cloud, double timestamp);

    /**
     * @brief Extract features from organized point cloud
     * @param cloud Input organized point cloud (rows = scan lines)
     * @param timestamp Scan timestamp
     * @return Extracted features
     *
     * For organized clouds (e.g., from real Velodyne), uses scan line
     * structure for more accurate curvature computation.
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
    std::vector<std::vector<PointWithCurvature>> computeCurvatureOrganized(
        const PointCloudConstPtr& cloud);

    /**
     * @brief Select features from points sorted by curvature
     */
    void selectFeatures(
        std::vector<PointWithCurvature>& points,
        PointCloudPtr& edge_cloud,
        PointCloudPtr& planar_cloud,
        int max_edges,
        int max_planars);

    /**
     * @brief Mark neighboring points as invalid (to ensure spatial distribution)
     */
    void markNeighborsInvalid(
        std::vector<PointWithCurvature>& points,
        int center_idx,
        int radius);

    /**
     * @brief Check if point is valid (range, neighbors, etc.)
     */
    bool isValidPoint(const pcl::PointXYZ& pt) const;

    /**
     * @brief Compute range from origin
     */
    float computeRange(const pcl::PointXYZ& pt) const;

    FeatureExtractionConfig config_;
};

}  // namespace olive

#endif  // OLIVE_LIDAR_FEATURE_EXTRACTOR_HPP_
