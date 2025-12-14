/**
 * @file local_map.hpp
 * @brief Sliding window local map for scan-to-map registration
 *
 * Maintains a local submap built from recent keyframes for more
 * robust registration than frame-to-frame matching.
 *
 * Benefits:
 * - More points = stronger constraints
 * - Averaging effect reduces noise
 * - Better rotation estimation
 */

#ifndef OLIVE_LIDAR_LOCAL_MAP_HPP_
#define OLIVE_LIDAR_LOCAL_MAP_HPP_

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <deque>
#include <memory>
#include <mutex>

#include "olive/common/types.hpp"
#include "olive/lidar_odom/feature_extractor.hpp"

namespace olive
{

/**
 * @brief Configuration for local map management
 */
struct LocalMapConfig
{
    // Map size limits
    int max_keyframes = 10;              ///< Maximum keyframes in sliding window
    double map_radius = 50.0;            ///< Radius around current pose to keep (m)

    // Downsampling for map points
    double edge_voxel_size = 0.2;        ///< Voxel size for edge features (m)
    double planar_voxel_size = 0.4;      ///< Voxel size for planar features (m)
    double full_voxel_size = 0.3;        ///< Voxel size for full cloud (m)

    // Map update thresholds
    double update_distance = 0.5;        ///< Min distance to add new keyframe (m)
    double update_rotation = 0.2;        ///< Min rotation to add new keyframe (rad)

    // Point limits for performance
    int max_edge_points = 5000;          ///< Max edge points in map
    int max_planar_points = 10000;       ///< Max planar points in map
    int max_full_points = 20000;         ///< Max full cloud points in map
};

/**
 * @brief Keyframe with features and pose
 */
struct FeatureKeyframe
{
    ExtractedFeatures features;          ///< Extracted features
    Pose3D pose;                         ///< Pose in odom frame
    double timestamp;

    FeatureKeyframe() : timestamp(0.0) {}
};

/**
 * @brief Local map manager with sliding window
 *
 * Accumulates features from recent keyframes into a local submap.
 * The submap is used as the target for scan-to-map registration.
 */
class LocalMap
{
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using PointCloudPtr = PointCloud::Ptr;
    using PointCloudConstPtr = PointCloud::ConstPtr;

    /**
     * @brief Construct with configuration
     */
    explicit LocalMap(const LocalMapConfig& config = LocalMapConfig());

    /**
     * @brief Add a new keyframe to the map
     * @param features Extracted features from the scan
     * @param pose Current pose in odom frame
     * @return True if keyframe was added (met distance/rotation threshold)
     */
    bool addKeyframe(const ExtractedFeatures& features, const Pose3D& pose);

    /**
     * @brief Get the accumulated edge feature map
     */
    PointCloudConstPtr getEdgeMap() const { return edge_map_; }

    /**
     * @brief Get the accumulated planar feature map
     */
    PointCloudConstPtr getPlanarMap() const { return planar_map_; }

    /**
     * @brief Get the full point map (for fallback GICP)
     */
    PointCloudConstPtr getFullMap() const { return full_map_; }

    /**
     * @brief Check if map has sufficient features for registration
     */
    bool isReady() const;

    /**
     * @brief Get number of keyframes in map
     */
    size_t numKeyframes() const { return keyframes_.size(); }

    /**
     * @brief Get total edge points in map
     */
    size_t numEdgePoints() const { return edge_map_ ? edge_map_->size() : 0; }

    /**
     * @brief Get total planar points in map
     */
    size_t numPlanarPoints() const { return planar_map_ ? planar_map_->size() : 0; }

    /**
     * @brief Clear all keyframes and maps
     */
    void clear();

    /**
     * @brief Update configuration
     */
    void setConfig(const LocalMapConfig& config) { config_ = config; }

    /**
     * @brief Get pose of last added keyframe
     */
    const Pose3D& getLastKeyframePose() const { return last_keyframe_pose_; }

private:
    /**
     * @brief Transform features to odom frame and add to map
     */
    void addFeaturesToMap(const ExtractedFeatures& features, const Pose3D& pose);

    /**
     * @brief Remove old keyframes outside sliding window
     */
    void removeOldKeyframes();

    /**
     * @brief Rebuild map from keyframes (after removal)
     */
    void rebuildMap();

    /**
     * @brief Downsample a point cloud
     */
    void downsampleCloud(PointCloudPtr& cloud, double voxel_size);

    /**
     * @brief Check if should add new keyframe based on motion
     */
    bool shouldAddKeyframe(const Pose3D& pose) const;

    /**
     * @brief Transform point cloud by pose
     */
    PointCloudPtr transformCloud(const PointCloudConstPtr& cloud, const Pose3D& pose);

    LocalMapConfig config_;

    // Sliding window of keyframes
    std::deque<FeatureKeyframe> keyframes_;

    // Accumulated maps
    PointCloudPtr edge_map_;
    PointCloudPtr planar_map_;
    PointCloudPtr full_map_;

    // Last keyframe pose (for motion threshold check)
    Pose3D last_keyframe_pose_;
    bool has_keyframe_ = false;

    // Voxel filters
    pcl::VoxelGrid<pcl::PointXYZ> edge_filter_;
    pcl::VoxelGrid<pcl::PointXYZ> planar_filter_;
    pcl::VoxelGrid<pcl::PointXYZ> full_filter_;

    // Thread safety
    mutable std::mutex map_mutex_;
};

}  // namespace olive

#endif  // OLIVE_LIDAR_LOCAL_MAP_HPP_
