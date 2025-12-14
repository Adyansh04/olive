/**
 * @file local_map.cpp
 * @brief Implementation of sliding window local map
 */

#include "olive/lidar_odom/local_map.hpp"

#include <pcl/common/transforms.h>

#include <Eigen/Geometry>

namespace olive
{

LocalMap::LocalMap(const LocalMapConfig& config)
    : config_(config)
    , edge_map_(new PointCloud())
    , planar_map_(new PointCloud())
    , full_map_(new PointCloud())
{
    // Configure voxel filters
    edge_filter_.setLeafSize(
        config_.edge_voxel_size,
        config_.edge_voxel_size,
        config_.edge_voxel_size);

    planar_filter_.setLeafSize(
        config_.planar_voxel_size,
        config_.planar_voxel_size,
        config_.planar_voxel_size);

    full_filter_.setLeafSize(
        config_.full_voxel_size,
        config_.full_voxel_size,
        config_.full_voxel_size);
}

bool LocalMap::addKeyframe(const ExtractedFeatures& features, const Pose3D& pose)
{
    std::lock_guard<std::mutex> lock(map_mutex_);

    // Check if we should add this keyframe
    if (has_keyframe_ && !shouldAddKeyframe(pose))
    {
        return false;
    }

    // Create keyframe
    FeatureKeyframe kf;
    kf.features = features;
    kf.pose = pose;
    kf.timestamp = pose.timestamp;

    // Add features to accumulated maps
    addFeaturesToMap(features, pose);

    // Store keyframe
    keyframes_.push_back(kf);

    // Update last keyframe pose
    last_keyframe_pose_ = pose;
    has_keyframe_ = true;

    // Remove old keyframes if exceeded limit
    if (keyframes_.size() > static_cast<size_t>(config_.max_keyframes))
    {
        removeOldKeyframes();
    }

    return true;
}

bool LocalMap::isReady() const
{
    std::lock_guard<std::mutex> lock(map_mutex_);

    // Need at least one keyframe
    if (keyframes_.empty())
    {
        return false;
    }

    // Need minimum number of features
    const size_t min_edge_points = 20;
    const size_t min_planar_points = 50;

    return (edge_map_->size() >= min_edge_points ||
            planar_map_->size() >= min_planar_points);
}

void LocalMap::clear()
{
    std::lock_guard<std::mutex> lock(map_mutex_);

    keyframes_.clear();
    edge_map_->clear();
    planar_map_->clear();
    full_map_->clear();
    has_keyframe_ = false;
}

void LocalMap::addFeaturesToMap(const ExtractedFeatures& features, const Pose3D& pose)
{
    // Transform and add edge features
    if (!features.edge_points->empty())
    {
        auto transformed_edges = transformCloud(features.edge_points, pose);
        *edge_map_ += *transformed_edges;

        // Downsample if too many points
        if (edge_map_->size() > static_cast<size_t>(config_.max_edge_points))
        {
            downsampleCloud(edge_map_, config_.edge_voxel_size);
        }
    }

    // Transform and add planar features
    if (!features.planar_points->empty())
    {
        auto transformed_planars = transformCloud(features.planar_points, pose);
        *planar_map_ += *transformed_planars;

        // Downsample if too many points
        if (planar_map_->size() > static_cast<size_t>(config_.max_planar_points))
        {
            downsampleCloud(planar_map_, config_.planar_voxel_size);
        }
    }

    // Transform and add full cloud
    if (!features.full_cloud->empty())
    {
        auto transformed_full = transformCloud(features.full_cloud, pose);
        *full_map_ += *transformed_full;

        // Downsample if too many points
        if (full_map_->size() > static_cast<size_t>(config_.max_full_points))
        {
            downsampleCloud(full_map_, config_.full_voxel_size);
        }
    }
}

void LocalMap::removeOldKeyframes()
{
    // Remove oldest keyframe
    if (!keyframes_.empty())
    {
        keyframes_.pop_front();
    }

    // Rebuild maps from remaining keyframes
    rebuildMap();
}

void LocalMap::rebuildMap()
{
    // Clear current maps
    edge_map_->clear();
    planar_map_->clear();
    full_map_->clear();

    // Re-add all keyframes
    for (const auto& kf : keyframes_)
    {
        addFeaturesToMap(kf.features, kf.pose);
    }
}

void LocalMap::downsampleCloud(PointCloudPtr& cloud, double voxel_size)
{
    if (cloud->empty())
    {
        return;
    }

    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    filter.setInputCloud(cloud);

    PointCloudPtr filtered(new PointCloud());
    filter.filter(*filtered);

    cloud.swap(filtered);
}

bool LocalMap::shouldAddKeyframe(const Pose3D& pose) const
{
    if (!has_keyframe_)
    {
        return true;
    }

    // Check translation
    double dist = (pose.position - last_keyframe_pose_.position).norm();
    if (dist >= config_.update_distance)
    {
        return true;
    }

    // Check rotation
    Eigen::Quaterniond q_diff = last_keyframe_pose_.orientation.inverse() * pose.orientation;
    double angle = 2.0 * std::acos(std::min(1.0, std::abs(q_diff.w())));
    if (angle >= config_.update_rotation)
    {
        return true;
    }

    return false;
}

LocalMap::PointCloudPtr LocalMap::transformCloud(
    const PointCloudConstPtr& cloud,
    const Pose3D& pose)
{
    PointCloudPtr transformed(new PointCloud());

    // Build transformation matrix
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = pose.orientation.toRotationMatrix().cast<float>();
    transform.block<3, 1>(0, 3) = pose.position.cast<float>();

    pcl::transformPointCloud(*cloud, *transformed, transform);

    return transformed;
}

}  // namespace olive
