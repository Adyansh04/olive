#include "olive/fusion/graph/loop_detector.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

#include <cmath>
#include <limits>

namespace olive
{

namespace
{

/// Append a keyframe's feature clouds, transformed to the map frame
void appendTransformed(const Keyframe& keyframe, Cloud& out)
{
    const Eigen::Affine3f transform(keyframe.pose.matrix().cast<float>());
    Cloud                 world;
    pcl::transformPointCloud(*keyframe.edge, world, transform);
    out += world;
    pcl::transformPointCloud(*keyframe.planar, world, transform);
    out += world;
}

gtsam::Pose3 projectPlanar(const gtsam::Pose3& pose)
{
    return gtsam::Pose3(
        gtsam::Rot3::Yaw(pose.rotation().yaw()),
        gtsam::Point3(pose.translation().x(), pose.translation().y(), 0.0));
}

}  // namespace

LoopDetector::LoopDetector(const LoopDetectorConfig& config)
  : config_(config)
{}

std::optional<size_t> LoopDetector::findCandidate(const KeyframeMap& map, size_t current_index) const
{
    const Keyframe& current = map.at(current_index);

    // A few hundred keyframes at most: a linear scan beats maintaining a
    // second kd-tree, and lets every filter apply in one pass.
    double                best_distance = config_.search_radius;
    std::optional<size_t> best;
    for (size_t i = 0; i < current_index; ++i)
    {
        const Keyframe& old = map.at(i);
        if (current.stamp - old.stamp < config_.min_time_diff)
            break;  // keyframes are time-ordered; the rest are too recent
        if (!map.hasCloud(i) || old.low_quality)
            continue;

        const double distance = (current.pose.translation() - old.pose.translation()).norm();
        if (distance < best_distance)
        {
            best_distance = distance;
            best          = i;
        }
    }
    return best;
}

std::optional<LoopClosure> LoopDetector::detect(const KeyframeMap& map, size_t current_index) const
{
    if (!map.hasCloud(current_index) || map.at(current_index).low_quality)
        return std::nullopt;

    const auto candidate = findCandidate(map, current_index);
    if (!candidate)
        return std::nullopt;

    // Submap: the candidate's cloud-bearing neighborhood at optimized poses.
    const Cloud::Ptr target(new Cloud);
    const size_t lo = *candidate >= static_cast<size_t>(config_.submap_half_width) ?
                          *candidate - config_.submap_half_width :
                          0;
    const size_t hi = std::min(map.size() - 1, *candidate + config_.submap_half_width);
    for (size_t i = lo; i <= hi; ++i)
    {
        // The current trajectory's own recent keyframes must not leak into
        // the target, or ICP aligns the scan against itself.
        if (map.at(current_index).stamp - map.at(i).stamp < config_.min_time_diff)
            continue;
        if (map.hasCloud(i))
            appendTransformed(map.at(i), *target);
    }
    if (target->size() < 100)
        return std::nullopt;

    const Cloud::Ptr           target_filtered(new Cloud);
    pcl::VoxelGrid<CloudPoint> filter;
    filter.setLeafSize(config_.submap_leaf_size, config_.submap_leaf_size, config_.submap_leaf_size);
    filter.setInputCloud(target);
    filter.filter(*target_filtered);

    // Source: the current keyframe's features at its current pose estimate.
    const Cloud::Ptr source(new Cloud);
    appendTransformed(map.at(current_index), *source);

    pcl::IterativeClosestPoint<CloudPoint, CloudPoint> icp;
    icp.setMaxCorrespondenceDistance(2.0 * config_.search_radius);
    icp.setMaximumIterations(60);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setInputSource(source);
    icp.setInputTarget(target_filtered);

    Cloud aligned;
    icp.align(aligned);
    if (!icp.hasConverged() || icp.getFitnessScore() > config_.fitness_threshold)
        return std::nullopt;

    // The ICP transform is the map-frame correction of the current pose.
    const Eigen::Matrix4d correction        = icp.getFinalTransformation().cast<double>();
    const gtsam::Pose3    corrected_current = gtsam::Pose3(correction) * map.at(current_index).pose;

    const double implied_jump =
        (corrected_current.translation() - map.at(current_index).pose.translation()).norm();
    if (implied_jump > config_.max_correction)
        return std::nullopt;  // a wrong loop folds the map; be conservative

    gtsam::Pose3 relative = map.at(*candidate).pose.between(corrected_current);
    if (config_.planar)
    {
        // Every keyframe carries a z/roll/pitch prior pin; a 6-DoF loop
        // relative would fight both endpoints.
        relative = projectPlanar(relative);
    }

    return LoopClosure{ *candidate, relative, icp.getFitnessScore() };
}

}  // namespace olive
