#include "olive/fusion/keyframe_map.hpp"

#include <pcl/common/transforms.h>

namespace olive
{

KeyframeMap::KeyframeMap(const KeyframeConfig& config)
  : config_(config)
  , keyframe_positions_(new Cloud)
  , local_edge_(new Cloud)
  , local_planar_(new Cloud)
{
    edge_filter_.setLeafSize(config_.edge_leaf_size, config_.edge_leaf_size, config_.edge_leaf_size);
    planar_filter_.setLeafSize(
        config_.planar_leaf_size,
        config_.planar_leaf_size,
        config_.planar_leaf_size);
}

void KeyframeMap::add(
    const gtsam::Pose3& pose,
    const Cloud::Ptr&   edge,
    const Cloud::Ptr&   planar,
    double              stamp)
{
    keyframes_.push_back({ pose, edge, planar, stamp });

    CloudPoint position;
    position.x         = static_cast<float>(pose.translation().x());
    position.y         = static_cast<float>(pose.translation().y());
    position.z         = static_cast<float>(pose.translation().z());
    position.intensity = static_cast<float>(keyframes_.size() - 1);
    keyframe_positions_->push_back(position);
}

bool KeyframeMap::shouldAddKeyframe(const gtsam::Pose3& pose) const
{
    if (keyframes_.empty())
        return true;

    const gtsam::Pose3   delta = keyframes_.back().pose.between(pose);
    const double         trans = delta.translation().norm();
    const gtsam::Vector3 rpy   = delta.rotation().rpy();

    return trans > config_.keyframe_translation || std::abs(rpy.x()) > config_.keyframe_rotation ||
           std::abs(rpy.y()) > config_.keyframe_rotation ||
           std::abs(rpy.z()) > config_.keyframe_rotation;
}

void KeyframeMap::buildLocalMap(
    const gtsam::Point3& position,
    double               current_time,
    Cloud::Ptr&          edge_map,
    Cloud::Ptr&          planar_map)
{
    local_edge_->clear();
    local_planar_->clear();
    edge_map   = local_edge_;
    planar_map = local_planar_;
    if (keyframes_.empty())
        return;

    // Spatially close keyframes...
    std::vector<int>             indices;
    std::vector<float>           sq_distances;
    pcl::KdTreeFLANN<CloudPoint> position_tree;
    position_tree.setInputCloud(keyframe_positions_);

    CloudPoint query;
    query.x = static_cast<float>(position.x());
    query.y = static_cast<float>(position.y());
    query.z = static_cast<float>(position.z());
    position_tree.radiusSearch(query, config_.search_radius, indices, sq_distances);

    std::vector<char> selected(keyframes_.size(), 0);
    for (int index : indices)
        selected[static_cast<size_t>(index)] = 1;

    // ... plus everything recent, so tight turns keep dense support.
    for (size_t i = keyframes_.size(); i > 0; --i)
    {
        if (current_time - keyframes_[i - 1].stamp > config_.recent_window)
            break;
        selected[i - 1] = 1;
    }

    for (size_t i = 0; i < keyframes_.size(); ++i)
    {
        if (selected[i] == 0)
            continue;

        auto cached = transformed_cache_.find(i);
        if (cached == transformed_cache_.end())
        {
            Cloud::Ptr            edge_world(new Cloud);
            Cloud::Ptr            planar_world(new Cloud);
            const Eigen::Affine3f transform(keyframes_[i].pose.matrix().cast<float>());
            pcl::transformPointCloud(*keyframes_[i].edge, *edge_world, transform);
            pcl::transformPointCloud(*keyframes_[i].planar, *planar_world, transform);
            cached = transformed_cache_.emplace(i, std::make_pair(edge_world, planar_world)).first;
        }

        *local_edge_ += *cached->second.first;
        *local_planar_ += *cached->second.second;
    }

    // Downsample in place to bound matching cost.
    Cloud::Ptr filtered(new Cloud);
    edge_filter_.setInputCloud(local_edge_);
    edge_filter_.filter(*filtered);
    local_edge_.swap(filtered);

    filtered.reset(new Cloud);
    planar_filter_.setInputCloud(local_planar_);
    planar_filter_.filter(*filtered);
    local_planar_.swap(filtered);

    edge_map   = local_edge_;
    planar_map = local_planar_;
}

void KeyframeMap::updatePose(size_t index, const gtsam::Pose3& pose)
{
    if (index >= keyframes_.size())
        return;
    keyframes_[index].pose = pose;

    CloudPoint& position = keyframe_positions_->points[index];
    position.x           = static_cast<float>(pose.translation().x());
    position.y           = static_cast<float>(pose.translation().y());
    position.z           = static_cast<float>(pose.translation().z());
}

void KeyframeMap::invalidateCache() { transformed_cache_.clear(); }

}  // namespace olive
