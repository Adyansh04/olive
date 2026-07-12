#include "olive/fusion/graph/keyframe_map.hpp"

#include <algorithm>
#include <cmath>

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
    double              stamp,
    bool                low_quality)
{
    keyframes_.push_back({ pose, edge, planar, stamp, stamp, low_quality });
    const size_t index = keyframes_.size() - 1;

    CloudPoint position;
    position.x         = static_cast<float>(pose.translation().x());
    position.y         = static_cast<float>(pose.translation().y());
    position.z         = static_cast<float>(pose.translation().z());
    position.intensity = static_cast<float>(index);
    keyframe_positions_->push_back(position);

    if (edge)
    {
        cloud_bearing_.push_back(index);
        // The FIRST (oldest) keyframe in a voxel cell claims it: old anchored
        // clouds never churn and keep the time separation loop closure needs.
        // Low-quality keyframes never claim, so a later good pass can. The
        // claimed key is REMEMBERED rather than recomputed from the pose:
        // global corrections (marker anchors, loops) move every pose, and
        // re-keying would orphan all claims and evict the whole map.
        if (config_.cloud_voxel > 0.0 && !low_quality)
        {
            const int64_t key = voxelKey(pose.translation());
            if (voxel_owner_.emplace(key, index).second)
                claimed_key_.emplace(index, key);
        }
    }

    enforceCloudBudget(stamp);
}

int64_t KeyframeMap::voxelKey(const gtsam::Point3& position) const
{
    const auto cell = [&](double v) {
        return static_cast<int64_t>(std::floor(v / config_.cloud_voxel)) & 0x1FFFFF;
    };
    return (cell(position.x()) << 42) | (cell(position.y()) << 21) | cell(position.z());
}

void KeyframeMap::enforceCloudBudget(double now)
{
    if (config_.cloud_voxel <= 0.0 && config_.max_cloud_keyframes == 0)
        return;

    auto evict = [this](size_t index) {
        keyframes_[index].edge.reset();
        keyframes_[index].planar.reset();
        transformed_cache_.erase(index);
    };
    auto is_protected = [&](size_t index) {
        return index == 0 || now - keyframes_[index].stamp <= config_.recent_window;
    };
    auto owns_cell = [&](size_t index) {
        if (config_.cloud_voxel <= 0.0)
            return true;  // voxel thinning disabled: everything "owns"
        return claimed_key_.contains(index);
    };

    // Pass 1: voxel thinning — drop clouds of non-owners once they age out
    // of the recent window.
    std::vector<size_t> kept;
    kept.reserve(cloud_bearing_.size());
    for (const size_t index : cloud_bearing_)
    {
        if (!keyframes_[index].edge)
            continue;  // already evicted elsewhere
        if (!is_protected(index) && !owns_cell(index))
        {
            evict(index);
        }
        else
        {
            kept.push_back(index);
        }
    }
    cloud_bearing_.swap(kept);

    // Pass 2: hard cap as a safety net — least-recently-selected first.
    if (config_.max_cloud_keyframes == 0 || cloud_bearing_.size() <= config_.max_cloud_keyframes)
        return;

    std::vector<size_t> candidates;
    for (const size_t index : cloud_bearing_)
    {
        if (!is_protected(index))
            candidates.push_back(index);
    }
    std::sort(candidates.begin(), candidates.end(), [this](size_t a, size_t b) {
        return keyframes_[a].last_selected < keyframes_[b].last_selected;
    });

    size_t to_evict = cloud_bearing_.size() - config_.max_cloud_keyframes;
    for (const size_t index : candidates)
    {
        if (to_evict == 0)
            break;
        evict(index);
        --to_evict;
    }
    std::erase_if(
        cloud_bearing_, [this](size_t index) { return keyframes_[index].edge == nullptr; });
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
    for (const int index : indices)
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
        if (!keyframes_[i].edge)
            continue;  // clouds evicted; pose-only keyframe
        keyframes_[i].last_selected = current_time;

        auto cached = transformed_cache_.find(i);
        if (cached == transformed_cache_.end())
        {
            const Cloud::Ptr      edge_world(new Cloud);
            const Cloud::Ptr      planar_world(new Cloud);
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
