/**
 * @file feature_extractor.cpp
 * @brief Implementation of LOAM-style feature extraction
 */

#include "olive/lidar_odom/feature_extractor.hpp"

#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace olive
{

FeatureExtractor::FeatureExtractor(const FeatureExtractionConfig& config)
    : config_(config)
{}

ExtractedFeatures FeatureExtractor::extractUnorganized(
    const PointCloudConstPtr& cloud,
    double timestamp)
{
    ExtractedFeatures features;
    features.timestamp = timestamp;

    if (!cloud || cloud->empty())
    {
        return features;
    }

    // Step 1: Compute curvature for all valid points
    auto points_with_curvature = computeCurvatureUnorganized(cloud);

    if (points_with_curvature.empty())
    {
        return features;
    }

    // Step 2: Sort by curvature (descending for edge selection)
    std::vector<PointWithCurvature> sorted_points = points_with_curvature;
    std::sort(sorted_points.begin(), sorted_points.end(),
        [](const PointWithCurvature& a, const PointWithCurvature& b) {
            return a.curvature > b.curvature;
        });

    // Step 3: Select edge features (high curvature)
    int edge_count = 0;
    int max_edges = config_.max_edge_features_per_line * config_.num_scan_lines;

    for (auto& pt : sorted_points)
    {
        if (!pt.is_valid)
            continue;

        if (pt.curvature > config_.edge_threshold)
        {
            features.edge_points->push_back(pt.point);
            pt.is_valid = false;  // Mark as used
            edge_count++;

            if (edge_count >= max_edges)
                break;
        }
    }

    // Step 4: Re-sort for planar selection (ascending curvature)
    std::sort(sorted_points.begin(), sorted_points.end(),
        [](const PointWithCurvature& a, const PointWithCurvature& b) {
            return a.curvature < b.curvature;
        });

    // Step 5: Select planar features (low curvature)
    int planar_count = 0;
    int max_planars = config_.max_planar_features_per_line * config_.num_scan_lines;

    for (auto& pt : sorted_points)
    {
        if (!pt.is_valid)
            continue;

        if (pt.curvature < config_.planar_threshold)
        {
            features.planar_points->push_back(pt.point);
            pt.is_valid = false;
            planar_count++;

            if (planar_count >= max_planars)
                break;
        }
    }

    // Step 6: Store valid points for full cloud (useful for fallback)
    for (const auto& pt : points_with_curvature)
    {
        features.full_cloud->push_back(pt.point);
    }

    return features;
}

ExtractedFeatures FeatureExtractor::extractOrganized(
    const PointCloudConstPtr& cloud,
    double timestamp)
{
    // For now, treat organized clouds the same as unorganized
    // TODO: Implement proper scan line extraction for real LiDARs
    return extractUnorganized(cloud, timestamp);
}

std::vector<PointWithCurvature> FeatureExtractor::computeCurvatureUnorganized(
    const PointCloudConstPtr& cloud)
{
    std::vector<PointWithCurvature> result;
    result.reserve(cloud->size());

    if (cloud->size() < 20)
    {
        return result;
    }

    // Build KD-tree for neighbor search
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // Number of neighbors for curvature computation
    const int k_neighbors = 2 * config_.curvature_region + 1;
    std::vector<int> indices(k_neighbors);
    std::vector<float> distances(k_neighbors);

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        const auto& pt = (*cloud)[i];

        // Skip invalid points
        if (!isValidPoint(pt))
        {
            continue;
        }

        // Find k nearest neighbors
        int found = kdtree.nearestKSearch(pt, k_neighbors, indices, distances);

        if (found < k_neighbors)
        {
            continue;
        }

        // Compute curvature as deviation from local plane
        // Using LOAM-style curvature: c = ||sum(p_j - p_i)||^2 / (n * ||p_i||^2)
        Eigen::Vector3f diff_sum = Eigen::Vector3f::Zero();
        Eigen::Vector3f pt_vec(pt.x, pt.y, pt.z);

        for (int j = 0; j < found; ++j)
        {
            if (indices[j] == static_cast<int>(i))
                continue;

            const auto& neighbor = (*cloud)[indices[j]];
            Eigen::Vector3f neighbor_vec(neighbor.x, neighbor.y, neighbor.z);
            diff_sum += (neighbor_vec - pt_vec);
        }

        float range_sq = pt_vec.squaredNorm();
        if (range_sq < 1e-6f)
        {
            continue;
        }

        // Curvature formula (normalized by range to be scale-invariant)
        float curvature = diff_sum.squaredNorm() / (static_cast<float>(found - 1) * range_sq);

        PointWithCurvature pwc;
        pwc.point = pt;
        pwc.curvature = curvature;
        pwc.scan_line = 0;  // Unknown for unorganized
        pwc.index_in_line = static_cast<int>(i);
        pwc.is_valid = true;

        result.push_back(pwc);
    }

    return result;
}

std::vector<std::vector<PointWithCurvature>> FeatureExtractor::computeCurvatureOrganized(
    const PointCloudConstPtr& cloud)
{
    // Placeholder for organized point cloud processing
    // This would use the ring/scan line structure of real LiDARs
    std::vector<std::vector<PointWithCurvature>> result;
    return result;
}

void FeatureExtractor::selectFeatures(
    std::vector<PointWithCurvature>& points,
    PointCloudPtr& edge_cloud,
    PointCloudPtr& planar_cloud,
    int max_edges,
    int max_planars)
{
    // This is used for organized extraction with sectors
    // Currently not implemented - using simpler approach
}

void FeatureExtractor::markNeighborsInvalid(
    std::vector<PointWithCurvature>& points,
    int center_idx,
    int radius)
{
    for (int i = std::max(0, center_idx - radius);
         i <= std::min(static_cast<int>(points.size()) - 1, center_idx + radius);
         ++i)
    {
        if (i != center_idx)
        {
            points[i].is_valid = false;
        }
    }
}

bool FeatureExtractor::isValidPoint(const pcl::PointXYZ& pt) const
{
    // Check for NaN
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
    {
        return false;
    }

    // Check range
    float range = computeRange(pt);
    if (range < config_.min_range || range > config_.max_range)
    {
        return false;
    }

    // Filter ground plane points (for ground robots)
    // Points near the ground provide poor constraints and cause Z drift
    if (config_.filter_ground)
    {
        // Point height relative to sensor
        float height = pt.z;
        
        // Check if point is in ground region
        if (height >= config_.ground_height_min && height <= config_.ground_height_max)
        {
            return false;  // Skip ground points
        }
    }

    return true;
}

float FeatureExtractor::computeRange(const pcl::PointXYZ& pt) const
{
    return std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
}

}  // namespace olive
