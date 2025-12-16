/**
 * @file feature_extractor.cpp
 * @brief Implementation of curvature-based feature extraction
 *
 * Extracts edge and planar features based on local surface curvature.
 * Edge features constrain rotation, planar features constrain translation.
 */

#include "olive/lidar_odom/feature_extractor.hpp"

#include <pcl/common/centroid.h>

#include <algorithm>
#include <cmath>

namespace olive
{

FeatureExtractor::FeatureExtractor(const FeatureExtractionConfig& config)
  : config_(config)
  , kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>())
{}

ExtractedFeatures
    FeatureExtractor::extractUnorganized(const PointCloudConstPtr& cloud, double timestamp)
{
    ExtractedFeatures features;
    features.timestamp = timestamp;

    if (!cloud || cloud->empty())
    {
        return features;
    }

    // Compute curvature for all valid points
    auto points_with_curvature = computeCurvatureUnorganized(cloud);

    if (points_with_curvature.empty())
    {
        return features;
    }

    // Calculate limits
    const int max_edges = std::min(
        config_.max_edge_features_per_line * config_.num_scan_lines,
        static_cast<int>(points_with_curvature.size()));
    const int max_planars = std::min(
        config_.max_planar_features_per_line * config_.num_scan_lines,
        static_cast<int>(points_with_curvature.size()));

    // PERF: Use partial_sort for top-N edges (high curvature) instead of full sort
    if (max_edges > 0 && max_edges < static_cast<int>(points_with_curvature.size()))
    {
        std::partial_sort(
            points_with_curvature.begin(),
            points_with_curvature.begin() + max_edges,
            points_with_curvature.end(),
            [](const PointWithCurvature& a, const PointWithCurvature& b) {
                return a.curvature > b.curvature;
            });
    }
    else
    {
        std::sort(
            points_with_curvature.begin(),
            points_with_curvature.end(),
            [](const PointWithCurvature& a, const PointWithCurvature& b) {
                return a.curvature > b.curvature;
            });
    }

    // Select edge features (high curvature)
    int edge_count = 0;
    for (int i = 0; i < max_edges && edge_count < max_edges; ++i)
    {
        auto& pt = points_with_curvature[i];
        if (!pt.is_valid)
            continue;

        if (pt.curvature > config_.edge_threshold)
        {
            features.edge_points->push_back(pt.point);
            pt.is_valid = false;  // Mark as used
            ++edge_count;
        }
    }

    // PERF: Use partial_sort for bottom-N planars (low curvature)
    // Sort remaining valid points by ascending curvature
    auto valid_end = std::partition(
        points_with_curvature.begin(),
        points_with_curvature.end(),
        [](const PointWithCurvature& pt) { return pt.is_valid; });

    const int remaining = static_cast<int>(std::distance(points_with_curvature.begin(), valid_end));
    const int sort_count = std::min(max_planars, remaining);

    if (sort_count > 0 && sort_count < remaining)
    {
        std::partial_sort(
            points_with_curvature.begin(),
            points_with_curvature.begin() + sort_count,
            valid_end,
            [](const PointWithCurvature& a, const PointWithCurvature& b) {
                return a.curvature < b.curvature;
            });
    }
    else if (sort_count > 0)
    {
        std::sort(
            points_with_curvature.begin(),
            valid_end,
            [](const PointWithCurvature& a, const PointWithCurvature& b) {
                return a.curvature < b.curvature;
            });
    }

    // Select planar features (low curvature)
    int planar_count = 0;
    for (int i = 0; i < sort_count && planar_count < max_planars; ++i)
    {
        auto& pt = points_with_curvature[i];
        if (!pt.is_valid)
            continue;

        if (pt.curvature < config_.planar_threshold)
        {
            features.planar_points->push_back(pt.point);
            pt.is_valid = false;
            ++planar_count;
        }
    }

    // Store all valid points for fallback GICP
    for (const auto& pt : points_with_curvature)
    {
        if (pt.is_valid || features.full_cloud->size() < 5000)
        {
            features.full_cloud->push_back(pt.point);
        }
    }

    return features;
}

ExtractedFeatures
    FeatureExtractor::extractOrganized(const PointCloudConstPtr& cloud, double timestamp)
{
    // For simulation LiDAR, treat as unorganized
    // Real LiDARs with ring info would use scan line structure
    return extractUnorganized(cloud, timestamp);
}

std::vector<PointWithCurvature>
    FeatureExtractor::computeCurvatureUnorganized(const PointCloudConstPtr& cloud)
{
    std::vector<PointWithCurvature> result;
    result.reserve(cloud->size());

    if (cloud->size() < 20)
    {
        return result;
    }  // Not enough points

    // Build KD-Tree for neighborhood search
    kdtree_->setInputCloud(cloud);

    const int          k_neighbors = 2 * config_.curvature_region + 1;
    std::vector<int>   indices(k_neighbors);
    std::vector<float> distances(k_neighbors);

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        const auto& pt = (*cloud)[i];

        // PERF: Early rejection before KNN search
        if (!isValidPoint(pt))
        {
            continue;
        }

        PointWithCurvature pwc;
        pwc.point    = pt;
        pwc.is_valid = true;

        // Find k nearest neighbors
        int found = kdtree_->nearestKSearch(pt, k_neighbors, indices, distances);

        if (found < k_neighbors)
        {
            pwc.is_valid = false;
            result.push_back(pwc);
            continue;
        }

        // Compute curvature as deviation from local centroid
        // c = ||p - mean(neighbors)|| / mean_dist
        Eigen::Vector3f centroid   = Eigen::Vector3f::Zero();
        float           total_dist = 0.0f;

        for (int j = 0; j < found; ++j)
        {
            const auto& neighbor = (*cloud)[indices[j]];
            centroid += Eigen::Vector3f(neighbor.x, neighbor.y, neighbor.z);
            total_dist += std::sqrt(distances[j]);
        }

        centroid /= static_cast<float>(found);
        float mean_dist = total_dist / static_cast<float>(found);

        if (mean_dist < 1e-6f)
        {
            pwc.curvature = 0.0f;
        }
        else
        {
            Eigen::Vector3f pt_vec(pt.x, pt.y, pt.z);
            float           deviation = (pt_vec - centroid).norm();
            pwc.curvature             = deviation / mean_dist;
        }

        result.push_back(pwc);
    }

    return result;
}

std::vector<std::vector<PointWithCurvature>>
    FeatureExtractor::computeCurvatureOrganized(const PointCloudConstPtr& /*cloud*/)
{
    // TODO: Placeholder for organized point cloud processing
    //  Would use ring/scan line structure of real LiDARs
    return {};
}

void FeatureExtractor::selectFeatures(
    std::vector<PointWithCurvature>& /*points*/,
    PointCloudPtr& /*edge_cloud*/,
    PointCloudPtr& /*planar_cloud*/,
    int /*max_edges*/,
    int /*max_planars*/)
{
    // TODO: Used for organized extraction with sectors - not implemented
}

void FeatureExtractor::markNeighborsInvalid(
    std::vector<PointWithCurvature>& points,
    int                              center_idx,
    int                              radius)
{
    const int start = std::max(0, center_idx - radius);
    const int end   = std::min(static_cast<int>(points.size()) - 1, center_idx + radius);

    for (int i = start; i <= end; ++i)
    {
        if (i != center_idx)
        {
            points[i].is_valid = false;
        }
    }
}

bool FeatureExtractor::isValidPoint(const pcl::PointXYZ& pt) const
{
    // Check for NaN/Inf
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
    {
        return false;
    }

    // Check range bounds
    const float range = computeRange(pt);
    if (range < config_.min_range || range > config_.max_range)
    {
        return false;
    }

    // Ground filtering for ground robots
    if (config_.filter_ground)
    {
        // Points below ground_height_max are likely ground
        if (pt.z < config_.ground_height_max && pt.z > config_.ground_height_min)
        {
            return false;
        }
    }

    return true;
}

float FeatureExtractor::computeRange(const pcl::PointXYZ& pt) const
{
    return std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
}

}  // namespace olive
