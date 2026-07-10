#include "olive/fusion/frontend/feature_extractor.hpp"

#include <algorithm>
#include <cmath>

namespace olive
{

FeatureExtractor::FeatureExtractor(const FeatureConfig& config)
  : config_(config)
  , ring_planar_(new Cloud)
  , ring_planar_ds_(new Cloud)
{
    planar_filter_.setLeafSize(
        config_.planar_leaf_size,
        config_.planar_leaf_size,
        config_.planar_leaf_size);
}

void FeatureExtractor::extract(const ScanImage& scan, FeatureClouds& out)
{
    out.edge->clear();
    out.planar->clear();
    out.stamp = scan.stamp;

    const int window = config_.curvature_window;
    const int n      = static_cast<int>(scan.points->size());
    if (n < 2 * window + 1)
        return;

    curvature_.assign(n, 0.0F);
    picked_.assign(n, 0);
    is_edge_.assign(n, 0);

    computeCurvature(scan);
    markUnusablePoints(scan);
    selectFeatures(scan, out);
}

void FeatureExtractor::computeCurvature(const ScanImage& scan)
{
    const int window = config_.curvature_window;
    const int n      = static_cast<int>(scan.points->size());
    for (int i = window; i < n - window; ++i)
    {
        float diff = -2.0F * static_cast<float>(window) * scan.range[i];
        for (int j = 1; j <= window; ++j)
            diff += scan.range[i - j] + scan.range[i + j];
        curvature_[i] = diff * diff;
    }
}

void FeatureExtractor::markUnusablePoints(const ScanImage& scan)
{
    const int window = config_.curvature_window;
    const int n      = static_cast<int>(scan.points->size());
    for (int i = window; i < n - window - 1; ++i)
    {
        // Occlusion boundaries: a range step between azimuth-adjacent points
        // leaves a shadow edge that must not be picked as a feature.
        const float range_a  = scan.range[i];
        const float range_b  = scan.range[i + 1];
        const int   col_diff = std::abs(scan.column[i + 1] - scan.column[i]);

        if (col_diff < config_.column_gap * 3)
        {
            if (range_a - range_b > config_.occlusion_range_gap)
            {
                for (int j = 0; j <= window; ++j)
                    picked_[i - j] = 1;
            }
            else if (range_b - range_a > config_.occlusion_range_gap)
            {
                for (int j = 1; j <= window + 1; ++j)
                    picked_[i + j] = 1;
            }
        }

        // Grazing beams: both neighbors far away in range means the surface
        // is near-parallel to the beam and its geometry is unstable.
        const float diff_prev = std::abs(scan.range[i - 1] - scan.range[i]);
        const float diff_next = std::abs(scan.range[i + 1] - scan.range[i]);
        if (diff_prev > config_.parallel_beam_ratio * scan.range[i] &&
            diff_next > config_.parallel_beam_ratio * scan.range[i])
            picked_[i] = 1;
    }
}

void FeatureExtractor::selectFeatures(const ScanImage& scan, FeatureClouds& out)
{
    const int window    = config_.curvature_window;
    const int num_rings = static_cast<int>(scan.ring_start.size());

    for (int ring = 0; ring < num_rings; ++ring)
    {
        ring_planar_->clear();

        const int ring_begin = scan.ring_start[ring] + window;
        const int ring_last  = scan.ring_end[ring] - window - 1;
        if (ring_begin >= ring_last)
            continue;

        for (int sector = 0; sector < config_.sectors_per_ring; ++sector)
        {
            const int sp =
                ring_begin + (ring_last - ring_begin) * sector / config_.sectors_per_ring;
            const int ep =
                ring_begin + (ring_last - ring_begin) * (sector + 1) / config_.sectors_per_ring - 1;
            if (sp >= ep)
                continue;

            sortable_.clear();
            for (int i = sp; i <= ep; ++i)
                sortable_.emplace_back(curvature_[i], i);
            std::sort(sortable_.begin(), sortable_.end());

            // Edges: highest curvature first, spaced out by suppressing
            // azimuth-adjacent neighbors after every pick.
            int edges_picked = 0;
            for (int k = static_cast<int>(sortable_.size()) - 1; k >= 0; --k)
            {
                const int ind = sortable_[k].second;
                if (picked_[ind] != 0 || curvature_[ind] <= config_.edge_threshold)
                    continue;
                if (++edges_picked > config_.max_edges_per_sector)
                    break;

                is_edge_[ind] = 1;
                out.edge->push_back(scan.points->points[ind]);

                picked_[ind] = 1;
                for (int l = 1; l <= window; ++l)
                {
                    if (std::abs(scan.column[ind + l] - scan.column[ind + l - 1]) >
                        config_.column_gap)
                        break;
                    picked_[ind + l] = 1;
                }
                for (int l = -1; l >= -window; --l)
                {
                    if (std::abs(scan.column[ind + l] - scan.column[ind + l + 1]) >
                        config_.column_gap)
                        break;
                    picked_[ind + l] = 1;
                }
            }

            // Planar candidates: lowest curvature first, with the same
            // neighbor suppression to spread them out.
            for (size_t k = 0; k < sortable_.size(); ++k)
            {
                const int ind = sortable_[k].second;
                if (picked_[ind] != 0 || curvature_[ind] >= config_.planar_threshold)
                    continue;

                picked_[ind] = 1;
                for (int l = 1; l <= window; ++l)
                {
                    if (std::abs(scan.column[ind + l] - scan.column[ind + l - 1]) >
                        config_.column_gap)
                        break;
                    picked_[ind + l] = 1;
                }
                for (int l = -1; l >= -window; --l)
                {
                    if (std::abs(scan.column[ind + l] - scan.column[ind + l + 1]) >
                        config_.column_gap)
                        break;
                    picked_[ind + l] = 1;
                }
            }

            // Everything not selected as an edge joins the planar pool; the
            // voxel filter below keeps its density manageable.
            for (int i = sp; i <= ep; ++i)
            {
                if (is_edge_[i] == 0)
                    ring_planar_->push_back(scan.points->points[i]);
            }
        }

        ring_planar_ds_->clear();
        planar_filter_.setInputCloud(ring_planar_);
        planar_filter_.filter(*ring_planar_ds_);
        *out.planar += *ring_planar_ds_;
    }
}

}  // namespace olive
