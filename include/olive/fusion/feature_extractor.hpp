/**
 * @file feature_extractor.hpp
 * @brief Curvature-based edge / planar feature extraction from a ScanImage
 */

#ifndef OLIVE_FUSION_FEATURE_EXTRACTOR_HPP_
#define OLIVE_FUSION_FEATURE_EXTRACTOR_HPP_

#include <pcl/filters/voxel_grid.h>

#include <utility>
#include <vector>

#include "olive/fusion/fusion_types.hpp"

namespace olive
{

/**
 * @brief Extracts edge (high curvature) and planar (low curvature) features.
 *
 * Curvature is computed per point from the range profile of its ring. Points
 * on occlusion boundaries and grazing (near-parallel) beams are rejected, and
 * each ring is split into azimuth sectors so features spread evenly around
 * the scan instead of clustering on the strongest structure.
 */
class FeatureExtractor
{
public:
    explicit FeatureExtractor(const FeatureConfig& config);

    /// Extract features from @p scan into @p out (reused between calls)
    void extract(const ScanImage& scan, FeatureClouds& out);

private:
    void computeCurvature(const ScanImage& scan);
    void markUnusablePoints(const ScanImage& scan);
    void selectFeatures(const ScanImage& scan, FeatureClouds& out);

    FeatureConfig config_;

    // Per-point work buffers, reused across scans (hot path: no reallocation
    // once they reach steady-state size).
    std::vector<float>                 curvature_;
    std::vector<char>                  picked_;
    std::vector<char>                  is_edge_;
    std::vector<std::pair<float, int>> sortable_;

    Cloud::Ptr                 ring_planar_;
    Cloud::Ptr                 ring_planar_ds_;
    pcl::VoxelGrid<CloudPoint> planar_filter_;
};

}  // namespace olive

#endif  // OLIVE_FUSION_FEATURE_EXTRACTOR_HPP_
