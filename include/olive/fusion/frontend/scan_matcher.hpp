/**
 * @file scan_matcher.hpp
 * @brief Scan-to-map registration of edge/planar features via Gauss-Newton
 */

#ifndef OLIVE_FUSION_FRONTEND_SCAN_MATCHER_HPP_
#define OLIVE_FUSION_FRONTEND_SCAN_MATCHER_HPP_

#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>

#include "olive/fusion/fusion_types.hpp"

namespace olive
{

/**
 * @brief 6-DoF pose as (roll, pitch, yaw, x, y, z) used inside the matcher
 */
struct MatcherPose
{
    float roll = 0, pitch = 0, yaw = 0;
    float x = 0, y = 0, z = 0;

    Eigen::Affine3f    affine() const;
    static MatcherPose fromAffine(const Eigen::Affine3f& t);
};

/**
 * @brief Aligns the current scan's features against local edge/planar maps.
 *
 * Edge points are matched to line segments (principal direction of their five
 * nearest map neighbors); planar points to fitted planes. The stacked
 * point-to-line / point-to-plane residuals are minimized by iterative
 * Gauss-Newton steps. On the first iteration the normal matrix is
 * eigen-checked; near-singular directions are projected out of the update so
 * a degenerate scene (long corridor) cannot inject garbage motion.
 */
class ScanMatcher
{
public:
    explicit ScanMatcher(const MatcherConfig& config);

    /// Set the local maps to match against (kd-trees are rebuilt)
    void setTarget(const Cloud::Ptr& edge_map, const Cloud::Ptr& planar_map);

    /**
     * @brief Register @p features against the current target
     * @param[in,out] pose initial guess in, optimized pose out
     * @return true when the optimization produced a usable pose
     */
    bool align(const FeatureClouds& features, MatcherPose& pose);

    /// True when the last align() hit a degenerate geometry direction
    bool isDegenerate() const { return is_degenerate_; }

    /// Eigenvalues of the first-iteration normal matrix (ascending)
    const Eigen::Matrix<float, 6, 1>& constraintEigenvalues() const { return eigenvalues_; }

private:
    void addEdgeResiduals(const Cloud& edge_scan, const Eigen::Affine3f& transform);
    void addPlanarResiduals(const Cloud& planar_scan, const Eigen::Affine3f& transform);
    bool gaussNewtonStep(MatcherPose& pose, int iteration);

    MatcherConfig config_;

    pcl::KdTreeFLANN<CloudPoint> edge_tree_;
    pcl::KdTreeFLANN<CloudPoint> planar_tree_;
    Cloud::Ptr                   edge_map_;
    Cloud::Ptr                   planar_map_;

    // Stacked correspondences for the current iteration: point in scan frame
    // plus the residual direction (xyz) and magnitude (intensity).
    Cloud residual_points_;
    Cloud residual_coeffs_;

    bool                       is_degenerate_ = false;
    Eigen::Matrix<float, 6, 1> eigenvalues_   = Eigen::Matrix<float, 6, 1>::Zero();
    Eigen::Matrix<float, 6, 6> degeneracy_projector_;
};

}  // namespace olive

#endif  // OLIVE_FUSION_FRONTEND_SCAN_MATCHER_HPP_
