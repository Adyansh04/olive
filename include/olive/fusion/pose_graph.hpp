/**
 * @file pose_graph.hpp
 * @brief iSAM2-backed keyframe pose graph for the fusion backend
 */

#ifndef OLIVE_FUSION_POSE_GRAPH_HPP_
#define OLIVE_FUSION_POSE_GRAPH_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <vector>

namespace olive
{

/**
 * @brief Noise sigmas for a 6-DoF factor, ROS order [x, y, z, roll, pitch, yaw]
 */
using FactorSigmas = std::array<double, 6>;

/**
 * @brief Owns the incremental factor graph over keyframe poses X(0..n).
 *
 * Keyframe i is connected to i-1 by relative-pose (between) factors from the
 * odometry sources. Global anchoring factors (fiducial markers) attach to
 * individual keyframes. All noise sigmas arrive in ROS axis order and are
 * converted to GTSAM tangent order internally.
 */
class PoseGraph
{
public:
    PoseGraph(double relinearize_threshold, int relinearize_skip);

    /// Number of keyframe variables in the graph
    size_t size() const { return num_keyframes_; }

    /**
     * @brief Insert X(0) with a prior at @p pose
     *
     * The prior is loose in translation and yaw so later global anchors can
     * move the trajectory without a fight.
     */
    void addFirstKeyframe(const gtsam::Pose3& pose);

    /**
     * @brief Insert X(n) with a between factor from X(n-1)
     * @param pose      Initial estimate for the new keyframe (scan-matched)
     * @param relative  Measured X(n-1) -> X(n) transform
     * @param sigmas    Factor noise, ROS order
     */
    void addKeyframe(
        const gtsam::Pose3& pose,
        const gtsam::Pose3& relative,
        const FactorSigmas& sigmas);

    /**
     * @brief Add an extra odometry between factor X(n-1) -> X(n) (e.g. wheels)
     *
     * Call after addKeyframe() and before optimize().
     */
    void addOdometryFactor(const gtsam::Pose3& relative, const FactorSigmas& sigmas);

    /**
     * @brief Softly pin z, roll and pitch of the newest keyframe to zero
     * @param sigmas {z (m), roll (rad), pitch (rad)}
     */
    void addPlanarPrior(const std::array<double, 3>& sigmas);

    /// Run the incremental update; call once after adding factors for a keyframe
    void optimize();

    /// Latest optimized pose of keyframe @p index
    gtsam::Pose3 pose(size_t index) const;

    /// Latest optimized pose of the newest keyframe
    gtsam::Pose3 latestPose() const;

    /// Marginal covariance of the newest keyframe (GTSAM tangent order)
    gtsam::Matrix latestCovariance() const;

    /// Copy every optimized pose out (for keyframe-map refresh)
    std::vector<gtsam::Pose3> allPoses() const;

private:
    gtsam::ISAM2                isam_;
    gtsam::NonlinearFactorGraph pending_factors_;
    gtsam::Values               pending_values_;
    gtsam::Values               current_estimate_;
    size_t                      num_keyframes_ = 0;
};

}  // namespace olive

#endif  // OLIVE_FUSION_POSE_GRAPH_HPP_
