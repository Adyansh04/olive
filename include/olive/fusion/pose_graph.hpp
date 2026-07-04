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
    void addOdometryFactor(const gtsam::Pose3& relative, const FactorSigmas& sigmas,
                           bool robust = false);

    /**
     * @brief Softly pin z, roll and pitch of the newest keyframe to zero
     * @param sigmas {z (m), roll (rad), pitch (rad)}
     */
    void addPlanarPrior(const std::array<double, 3>& sigmas);

    /**
     * @brief Close a loop: a robust between factor X(old) -> X(new)
     *
     * Marks the round as a global correction so the caller refreshes the
     * keyframe map, exactly like marker anchors.
     */
    void addLoopFactor(size_t old_index, size_t new_index, const gtsam::Pose3& relative,
                       double sigma);

    /**
     * @brief Anchor the newest keyframe to a known marker (position only)
     * @param measured_in_camera  Detected marker position, camera frame
     * @param marker_in_world     Surveyed marker position, map frame
     * @param base_from_camera    Fixed extrinsic
     * @param sigma               Isotropic position sigma (m); robustified
     */
    void addMarkerAnchor(
        const gtsam::Point3& measured_in_camera,
        const gtsam::Point3& marker_in_world,
        const gtsam::Pose3&  base_from_camera,
        double               sigma);

    /// Outcome of one incremental update round
    enum class OptimizeResult
    {
        OK,         ///< factors applied; only the newest pose moved meaningfully
        CORRECTED,  ///< a global factor bent past poses — refresh caches
        FAILED      ///< the update threw; pending work was rolled back
    };

    /**
     * @brief Run the incremental update; call once after adding factors
     *
     * On FAILED the pending factors/values are discarded and the keyframe
     * count rolls back to the last committed state, so the caller must skip
     * the keyframe entirely (an indeterminate system must not kill the node).
     */
    OptimizeResult optimize();

    /**
     * @brief Optimized pose of keyframe @p index
     *
     * Reads the cached full estimate, which is refreshed only when a global
     * correction occurs (matching the KeyframeMap pose-refresh semantics);
     * between corrections, past poses are stale by design. The newest pose
     * is always current via latestPose().
     */
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
    size_t                      num_keyframes_      = 0;
    size_t                      committed_keyframes_ = 0;
    bool                        has_global_factor_  = false;
};

}  // namespace olive

#endif  // OLIVE_FUSION_POSE_GRAPH_HPP_
