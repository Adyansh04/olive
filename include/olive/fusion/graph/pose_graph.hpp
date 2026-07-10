/**
 * @file pose_graph.hpp
 * @brief iSAM2-backed keyframe pose graph for the fusion backend
 */

#ifndef OLIVE_FUSION_GRAPH_POSE_GRAPH_HPP_
#define OLIVE_FUSION_GRAPH_POSE_GRAPH_HPP_

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <cstdint>
#include <optional>
#include <set>
#include <unordered_map>
#include <utility>
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
    void addOdometryFactor(
        const gtsam::Pose3& relative,
        const FactorSigmas& sigmas,
        bool                robust = false);

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
    void
        addLoopFactor(size_t old_index, size_t new_index, const gtsam::Pose3& relative, double sigma);

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

    /**
     * @brief Observe a marker landmark from the newest keyframe (TagSLAM-style)
     *
     * The landmark is a Point3 VARIABLE L(id): its first sighting inserts the
     * value (at the surveyed position when given, else back-projected from
     * the keyframe pose) and every sighting adds a robust binary factor, so
     * consecutive sightings constrain the motion between keyframes — markers
     * act as an odometry source. Surveyed landmarks additionally get a
     * position prior and mark the round as a global correction; free
     * landmarks trigger a correction only when re-observed after a gap
     * (a de-facto loop closure).
     *
     * @param landmark_id         Stable landmark key (decoded WhyCode id, or
     *                            an offset tracking id for undecoded tracks)
     * @param measured_in_camera  Detected marker position, camera frame
     * @param base_from_camera    Fixed extrinsic
     * @param sigma               Isotropic observation sigma (m); robustified
     * @param surveyed_position   Known world position (map frame), if surveyed
     * @param survey_sigma        Prior sigma for surveyed landmarks (m)
     */
    void addMarkerObservation(
        int64_t                             landmark_id,
        const gtsam::Point3&                measured_in_camera,
        const gtsam::Pose3&                 base_from_camera,
        double                              sigma,
        const std::optional<gtsam::Point3>& surveyed_position,
        double                              survey_sigma);

    /// Committed landmark estimates (id -> optimized position), for debug viz
    std::vector<std::pair<int64_t, gtsam::Point3>> landmarks() const;

    /**
     * @brief Install IMU velocity/bias states + priors on the FIRST keyframe
     *
     * Call between addFirstKeyframe() and optimize() to enable tight IMU
     * coupling: V(0) = 0 (stationary init) and B(0) seeded with the measured
     * gyro bias.
     */
    void addImuPriors(
        const gtsam::Vector3& gyro_bias_seed,
        double                velocity_sigma,
        double                accel_bias_sigma,
        double                gyro_bias_sigma);

    /**
     * @brief Chain a CombinedImuFactor onto the newest keyframe
     *
     * Adds V(n-1)/B(n-1) states and the 6-key preintegration factor from the
     * previous keyframe. If the chain was broken (a keyframe without IMU
     * states, e.g. a wheel-paced dropout keyframe), the states are re-seeded
     * with priors instead so the factor never references a missing key.
     * @param pim           Preintegrated measurements over the interval
     * @param planar_guard  Add a soft zero-z velocity prior (planar robots:
     *                      guards the vz / accel-bias-z co-drift)
     */
    void
        addCombinedImuFactor(const gtsam::PreintegratedCombinedMeasurements& pim, bool planar_guard);

    /// Latest committed IMU state (rollback-safe: refreshed only on success)
    gtsam::Vector3               latestVelocity() const { return last_velocity_; }
    gtsam::imuBias::ConstantBias latestBias() const { return last_bias_; }

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
     * On FAILED the pending factors/values are discarded and all bookkeeping
     * (keyframe count, landmark ids, IMU chain, committed velocity/bias)
     * rolls back to the last committed state, so the caller must skip the
     * keyframe entirely (an indeterminate system must not kill the node).
     * Committed queries keep serving the last good state. Note: iSAM2 gives
     * no strong exception guarantee — depending on where the update threw,
     * later rounds may fail too; the node then degrades to scan-matching
     * (published pose stays live), which is the intended failure mode.
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
    size_t                      num_keyframes_       = 0;
    size_t                      committed_keyframes_ = 0;
    bool                        has_global_factor_   = false;

    // Landmark bookkeeping: pending ids merge into committed on a successful
    // optimize and are dropped on FAILED (their values roll back with
    // pending_values_).
    std::set<int64_t>                   landmark_ids_;
    std::set<int64_t>                   pending_landmark_ids_;
    std::unordered_map<int64_t, size_t> landmark_last_seen_kf_;

    // IMU tight-coupling state. imu_chain_tail_ = index of the last COMMITTED
    // keyframe carrying V/B states (-1 = none); the pending variant commits or
    // rolls back with optimize(), mirroring the keyframe count.
    bool                         imu_enabled_            = false;
    long                         imu_chain_tail_         = -1;
    long                         pending_imu_chain_tail_ = -1;
    double                       velocity_sigma_         = 0.1;
    double                       accel_bias_sigma_       = 0.1;
    double                       gyro_bias_sigma_        = 0.01;
    gtsam::Vector3               last_velocity_{ 0.0, 0.0, 0.0 };
    gtsam::imuBias::ConstantBias last_bias_;
};

}  // namespace olive

#endif  // OLIVE_FUSION_GRAPH_POSE_GRAPH_HPP_
