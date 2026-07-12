#include "olive/fusion/graph/pose_graph.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/slam/PartialPriorFactor.h>

#include <numbers>

#include "olive/fusion/graph/marker_anchor_factor.hpp"
#include "olive/fusion/graph/marker_observation_factor.hpp"

namespace olive
{

namespace
{

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

/// A free landmark re-observed after this many keyframes acts like a loop
/// closure: past poses will move, so the round must run the correction path.
constexpr size_t LANDMARK_REVISIT_GAP = 10;

/// ROS-order sigmas -> GTSAM tangent-order ([rot, trans]) diagonal noise
gtsam::noiseModel::Diagonal::shared_ptr toNoiseModel(const FactorSigmas& sigmas)
{
    gtsam::Vector6 v;
    v << sigmas[3], sigmas[4], sigmas[5], sigmas[0], sigmas[1], sigmas[2];
    return gtsam::noiseModel::Diagonal::Sigmas(v);
}

}  // namespace

PoseGraph::PoseGraph(double relinearize_threshold, int relinearize_skip)
  : isam_([&] {
      gtsam::ISAM2Params params;
      params.relinearizeThreshold = relinearize_threshold;
      params.relinearizeSkip      = relinearize_skip;
      return params;
  }())
{}

void PoseGraph::addFirstKeyframe(const gtsam::Pose3& pose)
{
    // [rot, trans] variances: firm roll/pitch, free yaw, very loose position
    // so a global anchor can later pull the whole trajectory.
    gtsam::Vector6 variances;
    variances << 1e-2, 1e-2, std::numbers::pi * std::numbers::pi, 1e8, 1e8, 1e8;
    const auto prior_noise = gtsam::noiseModel::Diagonal::Variances(variances);

    pending_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), pose, prior_noise));
    pending_values_.insert(X(0), pose);
    num_keyframes_ = 1;
}

void PoseGraph::addKeyframe(
    const gtsam::Pose3& pose,
    const gtsam::Pose3& relative,
    const FactorSigmas& sigmas)
{
    const size_t n = num_keyframes_;
    pending_factors_.add(
        gtsam::BetweenFactor<gtsam::Pose3>(X(n - 1), X(n), relative, toNoiseModel(sigmas)));
    pending_values_.insert(X(n), pose);
    ++num_keyframes_;
}

void PoseGraph::addOdometryFactor(
    const gtsam::Pose3& relative,
    const FactorSigmas& sigmas,
    bool                robust)
{
    const size_t            n     = num_keyframes_ - 1;
    gtsam::SharedNoiseModel noise = toNoiseModel(sigmas);
    if (robust)
    {
        noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(0.1),
            noise);
    }
    pending_factors_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(n - 1), X(n), relative, noise));
}

void PoseGraph::addPlanarPrior(const std::array<double, 3>& sigmas)
{
    // Pose3 tangent order is [roll, pitch, yaw, x, y, z].
    const std::vector<size_t> indices = { 0, 1, 5 };
    gtsam::Vector3            measured;
    measured << 0.0, 0.0, 0.0;

    const auto noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector3() << sigmas[1], sigmas[2], sigmas[0]).finished());

    pending_factors_.add(
        gtsam::PartialPriorFactor<gtsam::Pose3>(X(num_keyframes_ - 1), indices, measured, noise));
}

void PoseGraph::addLoopFactor(
    size_t              old_index,
    size_t              new_index,
    const gtsam::Pose3& relative,
    double              sigma)
{
    // Robust kernel: a surviving false loop must bend, not fold, the map.
    gtsam::Vector6 sigmas;
    sigmas << sigma, sigma, sigma, sigma, sigma, sigma;
    const auto noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(0.1),
        gtsam::noiseModel::Diagonal::Sigmas(sigmas));
    pending_factors_.add(
        gtsam::BetweenFactor<gtsam::Pose3>(X(old_index), X(new_index), relative, noise));
    has_global_factor_ = true;
}

void PoseGraph::addMarkerAnchor(
    const gtsam::Point3& measured_in_camera,
    const gtsam::Point3& marker_in_world,
    const gtsam::Pose3&  base_from_camera,
    double               sigma)
{
    // Robust kernel: one mis-decoded marker must not yank the trajectory.
    const auto noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(0.1),
        gtsam::noiseModel::Isotropic::Sigma(3, sigma));
    pending_factors_.add(MarkerAnchorFactor(
        X(num_keyframes_ - 1),
        measured_in_camera,
        marker_in_world,
        base_from_camera,
        noise));
    has_global_factor_ = true;
}

void PoseGraph::addMarkerObservation(
    int64_t                             landmark_id,
    const gtsam::Point3&                measured_in_camera,
    const gtsam::Pose3&                 base_from_camera,
    double                              sigma,
    const std::optional<gtsam::Point3>& surveyed_position,
    double                              survey_sigma)
{
    const size_t current_kf = num_keyframes_ - 1;
    const auto   key_index  = static_cast<uint64_t>(landmark_id);

    const bool known =
        landmark_ids_.contains(landmark_id) || pending_landmark_ids_.contains(landmark_id);
    if (!known)
    {
        // First sighting: insert the landmark value. The surveyed position is
        // the best linearization point when available; otherwise back-project
        // the measurement through the (pending) keyframe pose.
        gtsam::Point3 initial = surveyed_position.value_or(gtsam::Point3(0, 0, 0));
        if (!surveyed_position)
        {
            gtsam::Pose3 kf_pose;
            if (pending_values_.exists(X(current_kf)))
            {
                kf_pose = pending_values_.at<gtsam::Pose3>(X(current_kf));
            }
            else if (current_estimate_.exists(X(current_kf)))
            {
                kf_pose = current_estimate_.at<gtsam::Pose3>(X(current_kf));
            }
            initial = kf_pose.compose(base_from_camera).transformFrom(measured_in_camera);
        }
        pending_values_.insert(L(key_index), initial);
        pending_landmark_ids_.insert(landmark_id);

        if (surveyed_position)
        {
            pending_factors_.add(gtsam::PriorFactor<gtsam::Point3>(
                L(key_index),
                *surveyed_position,
                gtsam::noiseModel::Isotropic::Sigma(3, survey_sigma)));
        }
    }

    // Robust kernel: one mis-decoded marker must not yank the trajectory.
    const auto noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(0.1),
        gtsam::noiseModel::Isotropic::Sigma(3, sigma));
    pending_factors_.add(MarkerObservationFactor(
        X(current_kf),
        L(key_index),
        measured_in_camera,
        base_from_camera,
        noise));

    // Correction policy: surveyed landmarks anchor the trajectory (their
    // prior can bend the past); a free landmark only bends the past when it
    // is re-observed after a gap — consecutive sightings act as odometry and
    // stay on the cheap incremental path.
    const auto last_seen = landmark_last_seen_kf_.find(landmark_id);
    if (surveyed_position)
    {
        has_global_factor_ = true;
    }
    else if (
        last_seen != landmark_last_seen_kf_.end() &&
        current_kf - last_seen->second >= LANDMARK_REVISIT_GAP)
    {
        has_global_factor_ = true;
    }
    // (Not rolled back on FAILED: a stale entry only shifts one revisit-gap
    // decision by a keyframe, which is harmless.)
    landmark_last_seen_kf_[landmark_id] = current_kf;
}

void PoseGraph::addImuPriors(
    const gtsam::Vector3& gyro_bias_seed,
    double                velocity_sigma,
    double                accel_bias_sigma,
    double                gyro_bias_sigma)
{
    imu_enabled_      = true;
    velocity_sigma_   = velocity_sigma;
    accel_bias_sigma_ = accel_bias_sigma;
    gyro_bias_sigma_  = gyro_bias_sigma;

    // Stationary start: zero velocity, measured gyro bias, zero accel bias.
    last_velocity_ = gtsam::Vector3::Zero();
    last_bias_     = gtsam::imuBias::ConstantBias(gtsam::Vector3::Zero(), gyro_bias_seed);

    pending_values_.insert(V(0), last_velocity_);
    pending_values_.insert(B(0), last_bias_);
    pending_factors_.add(gtsam::PriorFactor<gtsam::Vector3>(
        V(0),
        last_velocity_,
        gtsam::noiseModel::Isotropic::Sigma(3, velocity_sigma_)));
    gtsam::Vector6 bias_sigmas;
    bias_sigmas << accel_bias_sigma_, accel_bias_sigma_, accel_bias_sigma_, gyro_bias_sigma_,
        gyro_bias_sigma_, gyro_bias_sigma_;
    pending_factors_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        B(0),
        last_bias_,
        gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas)));
    pending_imu_chain_tail_ = 0;
}

void PoseGraph::addCombinedImuFactor(
    const gtsam::PreintegratedCombinedMeasurements& pim,
    bool                                            planar_guard)
{
    const size_t n = num_keyframes_ - 1;

    // Initial estimates: velocity carried forward (the factor corrects it),
    // bias carried forward (its random walk is inside the combined factor).
    pending_values_.insert(V(n), last_velocity_);
    pending_values_.insert(B(n), last_bias_);

    const long chained_from =
        pending_imu_chain_tail_ >= 0 ? pending_imu_chain_tail_ : imu_chain_tail_;
    if (chained_from == static_cast<long>(n) - 1)
    {
        pending_factors_.add(
            gtsam::CombinedImuFactor(X(n - 1), V(n - 1), X(n), V(n), B(n - 1), B(n), pim));
    }
    else
    {
        // Chain broken (interval cap across a stationary pause, or a
        // keyframe without IMU states): re-seed with priors so the following
        // factor never references a missing key. The committed velocity is
        // STALE here (it predates the break), so its prior must be loose — a
        // seed, not a measurement. The bias varies slowly; the committed
        // value stays trustworthy.
        pending_factors_.add(gtsam::PriorFactor<gtsam::Vector3>(
            V(n),
            last_velocity_,
            gtsam::noiseModel::Isotropic::Sigma(3, 1.0)));
        gtsam::Vector6 bias_sigmas;
        bias_sigmas << accel_bias_sigma_, accel_bias_sigma_, accel_bias_sigma_, gyro_bias_sigma_,
            gyro_bias_sigma_, gyro_bias_sigma_;
        pending_factors_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
            B(n),
            last_bias_,
            gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas)));
    }

    if (planar_guard)
    {
        // z-only velocity pin: guards the slow vz / accel-bias-z co-drift a
        // planar pose prior alone cannot observe. x/y stay unconstrained.
        pending_factors_.add(gtsam::PriorFactor<gtsam::Vector3>(
            V(n),
            gtsam::Vector3::Zero(),
            gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(5.0, 5.0, 0.05))));
    }

    pending_imu_chain_tail_ = static_cast<long>(n);
}

PoseGraph::OptimizeResult PoseGraph::optimize()
{
    const bool corrected = has_global_factor_;
    has_global_factor_   = false;

    try
    {
        isam_.update(pending_factors_, pending_values_);
        isam_.update();
        if (corrected)
        {
            // An anchor bends the whole recent trajectory; extra
            // relinearization rounds let the correction propagate.
            for (int i = 0; i < 5; ++i)
                isam_.update();
        }
    }
    catch (const std::exception&)
    {
        // Indeterminate/degenerate systems must not kill the node: discard
        // the pending work and roll back to the last committed keyframe.
        pending_factors_.resize(0);
        pending_values_.clear();
        pending_landmark_ids_.clear();
        pending_imu_chain_tail_ = -1;
        num_keyframes_          = committed_keyframes_;
        return OptimizeResult::FAILED;
    }

    pending_factors_.resize(0);
    pending_values_.clear();
    landmark_ids_.merge(pending_landmark_ids_);
    pending_landmark_ids_.clear();
    committed_keyframes_ = num_keyframes_;
    if (pending_imu_chain_tail_ >= 0)
        imu_chain_tail_ = pending_imu_chain_tail_;
    pending_imu_chain_tail_ = -1;

    // Committed IMU state (rollback-safe: only refreshed here on success).
    if (imu_enabled_ && imu_chain_tail_ >= 0)
    {
        last_velocity_ =
            isam_.calculateEstimate<gtsam::Vector3>(V(static_cast<uint64_t>(imu_chain_tail_)));
        last_bias_ = isam_.calculateEstimate<gtsam::imuBias::ConstantBias>(
            B(static_cast<uint64_t>(imu_chain_tail_)));
    }

    // The full estimate is only needed when past poses moved; the common
    // path stays incremental (see pose() staleness contract).
    if (corrected || current_estimate_.empty())
        current_estimate_ = isam_.calculateBestEstimate();

    return corrected ? OptimizeResult::CORRECTED : OptimizeResult::OK;
}

gtsam::Pose3 PoseGraph::pose(size_t index) const
{
    return current_estimate_.at<gtsam::Pose3>(X(index));
}

gtsam::Pose3 PoseGraph::latestPose() const
{
    // Single-key query: retracts by the same delta the full estimate uses,
    // without materializing every pose.
    return isam_.calculateEstimate<gtsam::Pose3>(X(num_keyframes_ - 1));
}

gtsam::Matrix PoseGraph::latestCovariance() const
{
    return isam_.marginalCovariance(X(num_keyframes_ - 1));
}

std::vector<gtsam::Pose3> PoseGraph::allPoses() const
{
    std::vector<gtsam::Pose3> poses;
    poses.reserve(num_keyframes_);
    for (size_t i = 0; i < num_keyframes_; ++i)
        poses.push_back(pose(i));
    return poses;
}

std::vector<std::pair<int64_t, gtsam::Point3>> PoseGraph::landmarks() const
{
    // Committed landmarks only (a handful of single-key queries; called at
    // keyframe rate for debug visualization, never on the scan hot path).
    std::vector<std::pair<int64_t, gtsam::Point3>> result;
    result.reserve(landmark_ids_.size());
    for (const int64_t id : landmark_ids_)
    {
        result.emplace_back(
            id,
            isam_.calculateEstimate<gtsam::Point3>(L(static_cast<uint64_t>(id))));
    }
    return result;
}

}  // namespace olive
