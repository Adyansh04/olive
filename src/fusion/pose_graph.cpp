#include "olive/fusion/pose_graph.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/slam/PartialPriorFactor.h>

#include "olive/fusion/marker_anchor_factor.hpp"

namespace olive
{

namespace
{

using gtsam::symbol_shorthand::X;

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
    variances << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8;
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

void PoseGraph::addOdometryFactor(const gtsam::Pose3& relative, const FactorSigmas& sigmas,
                                  bool robust)
{
    const size_t          n     = num_keyframes_ - 1;
    gtsam::SharedNoiseModel noise = toNoiseModel(sigmas);
    if (robust)
        noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(0.1), noise);
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

void PoseGraph::addMarkerAnchor(const gtsam::Point3& measured_in_camera,
                                const gtsam::Point3& marker_in_world,
                                const gtsam::Pose3& base_from_camera, double sigma)
{
    // Robust kernel: one mis-decoded marker must not yank the trajectory.
    const auto noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(0.1),
        gtsam::noiseModel::Isotropic::Sigma(3, sigma));
    pending_factors_.add(MarkerAnchorFactor(
        X(num_keyframes_ - 1), measured_in_camera, marker_in_world, base_from_camera, noise));
    has_global_factor_ = true;
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
            for (int i = 0; i < 5; ++i) isam_.update();
        }
    }
    catch (const std::exception&)
    {
        // Indeterminate/degenerate systems must not kill the node: discard
        // the pending work and roll back to the last committed keyframe.
        pending_factors_.resize(0);
        pending_values_.clear();
        num_keyframes_ = committed_keyframes_;
        return OptimizeResult::FAILED;
    }

    pending_factors_.resize(0);
    pending_values_.clear();
    committed_keyframes_ = num_keyframes_;

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

}  // namespace olive
