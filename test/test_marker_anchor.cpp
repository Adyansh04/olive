/**
 * @file test_marker_anchor.cpp
 * @brief MarkerAnchorFactor: Jacobian correctness and pose recovery
 */

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include "olive/fusion/graph/marker_anchor_factor.hpp"

namespace
{

using gtsam::symbol_shorthand::X;

const gtsam::Pose3
    CAMERA_EXTRINSIC(gtsam::Rot3::Ypr(-M_PI_2, 0.0, -M_PI_2), gtsam::Point3(0.2, 0.0, 0.06));

gtsam::Point3 measure(const gtsam::Pose3& robot, const gtsam::Point3& marker_world)
{
    return robot.compose(CAMERA_EXTRINSIC).transformTo(marker_world);
}

TEST(MarkerAnchorFactor, ZeroErrorAtTruth)
{
    const gtsam::Pose3  robot(gtsam::Rot3::Yaw(0.7), gtsam::Point3(1.0, -2.0, 0.0));
    const gtsam::Point3 marker(7.5, 7.5, 1.2);

    olive::MarkerAnchorFactor factor(
        X(0),
        measure(robot, marker),
        marker,
        CAMERA_EXTRINSIC,
        gtsam::noiseModel::Isotropic::Sigma(3, 0.1));

    EXPECT_LT(factor.evaluateError(robot).norm(), 1e-12);
}

TEST(MarkerAnchorFactor, JacobianMatchesNumerical)
{
    const gtsam::Pose3  robot(gtsam::Rot3::Yaw(-0.4), gtsam::Point3(-2.0, 3.0, 0.0));
    const gtsam::Point3 marker(-7.5, 7.5, 1.2);

    olive::MarkerAnchorFactor factor(
        X(0),
        measure(robot, marker) + gtsam::Point3(0.05, -0.02, 0.01),
        marker,
        CAMERA_EXTRINSIC,
        gtsam::noiseModel::Isotropic::Sigma(3, 0.1));

    gtsam::Matrix analytic;
    factor.evaluateError(robot, analytic);

    const gtsam::Matrix numerical = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
        [&](const gtsam::Pose3& p) { return factor.evaluateError(p); },
        robot);

    EXPECT_TRUE((analytic - numerical).cwiseAbs().maxCoeff() < 1e-5);
}

TEST(MarkerAnchorFactor, AnchorsCorrectDriftedPose)
{
    // The robot believes it is at the drifted pose; two marker sightings
    // (synthesized from the true pose) must pull it back.
    const gtsam::Pose3 truth(gtsam::Rot3::Yaw(0.3), gtsam::Point3(-5.0, -5.5, 0.0));
    const gtsam::Pose3 drifted(gtsam::Rot3::Yaw(0.42), gtsam::Point3(-4.4, -5.9, 0.0));

    const gtsam::Point3 marker_a(-7.5, -7.5, 1.2);
    const gtsam::Point3 marker_b(7.5, -7.5, 1.2);

    gtsam::NonlinearFactorGraph graph;
    // Loose prior at the drifted belief (mirrors the loose X(0) prior).
    gtsam::Vector6 prior_variances;
    prior_variances << 1e-2, 1e-2, M_PI * M_PI, 1e4, 1e4, 1e4;
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        X(0),
        drifted,
        gtsam::noiseModel::Diagonal::Variances(prior_variances)));

    const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.05);
    graph.add(
        olive::MarkerAnchorFactor(X(0), measure(truth, marker_a), marker_a, CAMERA_EXTRINSIC, noise));
    graph.add(
        olive::MarkerAnchorFactor(X(0), measure(truth, marker_b), marker_b, CAMERA_EXTRINSIC, noise));

    gtsam::Values initial;
    initial.insert(X(0), drifted);

    const gtsam::Values result    = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
    const gtsam::Pose3  recovered = result.at<gtsam::Pose3>(X(0));

    EXPECT_LT((recovered.translation() - truth.translation()).norm(), 0.02);
    EXPECT_NEAR(recovered.rotation().yaw(), truth.rotation().yaw(), 0.01);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
