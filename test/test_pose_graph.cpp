/**
 * @file test_pose_graph.cpp
 * @brief PoseGraph: rollback semantics, landmark bookkeeping, IMU chain
 */

#include <gtest/gtest.h>

#include "olive/fusion/graph/pose_graph.hpp"

namespace
{

const olive::FactorSigmas TIGHT = { 0.01, 0.01, 0.01, 0.001, 0.001, 0.001 };

/// PIM params mirroring the production configuration
gtsam::PreintegratedCombinedMeasurements makePim()
{
    auto params                     = gtsam::PreintegrationCombinedParams::MakeSharedU(9.81);
    params->accelerometerCovariance = gtsam::I_3x3 * 1e-4;
    params->gyroscopeCovariance     = gtsam::I_3x3 * 1e-8;
    params->integrationCovariance   = gtsam::I_3x3 * 1e-8;
    params->biasAccCovariance       = gtsam::I_3x3 * 1e-6;
    params->biasOmegaCovariance     = gtsam::I_3x3 * 1e-7;
    params->biasAccOmegaInt         = gtsam::I_6x6 * 1e-8;
    return gtsam::PreintegratedCombinedMeasurements(params);
}

/// Integrate 1 s of constant body accel (specific force includes +9.81 z)
void integrateInterval(gtsam::PreintegratedCombinedMeasurements& pim, double accel_x)
{
    for (int i = 0; i < 100; ++i)
        pim.integrateMeasurement(gtsam::Vector3(accel_x, 0.0, 9.81), gtsam::Vector3::Zero(), 0.01);
}

TEST(PoseGraph, FailedOptimizeRollsBackKeyframeCount)
{
    olive::PoseGraph graph(0.1, 1);
    graph.addFirstKeyframe(gtsam::Pose3());
    ASSERT_EQ(graph.optimize(), olive::PoseGraph::OptimizeResult::OK);

    graph.addKeyframe(
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
        TIGHT);
    // A loop factor referencing keyframes that do not exist forces the
    // incremental update to throw. The contract: FAILED is returned (the node
    // survives) and the bookkeeping rolls back to the committed state.
    // (Full ISAM2 re-usability after an exception is NOT guaranteed — the
    // caller skips the keyframe; see the optimize() documentation.)
    graph.addLoopFactor(10, 11, gtsam::Pose3(), 0.1);
    EXPECT_EQ(graph.optimize(), olive::PoseGraph::OptimizeResult::FAILED);
    EXPECT_EQ(graph.size(), 1u);  // the pending keyframe rolled back

    // Committed queries still serve the last good state.
    EXPECT_LT(graph.latestPose().translation().norm(), 1e-9);
}

TEST(PoseGraph, LandmarkBookkeepingSurvivesRollback)
{
    olive::PoseGraph graph(0.1, 1);
    graph.addFirstKeyframe(gtsam::Pose3());
    ASSERT_EQ(graph.optimize(), olive::PoseGraph::OptimizeResult::OK);

    const gtsam::Pose3 step(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
    graph.addKeyframe(step, step, TIGHT);
    graph.addMarkerObservation(
        5,
        gtsam::Point3(2.0, 0.5, 0.3),
        gtsam::Pose3(),
        0.1,
        std::nullopt,
        0.05);
    graph.addLoopFactor(10, 11, gtsam::Pose3(), 0.1);  // poison the round
    ASSERT_EQ(graph.optimize(), olive::PoseGraph::OptimizeResult::FAILED);
    // The pending landmark id must NOT leak into the committed set.
    EXPECT_TRUE(graph.landmarks().empty());
}

TEST(PoseGraph, LandmarkCommitsOnSuccess)
{
    olive::PoseGraph graph(0.1, 1);
    graph.addFirstKeyframe(gtsam::Pose3());
    ASSERT_EQ(graph.optimize(), olive::PoseGraph::OptimizeResult::OK);

    const gtsam::Pose3 step(gtsam::Rot3(), gtsam::Point3(1, 0, 0));
    graph.addKeyframe(step, step, TIGHT);
    graph.addMarkerObservation(
        5,
        gtsam::Point3(2.0, 0.5, 0.3),
        gtsam::Pose3(),
        0.1,
        std::nullopt,
        0.05);
    ASSERT_EQ(graph.optimize(), olive::PoseGraph::OptimizeResult::OK);
    ASSERT_EQ(graph.landmarks().size(), 1u);
    EXPECT_EQ(graph.landmarks()[0].first, 5);
    // Back-projected initialization: kf at x=1 observing (2.0, 0.5, 0.3) in
    // the body frame -> world (3.0, 0.5, 0.3).
    EXPECT_LT((graph.landmarks()[0].second - gtsam::Point3(3.0, 0.5, 0.3)).norm(), 1e-6);
}

TEST(PoseGraph, ImuChainRecoversConstantVelocity)
{
    olive::PoseGraph graph(0.1, 1);
    graph.addFirstKeyframe(gtsam::Pose3());
    graph.addImuPriors(gtsam::Vector3::Zero(), 0.1, 0.1, 0.01);
    ASSERT_EQ(graph.optimize(), olive::PoseGraph::OptimizeResult::OK);

    // Interval 1: accelerate 0 -> 1 m/s (Dx = 0.5 m); intervals 2..3: cruise
    // at 1 m/s (Dx = 1.0 m). Keyframe poses match the true kinematics.
    double x = 0.0;
    double v = 0.0;
    for (int k = 0; k < 3; ++k)
    {
        const double accel = k == 0 ? 1.0 : 0.0;
        const double dx    = v * 1.0 + 0.5 * accel;
        x += dx;
        v += accel;

        auto pim = makePim();
        pim.resetIntegrationAndSetBias(graph.latestBias());
        integrateInterval(pim, accel);

        graph.addKeyframe(
            gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(x, 0, 0)),
            gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(dx, 0, 0)),
            TIGHT);
        graph.addCombinedImuFactor(pim, /*planar_guard=*/true);
        ASSERT_EQ(graph.optimize(), olive::PoseGraph::OptimizeResult::OK);
    }

    EXPECT_NEAR(graph.latestVelocity().x(), 1.0, 0.05);
    EXPECT_LT(std::abs(graph.latestVelocity().z()), 0.02);  // planar guard
    EXPECT_LT(graph.latestBias().gyroscope().norm(), 0.01);
}

TEST(PoseGraph, ImuRollbackKeepsCommittedState)
{
    olive::PoseGraph graph(0.1, 1);
    graph.addFirstKeyframe(gtsam::Pose3());
    graph.addImuPriors(gtsam::Vector3(0.0, 0.0, 0.02), 0.1, 0.1, 0.01);
    ASSERT_EQ(graph.optimize(), olive::PoseGraph::OptimizeResult::OK);
    const auto committed_bias = graph.latestBias();

    // A poisoned round with an IMU factor must not corrupt the committed
    // velocity/bias or the chain.
    auto pim = makePim();
    pim.resetIntegrationAndSetBias(committed_bias);
    integrateInterval(pim, 0.0);
    graph.addKeyframe(
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.01, 0, 0)),
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.01, 0, 0)),
        TIGHT);
    graph.addCombinedImuFactor(pim, true);
    graph.addLoopFactor(10, 11, gtsam::Pose3(), 0.1);
    ASSERT_EQ(graph.optimize(), olive::PoseGraph::OptimizeResult::FAILED);
    EXPECT_EQ(graph.size(), 1u);
    // Committed velocity/bias survive the rollback untouched.
    EXPECT_LT((graph.latestBias().vector() - committed_bias.vector()).norm(), 1e-12);
    EXPECT_LT(graph.latestVelocity().norm(), 1e-12);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
