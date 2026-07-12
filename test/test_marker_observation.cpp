/**
 * @file test_marker_observation.cpp
 * @brief MarkerObservationFactor: Jacobians, anchoring and marker-odometry
 */

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "olive/fusion/graph/marker_observation_factor.hpp"

namespace
{

using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

const gtsam::Pose3
    CAMERA_EXTRINSIC(gtsam::Rot3::Ypr(-M_PI_2, 0.0, -M_PI_2), gtsam::Point3(0.2, 0.0, 0.06));

gtsam::Point3 measure(const gtsam::Pose3& robot, const gtsam::Point3& marker_world)
{
    return robot.compose(CAMERA_EXTRINSIC).transformTo(marker_world);
}

TEST(MarkerObservationFactor, ZeroErrorAtTruth)
{
    const gtsam::Pose3  robot(gtsam::Rot3::Yaw(0.7), gtsam::Point3(1.0, -2.0, 0.0));
    const gtsam::Point3 marker(7.5, 7.5, 1.2);

    olive::MarkerObservationFactor factor(
        X(0),
        L(1),
        measure(robot, marker),
        CAMERA_EXTRINSIC,
        gtsam::noiseModel::Isotropic::Sigma(3, 0.1));

    EXPECT_LT(factor.evaluateError(robot, marker).norm(), 1e-12);
}

TEST(MarkerObservationFactor, JacobiansMatchNumerical)
{
    const gtsam::Pose3  robot(gtsam::Rot3::Yaw(-0.4), gtsam::Point3(-2.0, 3.0, 0.0));
    const gtsam::Point3 marker(-7.5, 7.5, 1.2);

    olive::MarkerObservationFactor factor(
        X(0),
        L(1),
        measure(robot, marker) + gtsam::Point3(0.05, -0.02, 0.01),
        CAMERA_EXTRINSIC,
        gtsam::noiseModel::Isotropic::Sigma(3, 0.1));

    gtsam::Matrix analytic_pose;
    gtsam::Matrix analytic_landmark;
    factor.evaluateError(robot, marker, analytic_pose, analytic_landmark);

    const gtsam::Matrix numeric_pose =
        gtsam::numericalDerivative21<gtsam::Vector, gtsam::Pose3, gtsam::Point3>(
            [&](const gtsam::Pose3& p, const gtsam::Point3& l) {
                return factor.evaluateError(p, l);
            },
            robot,
            marker);
    const gtsam::Matrix numeric_landmark =
        gtsam::numericalDerivative22<gtsam::Vector, gtsam::Pose3, gtsam::Point3>(
            [&](const gtsam::Pose3& p, const gtsam::Point3& l) {
                return factor.evaluateError(p, l);
            },
            robot,
            marker);

    EXPECT_TRUE((analytic_pose - numeric_pose).cwiseAbs().maxCoeff() < 1e-5);
    EXPECT_TRUE((analytic_landmark - numeric_landmark).cwiseAbs().maxCoeff() < 1e-5);
}

TEST(MarkerObservationFactor, SurveyedLandmarkCorrectsDriftedPose)
{
    // Landmark-mode equivalent of the anchor recovery test: the surveyed
    // position enters as a tight prior on L, observations pull X to truth.
    const gtsam::Pose3 truth(gtsam::Rot3::Yaw(0.3), gtsam::Point3(-5.0, -5.5, 0.0));
    const gtsam::Pose3 drifted(gtsam::Rot3::Yaw(0.42), gtsam::Point3(-4.4, -5.9, 0.0));

    const gtsam::Point3 marker_a(-7.5, -7.5, 1.2);
    const gtsam::Point3 marker_b(7.5, -7.5, 1.2);

    gtsam::NonlinearFactorGraph graph;
    gtsam::Vector6              prior_variances;
    prior_variances << 1e-2, 1e-2, M_PI * M_PI, 1e4, 1e4, 1e4;
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        X(0),
        drifted,
        gtsam::noiseModel::Diagonal::Variances(prior_variances)));

    const auto survey_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.05);
    graph.add(gtsam::PriorFactor<gtsam::Point3>(L(1), marker_a, survey_noise));
    graph.add(gtsam::PriorFactor<gtsam::Point3>(L(2), marker_b, survey_noise));

    const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.05);
    graph.add(olive::MarkerObservationFactor(
        X(0),
        L(1),
        measure(truth, marker_a),
        CAMERA_EXTRINSIC,
        noise));
    graph.add(olive::MarkerObservationFactor(
        X(0),
        L(2),
        measure(truth, marker_b),
        CAMERA_EXTRINSIC,
        noise));

    gtsam::Values initial;
    initial.insert(X(0), drifted);
    initial.insert(L(1), marker_a);
    initial.insert(L(2), marker_b);

    const gtsam::Values result    = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
    const gtsam::Pose3  recovered = result.at<gtsam::Pose3>(X(0));

    EXPECT_LT((recovered.translation() - truth.translation()).norm(), 0.03);
    EXPECT_NEAR(recovered.rotation().yaw(), truth.rotation().yaw(), 0.02);
}

TEST(MarkerObservationFactor, UnsurveyedLandmarkActsAsOdometry)
{
    // The marker-odometry effect in miniature: X(0) is pinned, the X(0)-X(1)
    // between factor is nearly uninformative, and the only usable motion
    // information is two sightings of the same FREE landmark (no prior).
    // Two positions observed from two poses fix translation and yaw under a
    // planar pin — the relative pose must be recovered from the marker alone.
    const gtsam::Pose3 pose0;
    const gtsam::Pose3 pose1(gtsam::Rot3::Yaw(0.25), gtsam::Point3(0.8, 0.3, 0.0));

    const gtsam::Point3 marker_a(4.0, 1.0, 1.2);
    const gtsam::Point3 marker_b(3.5, -1.5, 1.2);

    gtsam::NonlinearFactorGraph graph;
    graph.add(
        gtsam::PriorFactor<gtsam::Pose3>(X(0), pose0, gtsam::noiseModel::Isotropic::Sigma(6, 1e-3)));
    // Nearly uninformative between: without the marker the estimate stays put.
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        X(0),
        X(1),
        gtsam::Pose3(),
        gtsam::noiseModel::Isotropic::Sigma(6, 10.0)));
    // Planar pin on X(1) (z, roll, pitch): mirrors the runtime planar prior.
    gtsam::Vector6 planar_sigmas;
    planar_sigmas << 0.01, 0.01, 1e3, 1e3, 1e3, 0.01;
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        X(1),
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)),
        gtsam::noiseModel::Diagonal::Sigmas(planar_sigmas)));

    const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.02);
    graph.add(olive::MarkerObservationFactor(
        X(0),
        L(9),
        measure(pose0, marker_a),
        CAMERA_EXTRINSIC,
        noise));
    graph.add(olive::MarkerObservationFactor(
        X(1),
        L(9),
        measure(pose1, marker_a),
        CAMERA_EXTRINSIC,
        noise));
    graph.add(olive::MarkerObservationFactor(
        X(0),
        L(10),
        measure(pose0, marker_b),
        CAMERA_EXTRINSIC,
        noise));
    graph.add(olive::MarkerObservationFactor(
        X(1),
        L(10),
        measure(pose1, marker_b),
        CAMERA_EXTRINSIC,
        noise));

    gtsam::Values initial;
    initial.insert(X(0), pose0);
    initial.insert(X(1), pose0);  // start at rest: the marker must move it
    initial.insert(L(9), pose0.compose(CAMERA_EXTRINSIC).transformFrom(measure(pose0, marker_a)));
    initial.insert(L(10), pose0.compose(CAMERA_EXTRINSIC).transformFrom(measure(pose0, marker_b)));

    const gtsam::Values result    = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
    const gtsam::Pose3  recovered = result.at<gtsam::Pose3>(X(1));

    EXPECT_LT((recovered.translation() - pose1.translation()).norm(), 0.05);
    EXPECT_NEAR(recovered.rotation().yaw(), pose1.rotation().yaw(), 0.03);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
