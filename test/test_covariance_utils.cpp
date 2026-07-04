/**
 * @file test_covariance_utils.cpp
 * @brief Unit tests for the ROS<->GTSAM covariance permutation and pose conversions
 */

#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>

#include "olive/common/covariance_utils.hpp"
#include "olive/common/gtsam_conversions.hpp"

namespace
{

using olive::covariance_utils::Matrix6d;

Matrix6d labeledCovariance()
{
    // Distinct value per cell so any block mix-up is caught.
    Matrix6d cov;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            cov(i, j) = 10.0 * i + j;
    return cov;
}

TEST(CovarianceUtils, SwapMovesBlocksCorrectly)
{
    const Matrix6d ros_cov   = labeledCovariance();
    const Matrix6d gtsam_cov = olive::covariance_utils::rosToGtsamCovariance(ros_cov);

    // Translation variance block (upper-left in ROS) must land lower-right.
    EXPECT_TRUE((gtsam_cov.block<3, 3>(3, 3).isApprox(ros_cov.block<3, 3>(0, 0))));
    // Rotation variance block (lower-right in ROS) must land upper-left.
    EXPECT_TRUE((gtsam_cov.block<3, 3>(0, 0).isApprox(ros_cov.block<3, 3>(3, 3))));
    // Cross blocks swap places.
    EXPECT_TRUE((gtsam_cov.block<3, 3>(0, 3).isApprox(ros_cov.block<3, 3>(3, 0))));
    EXPECT_TRUE((gtsam_cov.block<3, 3>(3, 0).isApprox(ros_cov.block<3, 3>(0, 3))));
}

TEST(CovarianceUtils, SwapIsInvolution)
{
    const Matrix6d cov        = labeledCovariance();
    const Matrix6d round_trip = olive::covariance_utils::gtsamToRosCovariance(
        olive::covariance_utils::rosToGtsamCovariance(cov));
    EXPECT_TRUE((round_trip.isApprox(cov)));
}

TEST(CovarianceUtils, RosArrayRoundTrip)
{
    const Matrix6d               cov = labeledCovariance();
    const std::array<double, 36> arr = olive::covariance_utils::toRosArray(cov);
    // ROS covariance arrays are row-major.
    EXPECT_DOUBLE_EQ(arr[1], cov(0, 1));
    EXPECT_DOUBLE_EQ(arr[6], cov(1, 0));
    EXPECT_TRUE((olive::covariance_utils::fromRosArray(arr).isApprox(cov)));
}

TEST(GtsamConversions, PoseRoundTrip)
{
    geometry_msgs::msg::Pose ros_pose;
    ros_pose.position.x = 1.5;
    ros_pose.position.y = -2.0;
    ros_pose.position.z = 0.25;
    // 90 degrees about z
    ros_pose.orientation.w = std::sqrt(0.5);
    ros_pose.orientation.z = std::sqrt(0.5);

    const gtsam::Pose3             pose     = olive::gtsam_conversions::toGtsamPose(ros_pose);
    const geometry_msgs::msg::Pose returned = olive::gtsam_conversions::toRosPose(pose);

    EXPECT_NEAR(returned.position.x, ros_pose.position.x, 1e-12);
    EXPECT_NEAR(returned.position.y, ros_pose.position.y, 1e-12);
    EXPECT_NEAR(returned.position.z, ros_pose.position.z, 1e-12);
    EXPECT_NEAR(returned.orientation.w, ros_pose.orientation.w, 1e-12);
    EXPECT_NEAR(returned.orientation.z, ros_pose.orientation.z, 1e-12);
}

TEST(GtsamConversions, ComposeMatchesExpectedFrameMath)
{
    // Drive 1 m forward, turn 90 deg left, drive 1 m forward again:
    // the robot must end at (1, 1) facing +y. Exercises real GTSAM code paths
    // (catches any Eigen ABI mismatch at test time).
    const gtsam::Pose3 step_forward(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0));
    const gtsam::Pose3 turn_left(gtsam::Rot3::Yaw(M_PI_2), gtsam::Point3(0.0, 0.0, 0.0));

    const gtsam::Pose3 end = step_forward * turn_left * step_forward;

    EXPECT_NEAR(end.translation().x(), 1.0, 1e-12);
    EXPECT_NEAR(end.translation().y(), 1.0, 1e-12);
    EXPECT_NEAR(end.rotation().yaw(), M_PI_2, 1e-12);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
