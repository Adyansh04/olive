/**
 * @file test_wheel_odom_buffer.cpp
 * @brief WheelOdomBuffer: interpolation, relative pose, extrapolation slack
 */

#include <gtest/gtest.h>

#include "olive/fusion/wheel_odom_buffer.hpp"

namespace
{

gtsam::Pose3 poseAtX(double x) { return gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(x, 0.0, 0.0)); }

/// 20 Hz samples moving +x at 1 m/s over [0, duration]
void fill(olive::WheelOdomBuffer& buffer, double duration)
{
    for (double t = 0.0; t <= duration + 1e-9; t += 0.05)
        buffer.push(t, poseAtX(t));
}

TEST(WheelOdomBuffer, InterpolatesBetweenSamples)
{
    olive::WheelOdomBuffer buffer;
    fill(buffer, 2.0);

    const auto pose = buffer.poseAt(1.025);  // between two 20 Hz samples
    ASSERT_TRUE(pose.has_value());
    EXPECT_NEAR(pose->translation().x(), 1.025, 1e-6);
}

TEST(WheelOdomBuffer, RelativePoseMatchesMotion)
{
    olive::WheelOdomBuffer buffer;
    fill(buffer, 2.0);

    const auto relative = buffer.relativePose(0.5, 1.5);
    ASSERT_TRUE(relative.has_value());
    EXPECT_NEAR(relative->translation().x(), 1.0, 1e-6);
}

TEST(WheelOdomBuffer, DefaultSlackRejectsLateQueries)
{
    olive::WheelOdomBuffer buffer;
    fill(buffer, 1.0);

    // 0.2 s past the newest sample: outside the 0.05 s default slack.
    EXPECT_FALSE(buffer.poseAt(1.2).has_value());
}

TEST(WheelOdomBuffer, ConfiguredSlackAcceptsLateQueries)
{
    olive::WheelOdomBuffer buffer;
    buffer.setInterpolationSlack(0.3);
    fill(buffer, 1.0);

    // Same query as above now succeeds (clamped to the newest sample).
    const auto pose = buffer.poseAt(1.2);
    ASSERT_TRUE(pose.has_value());
    EXPECT_NEAR(pose->translation().x(), 1.0, 1e-6);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
