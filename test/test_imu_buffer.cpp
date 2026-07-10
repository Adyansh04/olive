/**
 * @file test_imu_buffer.cpp
 * @brief ImuBuffer: bias-corrected integration, mounting rotation, stats
 */

#include <gtest/gtest.h>

#include "olive/fusion/inputs/imu_buffer.hpp"

namespace
{

olive::ImuData
    sample(double t, const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel = { 0.0, 0.0, 9.81 })
{
    olive::ImuData data;
    data.timestamp           = t;
    data.angular_velocity    = gyro;
    data.linear_acceleration = accel;
    return data;
}

/// 100 Hz stream over [0, duration] with constant gyro rate
void fill(olive::ImuBuffer& buffer, double duration, const Eigen::Vector3d& rate)
{
    for (double t = 0.0; t <= duration + 1e-9; t += 0.01)
        buffer.push(sample(t, rate));
}

TEST(ImuBuffer, BiasSubtractedAtIntegration)
{
    olive::ImuBuffer      buffer;
    const Eigen::Vector3d bias(0.02, -0.01, 0.05);
    fill(buffer, 1.0, bias);  // stationary robot: measured rate == bias

    // Uncorrected, one second of pure bias integrates to a visible rotation.
    const double uncorrected = Eigen::AngleAxisd(buffer.relativeRotation(0.0, 1.0)).angle();
    EXPECT_NEAR(uncorrected, bias.norm(), 1e-3);

    // With the bias set, the same stored samples integrate to identity.
    buffer.setGyroBias(bias);
    const double corrected = Eigen::AngleAxisd(buffer.relativeRotation(0.0, 1.0)).angle();
    EXPECT_LT(corrected, 1e-6);
}

TEST(ImuBuffer, BiasRecoverableFromWindowStats)
{
    olive::ImuBuffer      buffer;
    const Eigen::Vector3d bias(0.015, 0.0, -0.03);
    fill(buffer, 1.5, bias);

    const auto stats = buffer.windowStats(0.0, 1.5);
    EXPECT_GE(stats.count, static_cast<size_t>(100));
    EXPECT_LT((stats.gyro_mean - bias).norm(), 1e-9);
    EXPECT_LT(stats.gyro_deviation, 1e-9);
    EXPECT_NEAR(stats.accel_mean.norm(), 9.81, 1e-6);
}

TEST(ImuBuffer, MountingRotationMapsRatesToBase)
{
    olive::ImuBuffer buffer;
    // IMU mounted yawed 90 deg: an IMU-frame x rotation is a base-frame y rotation.
    buffer.setMountingRotation(
        Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));
    fill(buffer, 1.0, Eigen::Vector3d(0.1, 0.0, 0.0));

    const Eigen::Quaterniond rotation = buffer.relativeRotation(0.0, 1.0);
    const Eigen::AngleAxisd  aa(rotation);
    EXPECT_NEAR(aa.angle(), 0.1, 1e-3);
    EXPECT_NEAR(std::abs(aa.axis().y()), 1.0, 1e-6);
}

TEST(ImuBuffer, CorrectedIntegrationMatchesTrueMotion)
{
    olive::ImuBuffer      buffer;
    const Eigen::Vector3d bias(0.02, 0.0, 0.01);
    const Eigen::Vector3d true_rate(0.0, 0.0, 0.5);  // 0.5 rad/s yaw
    fill(buffer, 2.0, true_rate + bias);
    buffer.setGyroBias(bias);

    const Eigen::Quaterniond rotation = buffer.relativeRotation(0.0, 2.0);
    EXPECT_NEAR(Eigen::AngleAxisd(rotation).angle(), 1.0, 1e-3);  // 0.5 rad/s * 2 s
}

TEST(ImuBuffer, RateNearReturnsBiasCorrectedRate)
{
    olive::ImuBuffer      buffer;
    const Eigen::Vector3d bias(0.01, 0.0, 0.0);
    fill(buffer, 1.0, Eigen::Vector3d(0.31, 0.0, 0.2));
    buffer.setGyroBias(bias);

    const Eigen::Vector3d rate = buffer.rateNear(0.5);
    EXPECT_NEAR(rate.x(), 0.30, 1e-9);
    EXPECT_NEAR(rate.z(), 0.20, 1e-9);
}

TEST(ImuBuffer, SamplesBetweenReturnsHalfOpenWindow)
{
    olive::ImuBuffer buffer;
    fill(buffer, 1.0, Eigen::Vector3d(0.1, 0.0, 0.0));  // samples at 0.00..1.00

    const auto samples = buffer.samplesBetween(0.20, 0.50);
    ASSERT_FALSE(samples.empty());
    // (t0, t1]: the sample AT t0 is excluded, the one at t1 included.
    EXPECT_GT(samples.front().timestamp, 0.20);
    EXPECT_LE(samples.back().timestamp, 0.50 + 1e-9);
    EXPECT_EQ(samples.size(), static_cast<size_t>(30));
}

TEST(ImuBuffer, SamplesBetweenAppliesMountingRotationToBoth)
{
    olive::ImuBuffer buffer;
    buffer.setMountingRotation(
        Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));
    // IMU-frame x gyro + x accel: both must come out on base-frame y.
    buffer.push(sample(0.1, Eigen::Vector3d(0.3, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 9.81)));

    const auto samples = buffer.samplesBetween(0.0, 0.2);
    ASSERT_EQ(samples.size(), 1u);
    EXPECT_NEAR(samples[0].angular_velocity.y(), 0.3, 1e-9);
    EXPECT_NEAR(samples[0].linear_acceleration.y(), 1.0, 1e-9);
    EXPECT_NEAR(samples[0].linear_acceleration.z(), 9.81, 1e-9);
}

TEST(ImuBuffer, SamplesBetweenIsRawNotBiasCorrected)
{
    olive::ImuBuffer      buffer;
    const Eigen::Vector3d rate(0.05, 0.0, 0.0);
    fill(buffer, 0.5, rate);
    buffer.setGyroBias(rate);  // bias == rate: integration would be identity

    const auto samples = buffer.samplesBetween(0.0, 0.5);
    ASSERT_FALSE(samples.empty());
    // The preintegrator estimates bias itself, so samples must stay raw.
    EXPECT_NEAR(samples.front().angular_velocity.x(), 0.05, 1e-9);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
