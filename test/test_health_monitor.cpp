/**
 * @file test_health_monitor.cpp
 * @brief HealthMonitor: liveness, timeouts, quality overrides
 */

#include <gtest/gtest.h>

#include "olive/fusion/health_monitor.hpp"

namespace
{

olive::HealthMonitor makeMonitor()
{
    olive::HealthMonitor monitor;
    monitor.configure({
        { "lidar", 1.0 },
        { "imu", 0.5 },
        { "markers", 0.0 },  // optional
    });
    return monitor;
}

olive::SensorHealth
    healthOf(const std::vector<olive::HealthMonitor::Status>& all, const std::string& name)
{
    for (const auto& status : all)
        if (status.name == name)
            return status.health;
    return olive::SensorHealth::FAILED;
}

TEST(HealthMonitor, RequiredSensorFailsWithoutData)
{
    auto       monitor = makeMonitor();
    const auto all     = monitor.evaluate(10.0);
    EXPECT_EQ(healthOf(all, "lidar"), olive::SensorHealth::FAILED);
    // Optional sensors report POOR rather than FAILED when absent.
    EXPECT_EQ(healthOf(all, "markers"), olive::SensorHealth::POOR);
}

TEST(HealthMonitor, BeatsKeepSensorsHealthy)
{
    auto monitor = makeMonitor();
    monitor.beat("lidar", 10.0);
    monitor.beat("imu", 10.2);
    EXPECT_EQ(healthOf(monitor.evaluate(10.3), "lidar"), olive::SensorHealth::GOOD);
    EXPECT_EQ(healthOf(monitor.evaluate(10.3), "imu"), olive::SensorHealth::GOOD);
}

TEST(HealthMonitor, TimeoutFailsThenRecovers)
{
    auto monitor = makeMonitor();
    monitor.beat("lidar", 10.0);
    EXPECT_EQ(healthOf(monitor.evaluate(12.0), "lidar"), olive::SensorHealth::FAILED);
    monitor.beat("lidar", 12.1);
    EXPECT_EQ(healthOf(monitor.evaluate(12.2), "lidar"), olive::SensorHealth::GOOD);
}

TEST(HealthMonitor, QualityOverrideClearsOnNextBeat)
{
    auto monitor = makeMonitor();
    monitor.beat("lidar", 10.0);
    monitor.flagQuality("lidar", olive::SensorHealth::DEGRADED, "degenerate geometry");
    EXPECT_EQ(healthOf(monitor.evaluate(10.1), "lidar"), olive::SensorHealth::DEGRADED);

    monitor.beat("lidar", 10.2);
    EXPECT_EQ(healthOf(monitor.evaluate(10.3), "lidar"), olive::SensorHealth::GOOD);
}

TEST(HealthMonitor, TimeoutBeatsQualityOverride)
{
    auto monitor = makeMonitor();
    monitor.beat("lidar", 10.0);
    monitor.flagQuality("lidar", olive::SensorHealth::DEGRADED, "degenerate geometry");
    EXPECT_EQ(healthOf(monitor.evaluate(20.0), "lidar"), olive::SensorHealth::FAILED);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
