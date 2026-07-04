/**
 * @file test_scan_preprocessor.cpp
 * @brief ScanPreprocessor: layouts, time-field encodings, deskew recovery
 */

#include <gtest/gtest.h>

#include <cstring>

#include "olive/fusion/imu_buffer.hpp"
#include "olive/fusion/scan_preprocessor.hpp"

namespace
{

struct RawPoint
{
    float    x, y, z;
    uint16_t ring;
    double   rel_time;  // seconds relative to the header stamp
};

/// Builds a PointCloud2 with x,y,z + ring + one optional time encoding.
/// time_mode: "" (no field), "time" (float32 rel s), "t" (uint32 ns),
/// "timestamp" (float64 absolute).
sensor_msgs::msg::PointCloud2 makeCloud(
    const std::vector<RawPoint>& points,
    uint32_t                     height,
    uint32_t                     width,
    const std::string&           time_mode,
    double                       header_stamp)
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp.sec = static_cast<int32_t>(header_stamp);
    msg.header.stamp.nanosec =
        static_cast<uint32_t>((header_stamp - static_cast<int32_t>(header_stamp)) * 1e9);
    msg.height   = height;
    msg.width    = width;
    msg.is_dense = false;

    auto addField = [&](const std::string& name, uint32_t offset, uint8_t datatype) {
        sensor_msgs::msg::PointField field;
        field.name     = name;
        field.offset   = offset;
        field.datatype = datatype;
        field.count    = 1;
        msg.fields.push_back(field);
    };
    addField("x", 0, sensor_msgs::msg::PointField::FLOAT32);
    addField("y", 4, sensor_msgs::msg::PointField::FLOAT32);
    addField("z", 8, sensor_msgs::msg::PointField::FLOAT32);
    addField("ring", 12, sensor_msgs::msg::PointField::UINT16);
    uint32_t step = 16;
    if (time_mode == "time")
    {
        addField("time", step, sensor_msgs::msg::PointField::FLOAT32);
        step += 4;
    }
    else if (time_mode == "t")
    {
        addField("t", step, sensor_msgs::msg::PointField::UINT32);
        step += 4;
    }
    else if (time_mode == "timestamp")
    {
        addField("timestamp", step, sensor_msgs::msg::PointField::FLOAT64);
        step += 8;
    }
    msg.point_step = step;
    msg.row_step   = step * width;
    msg.data.resize(static_cast<size_t>(step) * height * width, 0);

    // rel_time range for the uint32 't' encoding: ns from frame START
    double t_min = 0.0;
    for (const auto& p : points)
        t_min = std::min(t_min, p.rel_time);

    for (size_t i = 0; i < points.size() && i < static_cast<size_t>(height) * width; ++i)
    {
        uint8_t* base = msg.data.data() + i * step;
        std::memcpy(base + 0, &points[i].x, 4);
        std::memcpy(base + 4, &points[i].y, 4);
        std::memcpy(base + 8, &points[i].z, 4);
        std::memcpy(base + 12, &points[i].ring, 2);
        if (time_mode == "time")
        {
            const float v = static_cast<float>(points[i].rel_time);
            std::memcpy(base + 16, &v, 4);
        }
        else if (time_mode == "t")
        {
            const uint32_t v = static_cast<uint32_t>((points[i].rel_time - t_min) * 1e9);
            std::memcpy(base + 16, &v, 4);
        }
        else if (time_mode == "timestamp")
        {
            const double v = header_stamp + points[i].rel_time;
            std::memcpy(base + 16, &v, 8);
        }
    }
    return msg;
}

/// 4 rings x 90 columns of a cylinder wall at 3 m
std::vector<RawPoint> cylinderScan()
{
    std::vector<RawPoint> points;
    for (uint16_t ring = 0; ring < 4; ++ring)
    {
        for (int col = 0; col < 90; ++col)
        {
            const float azimuth = static_cast<float>(col) / 90.0F * 2.0F * static_cast<float>(M_PI);
            points.push_back({ 3.0F * std::cos(azimuth),
                               3.0F * std::sin(azimuth),
                               0.1F * ring,
                               ring,
                               0.1 * col / 90.0 });
        }
    }
    return points;
}

olive::PreprocessorConfig defaultConfig()
{
    olive::PreprocessorConfig config;
    config.min_range = 0.3F;
    config.max_range = 12.0F;
    return config;
}

TEST(ScanPreprocessor, OrganizedCloudWithoutTimeFieldHasZeroRelTime)
{
    const auto points = cylinderScan();
    const auto msg    = makeCloud(points, 4, 90, "", 100.0);

    olive::ScanPreprocessor preprocessor(defaultConfig());
    olive::ScanImage        scan;
    ASSERT_TRUE(preprocessor.process(msg, scan));
    EXPECT_EQ(scan.points->size(), points.size());
    for (float t : scan.rel_time)
        EXPECT_EQ(t, 0.0F);
}

TEST(ScanPreprocessor, ThreeTimeEncodingsAgree)
{
    const auto points = cylinderScan();

    olive::ScanPreprocessor preprocessor(defaultConfig());
    olive::ScanImage        velodyne, ouster, hesai;
    ASSERT_TRUE(preprocessor.process(makeCloud(points, 4, 90, "time", 100.0), velodyne));
    ASSERT_TRUE(preprocessor.process(makeCloud(points, 4, 90, "t", 100.0), ouster));
    ASSERT_TRUE(preprocessor.process(makeCloud(points, 4, 90, "timestamp", 100.0), hesai));

    ASSERT_EQ(velodyne.rel_time.size(), ouster.rel_time.size());
    ASSERT_EQ(velodyne.rel_time.size(), hesai.rel_time.size());
    for (size_t i = 0; i < velodyne.rel_time.size(); ++i)
    {
        EXPECT_NEAR(velodyne.rel_time[i], ouster.rel_time[i], 1e-5);
        EXPECT_NEAR(velodyne.rel_time[i], hesai.rel_time[i], 1e-4);
    }
    EXPECT_GT(velodyne.rel_time.back(), 0.09F);  // spans the 0.1 s scan
}

TEST(ScanPreprocessor, UnorganizedRingCloudMatchesOrganized)
{
    auto       points        = cylinderScan();
    const auto organized_msg = makeCloud(points, 4, 90, "time", 100.0);

    // Same points as a flat cloud in scrambled order.
    std::vector<RawPoint> shuffled = points;
    std::reverse(shuffled.begin(), shuffled.end());
    const auto flat_msg =
        makeCloud(shuffled, 1, static_cast<uint32_t>(shuffled.size()), "time", 100.0);

    olive::ScanPreprocessor preprocessor(defaultConfig());
    olive::ScanImage        organized, flat;
    ASSERT_TRUE(preprocessor.process(organized_msg, organized));
    ASSERT_TRUE(preprocessor.process(flat_msg, flat));

    ASSERT_EQ(organized.points->size(), flat.points->size());
    ASSERT_EQ(organized.ring_start.size(), flat.ring_start.size());

    // Ring extents must match; within a ring the flat path sorts by azimuth,
    // which for this synthetic scan reproduces the organized order.
    for (size_t ring = 0; ring < organized.ring_start.size(); ++ring)
    {
        EXPECT_EQ(organized.ring_start[ring], flat.ring_start[ring]);
        EXPECT_EQ(organized.ring_end[ring], flat.ring_end[ring]);
    }
    for (size_t i = 0; i < organized.points->size(); ++i)
    {
        EXPECT_NEAR(organized.points->points[i].x, flat.points->points[i].x, 1e-5);
        EXPECT_NEAR(organized.points->points[i].y, flat.points->points[i].y, 1e-5);
        EXPECT_NEAR(organized.rel_time[i], flat.rel_time[i], 1e-5);
    }
}

TEST(ScanPreprocessor, DeskewRecoversSpinDistortedScan)
{
    // The robot spins at 1 rad/s while scanning a cylinder over 0.1 s: each
    // point is rotated by -yaw(t_pt) relative to where a stationary scan
    // would see it. Deskew with the matching gyro stream must restore the
    // stationary geometry.
    const double yaw_rate = 1.0;
    const auto   points   = cylinderScan();

    std::vector<RawPoint> skewed = points;
    for (auto& p : skewed)
    {
        const float yaw = static_cast<float>(-yaw_rate * p.rel_time);
        const float x   = p.x * std::cos(yaw) - p.y * std::sin(yaw);
        const float y   = p.x * std::sin(yaw) + p.y * std::cos(yaw);
        p.x             = x;
        p.y             = y;
    }

    olive::ScanPreprocessor preprocessor(defaultConfig());
    olive::ScanImage        reference, distorted;
    ASSERT_TRUE(preprocessor.process(makeCloud(points, 4, 90, "time", 100.0), reference));
    ASSERT_TRUE(preprocessor.process(makeCloud(skewed, 4, 90, "time", 100.0), distorted));

    // Gyro stream matching the spin.
    olive::ImuBuffer imu;
    for (double t = 99.9; t <= 100.25; t += 0.005)
    {
        olive::ImuData sample;
        sample.timestamp        = t;
        sample.angular_velocity = Eigen::Vector3d(0.0, 0.0, yaw_rate);
        imu.push(sample);
    }

    olive::deskewScan(distorted, imu.sampleRotations(100.0, 100.1, 32), 0.0, 0.1);

    double max_error = 0.0;
    for (size_t i = 0; i < reference.points->size(); ++i)
    {
        max_error = std::max<double>(
            max_error,
            (reference.points->points[i].getVector3fMap() -
             distorted.points->points[i].getVector3fMap())
                .norm());
    }
    // Bin quantization leaves sub-centimeter residual at 3 m range.
    EXPECT_LT(max_error, 0.02);

    // Sanity: without deskew the distortion is order 10x larger.
    // (0.1 s * 1 rad/s * 3 m ~ 0.3 m at the scan end)
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
