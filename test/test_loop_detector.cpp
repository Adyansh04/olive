/**
 * @file test_loop_detector.cpp
 * @brief LoopDetector: candidate selection and ICP relative recovery
 */

#include <gtest/gtest.h>
#include <pcl/common/transforms.h>

#include "olive/fusion/graph/loop_detector.hpp"

namespace
{

/// Feature clouds of a distinctive "room corner" seen from a pose:
/// two perpendicular walls sampled in the BODY frame of the observer.
void roomFeatures(const gtsam::Pose3& observer, olive::Cloud::Ptr& edge, olive::Cloud::Ptr& planar)
{
    edge.reset(new olive::Cloud);
    planar.reset(new olive::Cloud);
    const Eigen::Affine3f world_to_body(observer.inverse().matrix().cast<float>());

    auto push = [&](olive::Cloud& cloud, float wx, float wy, float wz) {
        const Eigen::Vector3f p = world_to_body * Eigen::Vector3f(wx, wy, wz);
        olive::CloudPoint     point;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z();
        cloud.push_back(point);
    };

    for (float a = -3.0F; a <= 3.0F; a += 0.05F)
    {
        for (float z = 0.0F; z <= 1.5F; z += 0.15F)
        {
            push(*planar, 5.0F, a, z);  // wall x = 5
            push(*planar, a, 5.0F, z);  // wall y = 5
        }
    }
    for (float z = 0.0F; z <= 1.5F; z += 0.05F)
        push(*edge, 5.0F, 5.0F, z);
}

olive::KeyframeConfig mapConfig()
{
    olive::KeyframeConfig config;
    config.recent_window = 1000.0;  // no eviction in these tests
    return config;
}

TEST(LoopDetector, ClosesLoopWithKnownDrift)
{
    olive::KeyframeMap map(mapConfig());

    // First pass: keyframes 0..3 near the corner, well-estimated.
    for (int i = 0; i < 4; ++i)
    {
        const gtsam::Pose3 pose(gtsam::Rot3(), gtsam::Point3(0.3 * i, 0.0, 0.0));
        olive::Cloud::Ptr  edge, planar;
        roomFeatures(pose, edge, planar);
        map.add(pose, edge, planar, 10.0 * i);
    }

    // Much later, the robot returns to (0.3, 0, 0) but its ESTIMATE has
    // drifted by (0.4, -0.3, yaw 3 deg): its stored clouds are what the
    // TRUE pose sees, while its stored pose is the drifted estimate.
    const gtsam::Pose3 true_pose(gtsam::Rot3(), gtsam::Point3(0.3, 0.0, 0.0));
    const gtsam::Pose3 drift(gtsam::Rot3::Yaw(0.05), gtsam::Point3(0.4, -0.3, 0.0));
    const gtsam::Pose3 drifted_estimate = drift * true_pose;
    olive::Cloud::Ptr  edge, planar;
    roomFeatures(true_pose, edge, planar);
    map.add(drifted_estimate, edge, planar, 100.0);

    olive::LoopDetectorConfig config;
    config.min_time_diff = 30.0;
    olive::LoopDetector detector(config);

    const auto loop = detector.detect(map, map.size() - 1);
    ASSERT_TRUE(loop.has_value());

    // The corrected current pose implied by the loop must be near the truth:
    // X(old) * relative ~= true_pose.
    const gtsam::Pose3 corrected = map.at(loop->old_index).pose * loop->relative;
    EXPECT_LT((corrected.translation() - true_pose.translation()).norm(), 0.08);
    EXPECT_NEAR(corrected.rotation().yaw(), 0.0, 0.02);
}

TEST(LoopDetector, RejectsRecentAndLowQualityCandidates)
{
    olive::KeyframeMap map(mapConfig());

    const gtsam::Pose3 pose(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0));
    olive::Cloud::Ptr  edge, planar;

    roomFeatures(pose, edge, planar);
    map.add(pose, edge, planar, 0.0, /*low_quality=*/true);  // bad first pass
    roomFeatures(pose, edge, planar);
    map.add(pose, edge, planar, 5.0);  // too recent relative to current
    roomFeatures(pose, edge, planar);
    map.add(pose, edge, planar, 20.0);  // current

    olive::LoopDetectorConfig config;
    config.min_time_diff = 30.0;
    olive::LoopDetector detector(config);

    EXPECT_FALSE(detector.detect(map, map.size() - 1).has_value());
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
