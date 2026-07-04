/**
 * @file test_keyframe_map.cpp
 * @brief KeyframeMap: cloud eviction policy, guards, accessors
 */

#include <gtest/gtest.h>

#include "olive/fusion/keyframe_map.hpp"

namespace
{

olive::Cloud::Ptr smallCloud(float x)
{
    olive::Cloud::Ptr cloud(new olive::Cloud);
    for (int i = 0; i < 30; ++i)
    {
        olive::CloudPoint p;
        p.x = x + 0.01F * i;
        p.y = 0.0F;
        p.z = 0.0F;
        cloud->push_back(p);
    }
    return cloud;
}

gtsam::Pose3 poseAt(double x, double y = 0.0)
{
    return gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(x, y, 0.0));
}

olive::KeyframeConfig boundedConfig()
{
    olive::KeyframeConfig config;
    config.recent_window       = 5.0;  // short, so tests can age keyframes out
    config.cloud_voxel         = 1.0;  // one cloud-bearing keyframe per metre cell
    config.max_cloud_keyframes = 0;
    config.search_radius       = 100.0;
    return config;
}

TEST(KeyframeMap, VoxelThinningKeepsOldestPerCell)
{
    olive::KeyframeMap map(boundedConfig());

    // Two keyframes in the SAME voxel cell, then time passes.
    map.add(poseAt(0.0), smallCloud(0.0F), smallCloud(0.0F), 0.0);
    map.add(poseAt(0.2), smallCloud(0.2F), smallCloud(0.2F), 1.0);
    // A later keyframe far away, stamped after the recent window elapsed.
    map.add(poseAt(10.0), smallCloud(10.0F), smallCloud(10.0F), 20.0);

    EXPECT_TRUE(map.hasCloud(0));   // oldest in its cell (and keyframe 0)
    EXPECT_FALSE(map.hasCloud(1));  // same cell, newer -> evicted after aging
    EXPECT_TRUE(map.hasCloud(2));   // recent
    // Poses survive eviction.
    EXPECT_NEAR(map.at(1).pose.translation().x(), 0.2, 1e-9);
}

TEST(KeyframeMap, LowQualityKeyframeNeverClaimsCell)
{
    olive::KeyframeMap map(boundedConfig());

    map.add(poseAt(0.0), smallCloud(0.0F), smallCloud(0.0F), 0.0);
    // Low-quality first pass through a NEW cell...
    map.add(poseAt(3.0), smallCloud(3.0F), smallCloud(3.0F), 1.0, /*low_quality=*/true);
    // ...then a good pass through the same cell, later.
    map.add(poseAt(3.1), smallCloud(3.1F), smallCloud(3.1F), 2.0);
    map.add(poseAt(10.0), smallCloud(10.0F), smallCloud(10.0F), 20.0);

    EXPECT_FALSE(map.hasCloud(1));  // low quality: no claim, aged out
    EXPECT_TRUE(map.hasCloud(2));   // the good pass owns the cell
}

TEST(KeyframeMap, HardCapEvictsLeastRecentlySelected)
{
    olive::KeyframeConfig config = boundedConfig();
    config.cloud_voxel           = 0.0;  // isolate the cap
    config.max_cloud_keyframes   = 2;
    olive::KeyframeMap map(config);

    map.add(poseAt(0.0), smallCloud(0.0F), smallCloud(0.0F), 0.0);
    map.add(poseAt(2.0), smallCloud(2.0F), smallCloud(2.0F), 1.0);

    // Select keyframe 1 recently (bumps last_selected).
    olive::Cloud::Ptr edge_map;
    olive::Cloud::Ptr planar_map;
    map.buildLocalMap(gtsam::Point3(2.0, 0.0, 0.0), 10.0, edge_map, planar_map);

    // Adding a third (aged others out of the recent window) trips the cap.
    map.add(poseAt(8.0), smallCloud(8.0F), smallCloud(8.0F), 20.0);

    EXPECT_EQ(map.cloudCount(), static_cast<size_t>(2));
    EXPECT_TRUE(map.hasCloud(0));  // keyframe 0 is always protected
    EXPECT_TRUE(map.hasCloud(2));  // recent
    EXPECT_FALSE(map.hasCloud(1));
}

TEST(KeyframeMap, BuildLocalMapSkipsEvictedKeyframes)
{
    olive::KeyframeMap map(boundedConfig());

    map.add(poseAt(0.0), smallCloud(0.0F), smallCloud(0.0F), 0.0);
    map.add(poseAt(0.2), smallCloud(0.2F), smallCloud(0.2F), 1.0);
    map.add(poseAt(10.0), smallCloud(10.0F), smallCloud(10.0F), 20.0);  // evicts kf 1

    olive::Cloud::Ptr edge_map;
    olive::Cloud::Ptr planar_map;
    // Must not crash on the evicted keyframe; contributions come from 0 and 2.
    map.buildLocalMap(gtsam::Point3(0.0, 0.0, 0.0), 20.0, edge_map, planar_map);
    EXPECT_GT(edge_map->size(), 0U);
}

TEST(KeyframeMap, UnboundedConfigEvictsNothing)
{
    olive::KeyframeConfig config = boundedConfig();
    config.cloud_voxel           = 0.0;
    config.max_cloud_keyframes   = 0;
    olive::KeyframeMap map(config);

    for (int i = 0; i < 10; ++i)
        map.add(poseAt(0.1 * i), smallCloud(0.1F * i), smallCloud(0.1F * i), 10.0 * i);
    EXPECT_EQ(map.cloudCount(), static_cast<size_t>(10));
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
