/**
 * @file test_scan_matcher.cpp
 * @brief Synthetic-geometry test: the matcher must recover a known pose offset
 */

#include <gtest/gtest.h>
#include <pcl/common/transforms.h>

#include "olive/fusion/scan_matcher.hpp"

namespace
{

using olive::Cloud;
using olive::CloudPoint;

CloudPoint makePoint(float x, float y, float z)
{
    CloudPoint p;
    p.x         = x;
    p.y         = y;
    p.z         = z;
    p.intensity = 0.0F;
    return p;
}

/// Two perpendicular walls + floor (planar) and the wall-wall corner (edge)
void buildRoom(Cloud::Ptr& edge_map, Cloud::Ptr& planar_map)
{
    edge_map.reset(new Cloud);
    planar_map.reset(new Cloud);

    for (float a = -4.0F; a <= 4.0F; a += 0.1F)
    {
        for (float z = 0.0F; z <= 2.0F; z += 0.1F)
        {
            planar_map->push_back(makePoint(5.0F, a, z));  // wall x = 5
            planar_map->push_back(makePoint(a, 5.0F, z));  // wall y = 5
        }
        for (float b = -4.0F; b <= 4.0F; b += 0.2F)
            planar_map->push_back(makePoint(a, b, 0.0F));  // floor z = 0
    }
    for (float z = 0.0F; z <= 2.0F; z += 0.02F)
        edge_map->push_back(makePoint(5.0F, 5.0F, z));  // vertical corner line
}

TEST(ScanMatcher, RecoversKnownOffset)
{
    Cloud::Ptr edge_map;
    Cloud::Ptr planar_map;
    buildRoom(edge_map, planar_map);

    // Ground-truth pose of the scan origin in the map frame.
    const Eigen::Affine3f truth =
        pcl::getTransformation(0.15F, -0.10F, 0.02F, 0.01F, -0.01F, 0.06F);

    // The scan sees every map point through the inverse of that pose.
    Cloud::Ptr edge_scan(new Cloud);
    Cloud::Ptr planar_scan(new Cloud);
    pcl::transformPointCloud(*edge_map, *edge_scan, truth.inverse());
    pcl::transformPointCloud(*planar_map, *planar_scan, truth.inverse());

    olive::MatcherConfig config;
    olive::ScanMatcher   matcher(config);
    matcher.setTarget(edge_map, planar_map);

    olive::FeatureClouds features;
    features.edge   = edge_scan;
    features.planar = planar_scan;

    olive::MatcherPose pose;  // identity initial guess
    ASSERT_TRUE(matcher.align(features, pose));

    EXPECT_NEAR(pose.x, 0.15F, 0.02F);
    EXPECT_NEAR(pose.y, -0.10F, 0.02F);
    EXPECT_NEAR(pose.z, 0.02F, 0.02F);
    EXPECT_NEAR(pose.roll, 0.01F, 0.01F);
    EXPECT_NEAR(pose.pitch, -0.01F, 0.01F);
    EXPECT_NEAR(pose.yaw, 0.06F, 0.01F);
}

TEST(ScanMatcher, StaysPutWhenAligned)
{
    Cloud::Ptr edge_map;
    Cloud::Ptr planar_map;
    buildRoom(edge_map, planar_map);

    olive::MatcherConfig config;
    olive::ScanMatcher   matcher(config);
    matcher.setTarget(edge_map, planar_map);

    olive::FeatureClouds features;
    features.edge   = Cloud::Ptr(new Cloud(*edge_map));
    features.planar = Cloud::Ptr(new Cloud(*planar_map));

    olive::MatcherPose pose;
    ASSERT_TRUE(matcher.align(features, pose));

    EXPECT_NEAR(pose.x, 0.0F, 5e-3F);
    EXPECT_NEAR(pose.y, 0.0F, 5e-3F);
    EXPECT_NEAR(pose.z, 0.0F, 5e-3F);
    EXPECT_NEAR(pose.yaw, 0.0F, 5e-3F);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
