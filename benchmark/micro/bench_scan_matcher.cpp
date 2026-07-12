/**
 * @file bench_scan_matcher.cpp
 * @brief Micro-benchmark for ScanMatcher::align() on synthetic room geometry
 *
 * Geometry is sized like a real local map + scan. "dump" mode prints the
 * resulting pose in hexfloat — used to prove an optimized build is
 * bit-identical before it replaces the original (see benchmark/README.md).
 */
#include <benchmark/benchmark.h>
#include <pcl/common/transforms.h>

#include <cstdio>
#include <random>

#include "olive/fusion/frontend/scan_matcher.hpp"

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

/// Two perpendicular walls + floor (planar) and wall corners (edge), sized
/// like a voxel-filtered local map (~20k planar, ~600 edge).
void buildMap(Cloud::Ptr& edge_map, Cloud::Ptr& planar_map)
{
    edge_map.reset(new Cloud);
    planar_map.reset(new Cloud);
    std::mt19937                          rng(20260712);
    std::uniform_real_distribution<float> jitter(-0.005F, 0.005F);

    for (float a = -8.0F; a <= 8.0F; a += 0.08F)
    {
        for (float z = 0.0F; z <= 2.0F; z += 0.08F)
        {
            planar_map->push_back(makePoint(8.0F + jitter(rng), a, z));
            planar_map->push_back(makePoint(a, 8.0F + jitter(rng), z));
            planar_map->push_back(makePoint(-8.0F + jitter(rng), a, z));
        }
        for (float b = -8.0F; b <= 8.0F; b += 0.16F)
            planar_map->push_back(makePoint(a, b, jitter(rng)));
    }
    for (float z = 0.0F; z <= 2.0F; z += 0.01F)
    {
        edge_map->push_back(makePoint(8.0F, 8.0F, z));
        edge_map->push_back(makePoint(-8.0F, 8.0F, z));
    }
}

/// A scan is the map seen through the inverse of the true pose, subsampled to
/// feature-extractor output sizes (~600 planar, ~60 edge).
void buildScan(
    const Cloud::Ptr&      edge_map,
    const Cloud::Ptr&      planar_map,
    const Eigen::Affine3f& truth,
    olive::FeatureClouds&  features)
{
    Cloud::Ptr edge_scan(new Cloud);
    Cloud::Ptr planar_scan(new Cloud);
    pcl::transformPointCloud(*edge_map, *edge_scan, truth.inverse());
    pcl::transformPointCloud(*planar_map, *planar_scan, truth.inverse());
    for (size_t i = 0; i < edge_scan->size(); i += 5)
        features.edge->push_back((*edge_scan)[i]);
    for (size_t i = 0; i < planar_scan->size(); i += 40)
        features.planar->push_back((*planar_scan)[i]);
}

struct Fixture
{
    Cloud::Ptr           edge_map;
    Cloud::Ptr           planar_map;
    olive::FeatureClouds features;
    olive::ScanMatcher   matcher{ olive::MatcherConfig{} };

    Fixture()
    {
        buildMap(edge_map, planar_map);
        const Eigen::Affine3f truth =
            pcl::getTransformation(0.15F, -0.10F, 0.02F, 0.01F, -0.01F, 0.06F);
        buildScan(edge_map, planar_map, truth, features);
        matcher.setTarget(edge_map, planar_map);
    }
};

Fixture& fixture()
{
    static Fixture f;
    return f;
}

olive::MatcherPose alignOnce()
{
    olive::MatcherPose pose;  // zero initial guess; align() converges to truth
    const bool         ok = fixture().matcher.align(fixture().features, pose);
    if (!ok)
        std::abort();  // bench geometry must always match
    return pose;
}

void BM_Align(benchmark::State& state)
{
    for (auto _ : state)
    {
        olive::MatcherPose pose = alignOnce();
        benchmark::DoNotOptimize(pose);
    }
}
BENCHMARK(BM_Align)->Unit(benchmark::kMillisecond);

/// setTarget (kd-tree build) cost, the other half of the hot pair.
void BM_SetTarget(benchmark::State& state)
{
    Fixture& f = fixture();
    for (auto _ : state)
        f.matcher.setTarget(f.edge_map, f.planar_map);
}
BENCHMARK(BM_SetTarget)->Unit(benchmark::kMillisecond);

}  // namespace

int main(int argc, char** argv)
{
    if (argc > 1 && std::string(argv[1]) == "dump")
    {
        const olive::MatcherPose pose = alignOnce();
        std::printf(
            "align pose bits: %a %a %a %a %a %a\n",
            static_cast<double>(pose.x),
            static_cast<double>(pose.y),
            static_cast<double>(pose.z),
            static_cast<double>(pose.roll),
            static_cast<double>(pose.pitch),
            static_cast<double>(pose.yaw));
        return 0;
    }
    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
    return 0;
}
