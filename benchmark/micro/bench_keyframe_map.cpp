/**
 * @file bench_keyframe_map.cpp
 * @brief Micro-benchmark for KeyframeMap::buildLocalMap() on a realistic map
 *
 * 80 keyframes along a loop, per-keyframe clouds sized like voxel-filtered
 * scan features. "dump" mode prints a coordinate checksum of the built local
 * map in hexfloat — used to prove an optimized build is bit-identical before
 * it replaces the original.
 */
#include <benchmark/benchmark.h>

#include <cstdio>
#include <random>

#include "olive/fusion/graph/keyframe_map.hpp"

namespace
{

using olive::Cloud;
using olive::CloudPoint;

Cloud::Ptr makeCloud(size_t n, unsigned seed)
{
    Cloud::Ptr                            cloud(new Cloud);
    std::mt19937                          rng(seed);
    std::uniform_real_distribution<float> d(-6.0F, 6.0F);
    std::uniform_real_distribution<float> z(0.0F, 2.0F);
    cloud->reserve(n);
    for (size_t i = 0; i < n; ++i)
        cloud->push_back(CloudPoint{ d(rng), d(rng), z(rng), 0.0F });
    return cloud;
}

/// 80 keyframes along a 14x14 square loop, 0.7 m apart (near the sim's 0.5 m
/// keyframe spacing).
olive::KeyframeMap& map()
{
    static olive::KeyframeMap km = [] {
        olive::KeyframeConfig config;  // defaults = sim config
        olive::KeyframeMap    m(config);
        const double          side = 14.0;
        for (int i = 0; i < 80; ++i)
        {
            const double t    = static_cast<double>(i) / 80.0 * 4.0;
            const int    leg  = static_cast<int>(t);
            const double frac = t - leg;
            double       x = -7.0, y = -7.0;
            if (leg == 0)
                x += side * frac;
            else if (leg == 1)
                x += side, y += side * frac;
            else if (leg == 2)
                x += side * (1.0 - frac), y += side;
            else
                y += side * (1.0 - frac);
            const gtsam::Pose3 pose(gtsam::Rot3::Yaw(t), gtsam::Point3(x, y, 0.0));
            m.add(pose, makeCloud(150, 7000 + i), makeCloud(1500, 9000 + i), static_cast<double>(i));
        }
        return m;
    }();
    return km;
}

void buildOnce(Cloud::Ptr& edge, Cloud::Ptr& planar)
{
    // Query from "now" at the loop end; current_time past the recent window.
    map().buildLocalMap(gtsam::Point3(-7.0, -7.0, 0.0), 100.0, edge, planar);
}

void BM_BuildLocalMap(benchmark::State& state)
{
    Cloud::Ptr edge;
    Cloud::Ptr planar;
    for (auto _ : state)
    {
        buildOnce(edge, planar);
        benchmark::DoNotOptimize(edge);
        benchmark::DoNotOptimize(planar);
    }
}
BENCHMARK(BM_BuildLocalMap)->Unit(benchmark::kMillisecond);

}  // namespace

int main(int argc, char** argv)
{
    if (argc > 1 && std::string(argv[1]) == "dump")
    {
        Cloud::Ptr edge;
        Cloud::Ptr planar;
        buildOnce(edge, planar);
        double sum = 0.0;
        for (const CloudPoint& p : *edge)
            sum += static_cast<double>(p.x) + p.y + p.z;
        for (const CloudPoint& p : *planar)
            sum += static_cast<double>(p.x) + p.y + p.z;
        std::printf(
            "local map: edge=%zu planar=%zu checksum=%a\n",
            edge->size(),
            planar->size(),
            sum);
        return 0;
    }
    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
    return 0;
}
