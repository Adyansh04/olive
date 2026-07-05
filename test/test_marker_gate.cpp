/**
 * @file test_marker_gate.cpp
 * @brief MarkerGate: acceptance classes, streaks, keying and dedupe
 */

#include <gtest/gtest.h>

#include "olive/fusion/marker_gate.hpp"

namespace
{

olive::MarkerGateConfig baseConfig()
{
    olive::MarkerGateConfig config;
    config.known_ids        = { 1, 2 };
    config.min_range        = 0.5;
    config.max_range        = 6.0;
    config.min_track_frames = 3;
    return config;
}

const gtsam::Point3 IN_RANGE(3.0, 0.5, 1.0);

/// Push the same detection for `frames` consecutive frames; returns the
/// number of accepted pushes.
int pushStreak(
    olive::MarkerGate& gate,
    int                whycode_id,
    int                tracking_id,
    bool               id_valid,
    int                frames,
    double             t0 = 0.0)
{
    int accepted = 0;
    for (int i = 0; i < frames; ++i)
        accepted += gate.push(t0 + 0.033 * i, whycode_id, tracking_id, id_valid, IN_RANGE) ? 1 : 0;
    return accepted;
}

TEST(MarkerGate, SurveyedIdPassesAfterStreak)
{
    olive::MarkerGate gate(baseConfig());
    EXPECT_EQ(pushStreak(gate, 1, 10, true, 4), 2);  // frames 3 and 4 pass

    const auto obs = gate.collectNear(0.05, 0.2);
    ASSERT_FALSE(obs.empty());
    EXPECT_EQ(obs.front().marker_id, 1);
    EXPECT_EQ(obs.front().landmark_key_id, 1);
    EXPECT_TRUE(obs.front().decoded);
}

TEST(MarkerGate, UnknownAndUndecodedRejectedByDefault)
{
    olive::MarkerGate gate(baseConfig());
    EXPECT_EQ(pushStreak(gate, 7, 11, true, 5), 0);   // decoded but unsurveyed
    EXPECT_EQ(pushStreak(gate, 1, 12, false, 5), 0);  // id bits invalid
}

TEST(MarkerGate, UnknownIdBecomesFreeLandmarkWhenAccepted)
{
    auto config               = baseConfig();
    config.accept_unknown_ids = true;
    olive::MarkerGate gate(config);

    EXPECT_GT(pushStreak(gate, 7, 11, true, 4), 0);
    const auto obs = gate.collectNear(0.05, 0.2);
    ASSERT_FALSE(obs.empty());
    EXPECT_EQ(obs.front().landmark_key_id, 7);
    EXPECT_TRUE(obs.front().decoded);
}

TEST(MarkerGate, UndecodedTrackKeyedByTrackingId)
{
    auto config                 = baseConfig();
    config.accept_undecoded_ids = true;
    olive::MarkerGate gate(config);

    EXPECT_GT(pushStreak(gate, 1, 42, false, 4), 0);
    const auto obs = gate.collectNear(0.05, 0.2);
    ASSERT_FALSE(obs.empty());
    EXPECT_EQ(obs.front().landmark_key_id, olive::UNDECODED_LANDMARK_BASE + 42);
    EXPECT_EQ(obs.front().marker_id, -1);
    EXPECT_FALSE(obs.front().decoded);
}

TEST(MarkerGate, RangeFailureErasesStreak)
{
    olive::MarkerGate gate(baseConfig());
    pushStreak(gate, 1, 10, true, 2);  // streak at 2, nothing accepted yet
    // Out-of-range detection kills the streak...
    EXPECT_FALSE(gate.push(0.07, 1, 10, true, gtsam::Point3(9.0, 0.0, 1.0)));
    // ...so the next two pushes are still below min_track_frames.
    EXPECT_EQ(pushStreak(gate, 1, 10, true, 2, 0.1), 0);
}

TEST(MarkerGate, CollectDedupesPerLandmark)
{
    auto config                 = baseConfig();
    config.accept_undecoded_ids = true;
    olive::MarkerGate gate(config);

    pushStreak(gate, 1, 10, true, 5);          // surveyed id 1
    pushStreak(gate, 1, 42, false, 5, 0.001);  // undecoded track 42

    const auto obs = gate.collectNear(0.1, 0.3);
    ASSERT_EQ(obs.size(), 2u);  // one per landmark key despite repeated pushes
    EXPECT_NE(obs[0].landmark_key_id, obs[1].landmark_key_id);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
