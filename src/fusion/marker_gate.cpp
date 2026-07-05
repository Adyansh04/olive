#include "olive/fusion/marker_gate.hpp"

#include <algorithm>

namespace olive
{

MarkerGate::MarkerGate(const MarkerGateConfig& config)
  : config_(config)
{}

bool MarkerGate::push(
    double               stamp,
    int                  whycode_id,
    int                  tracking_id,
    bool                 id_valid,
    const gtsam::Point3& position_in_camera)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Acceptance class decides the landmark key. Surveyed ids always pass;
    // decoded-unknown and undecoded detections pass only when their accept
    // flag is on (they become free landmarks, never world anchors).
    int64_t landmark_key_id = -1;
    if (id_valid && config_.known_ids.count(whycode_id) > 0)
        landmark_key_id = whycode_id;
    else if (id_valid && config_.accept_unknown_ids)
        landmark_key_id = whycode_id;
    else if (!id_valid && config_.accept_undecoded_ids)
        landmark_key_id = UNDECODED_LANDMARK_BASE + tracking_id;

    if (landmark_key_id < 0)
    {
        track_streak_.erase(tracking_id);
        return false;
    }

    const double range = position_in_camera.norm();
    if (range < config_.min_range || range > config_.max_range)
    {
        track_streak_.erase(tracking_id);
        return false;
    }

    if (++track_streak_[tracking_id] < config_.min_track_frames)
        return false;

    accepted_.push_back(
        { stamp, id_valid ? whycode_id : -1, landmark_key_id, id_valid, position_in_camera });
    const double cutoff = stamp - config_.history;
    while (!accepted_.empty() && accepted_.front().stamp < cutoff)
        accepted_.pop_front();
    return true;
}

std::vector<MarkerObservation> MarkerGate::collectNear(double stamp, double window)
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<MarkerObservation> result;
    std::deque<MarkerObservation>  kept;
    for (const MarkerObservation& obs : accepted_)
    {
        if (std::abs(obs.stamp - stamp) <= window)
            result.push_back(obs);
        else if (obs.stamp > stamp)
            kept.push_back(obs);  // future observations stay for the next keyframe
    }
    accepted_.swap(kept);

    // One observation per landmark per keyframe: keep the closest-in-time.
    std::sort(result.begin(), result.end(), [&](const auto& a, const auto& b) {
        return std::abs(a.stamp - stamp) < std::abs(b.stamp - stamp);
    });
    std::vector<MarkerObservation> unique;
    std::set<int64_t>              seen;
    for (const MarkerObservation& obs : result)
    {
        if (seen.insert(obs.landmark_key_id).second)
            unique.push_back(obs);
    }
    return unique;
}

}  // namespace olive
