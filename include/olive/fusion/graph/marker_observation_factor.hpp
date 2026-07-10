/**
 * @file marker_observation_factor.hpp
 * @brief Position-only fiducial observation factor on (pose, landmark)
 *
 * TagSLAM-style landmark formulation: the marker position is a variable in
 * the graph rather than a surveyed constant. Every keyframe that sees the
 * marker contributes one factor, so two sightings of the same physical
 * marker constrain the relative motion between the observing keyframes —
 * markers act as an odometry source even without a survey. Surveyed markers
 * get a separate position prior, which turns their observations into global
 * anchors. Marker orientation is never used — circular fiducial orientation
 * estimates flip under out-of-plane tilt and are unreliable.
 */

#ifndef OLIVE_FUSION_GRAPH_MARKER_OBSERVATION_FACTOR_HPP_
#define OLIVE_FUSION_GRAPH_MARKER_OBSERVATION_FACTOR_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace olive
{

/**
 * @brief Binary factor: residual between the measured and predicted marker
 *        position in the camera frame.
 *
 * predicted = (X * base_T_cam)^-1 * L
 */
class MarkerObservationFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3>
{
public:
    MarkerObservationFactor(
        gtsam::Key                     pose_key,
        gtsam::Key                     landmark_key,
        const gtsam::Point3&           measured_in_camera,
        const gtsam::Pose3&            base_from_camera,
        const gtsam::SharedNoiseModel& noise)
      : gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3>(noise, pose_key, landmark_key)
      , measured_(measured_in_camera)
      , base_from_camera_(base_from_camera)
    {}

    gtsam::Vector evaluateError(
        const gtsam::Pose3&             pose,
        const gtsam::Point3&            landmark,
        boost::optional<gtsam::Matrix&> H_pose     = boost::none,
        boost::optional<gtsam::Matrix&> H_landmark = boost::none) const override
    {
        // Camera pose in the world, with the chain rule through the fixed
        // extrinsic handled by GTSAM's compose Jacobian.
        gtsam::Matrix66    d_cam_d_pose;
        const gtsam::Pose3 world_from_camera = pose.compose(base_from_camera_, d_cam_d_pose);

        gtsam::Matrix36     d_pred_d_cam;
        gtsam::Matrix33     d_pred_d_landmark;
        const gtsam::Point3 predicted =
            world_from_camera.transformTo(landmark, d_pred_d_cam, d_pred_d_landmark);

        if (H_pose)
            *H_pose = d_pred_d_cam * d_cam_d_pose;
        if (H_landmark)
            *H_landmark = d_pred_d_landmark;
        return predicted - measured_;
    }

private:
    gtsam::Point3 measured_;  ///< Marker position in the camera frame
    gtsam::Pose3  base_from_camera_;
};

}  // namespace olive

#endif  // OLIVE_FUSION_GRAPH_MARKER_OBSERVATION_FACTOR_HPP_
