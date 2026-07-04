/**
 * @file marker_anchor_factor.hpp
 * @brief Position-only fiducial anchor factor on a robot pose
 *
 * A detected marker with a known world position acts like a local GPS fix:
 * the factor compares the marker position predicted from the robot pose (via
 * the fixed base<-camera extrinsic) against the measured position in the
 * camera frame. Marker orientation is never used — circular fiducial
 * orientation estimates flip under out-of-plane tilt and are unreliable.
 */

#ifndef OLIVE_FUSION_MARKER_ANCHOR_FACTOR_HPP_
#define OLIVE_FUSION_MARKER_ANCHOR_FACTOR_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace olive
{

/**
 * @brief Unary factor: residual between the measured and predicted marker
 *        position in the camera frame.
 *
 * predicted = (X * base_T_cam)^-1 * world_p_marker
 */
class MarkerAnchorFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3>
{
public:
    MarkerAnchorFactor(
        gtsam::Key                     pose_key,
        const gtsam::Point3&           measured_in_camera,
        const gtsam::Point3&           marker_in_world,
        const gtsam::Pose3&            base_from_camera,
        const gtsam::SharedNoiseModel& noise)
      : gtsam::NoiseModelFactorN<gtsam::Pose3>(noise, pose_key)
      , measured_(measured_in_camera)
      , marker_world_(marker_in_world)
      , base_from_camera_(base_from_camera)
    {}

    gtsam::Vector evaluateError(
        const gtsam::Pose3&             pose,
        boost::optional<gtsam::Matrix&> jacobian = boost::none) const override
    {
        // Camera pose in the world, with the chain rule through the fixed
        // extrinsic handled by GTSAM's compose Jacobian.
        gtsam::Matrix66    d_cam_d_pose;
        const gtsam::Pose3 world_from_camera = pose.compose(base_from_camera_, d_cam_d_pose);

        gtsam::Matrix36     d_pred_d_cam;
        const gtsam::Point3 predicted = world_from_camera.transformTo(marker_world_, d_pred_d_cam);

        if (jacobian)
            *jacobian = d_pred_d_cam * d_cam_d_pose;
        return predicted - measured_;
    }

private:
    gtsam::Point3 measured_;      ///< Marker position in the camera frame
    gtsam::Point3 marker_world_;  ///< Known marker position in the map frame
    gtsam::Pose3  base_from_camera_;
};

}  // namespace olive

#endif  // OLIVE_FUSION_MARKER_ANCHOR_FACTOR_HPP_
