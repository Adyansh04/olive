/**
 * @file gtsam_conversions.hpp
 * @brief Conversions between ROS / olive types and GTSAM geometry types
 *
 * All fusion code converts through these helpers so pose and covariance
 * conventions (see covariance_utils.hpp) are handled in exactly one place.
 */

#ifndef OLIVE_COMMON_GTSAM_CONVERSIONS_HPP_
#define OLIVE_COMMON_GTSAM_CONVERSIONS_HPP_

#include <gtsam/geometry/Pose3.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "olive/common/types.hpp"

namespace olive
{
namespace gtsam_conversions
{

/**
 * @brief ROS Pose message -> gtsam::Pose3
 */
inline gtsam::Pose3 toGtsamPose(const geometry_msgs::msg::Pose& pose)
{
    const gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z);
    const gtsam::Point3 translation(pose.position.x, pose.position.y, pose.position.z);
    return gtsam::Pose3(rotation, translation);
}

/**
 * @brief olive::Pose3D -> gtsam::Pose3
 */
inline gtsam::Pose3 toGtsamPose(const Pose3D& pose)
{
    return gtsam::Pose3(gtsam::Rot3(pose.orientation), gtsam::Point3(pose.position));
}

/**
 * @brief gtsam::Pose3 -> ROS Pose message
 */
inline geometry_msgs::msg::Pose toRosPose(const gtsam::Pose3& pose)
{
    geometry_msgs::msg::Pose ros_pose;
    ros_pose.position.x = pose.translation().x();
    ros_pose.position.y = pose.translation().y();
    ros_pose.position.z = pose.translation().z();

    const gtsam::Quaternion q = pose.rotation().toQuaternion();
    ros_pose.orientation.w    = q.w();
    ros_pose.orientation.x    = q.x();
    ros_pose.orientation.y    = q.y();
    ros_pose.orientation.z    = q.z();
    return ros_pose;
}

/**
 * @brief gtsam::Pose3 -> olive::Pose3D
 */
inline Pose3D toPose3D(const gtsam::Pose3& pose, double timestamp = 0.0)
{
    Pose3D out;
    out.position    = pose.translation();
    out.orientation = pose.rotation().toQuaternion();
    out.timestamp   = timestamp;
    return out;
}

/**
 * @brief ROS Point message -> gtsam::Point3
 */
inline gtsam::Point3 toGtsamPoint(const geometry_msgs::msg::Point& point)
{
    return gtsam::Point3(point.x, point.y, point.z);
}

}  // namespace gtsam_conversions
}  // namespace olive

#endif  // OLIVE_COMMON_GTSAM_CONVERSIONS_HPP_
