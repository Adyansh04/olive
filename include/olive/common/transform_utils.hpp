/**
 * @file transform_utils.hpp
 * @brief Coordinate transformation utilities
 *
 * Provides helper functions for coordinate transformations, pose conversions,
 * and ROS message conversions.
 */

#ifndef OLIVE_COMMON_TRANSFORM_UTILS_HPP_
#define OLIVE_COMMON_TRANSFORM_UTILS_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "olive/common/types.hpp"

namespace olive
{
namespace transform_utils
{

/**
 * @brief Convert Eigen pose to ROS pose message
 */
inline geometry_msgs::msg::Pose eigenToRosPose(const Pose3D& pose)
{
    geometry_msgs::msg::Pose ros_pose;
    ros_pose.position.x    = pose.position.x();
    ros_pose.position.y    = pose.position.y();
    ros_pose.position.z    = pose.position.z();
    ros_pose.orientation.w = pose.orientation.w();
    ros_pose.orientation.x = pose.orientation.x();
    ros_pose.orientation.y = pose.orientation.y();
    ros_pose.orientation.z = pose.orientation.z();
    return ros_pose;
}

/**
 * @brief Convert ROS Pose message to Eigen pose
 */
inline Pose3D rosPoseToEigen(const geometry_msgs::msg::Pose& ros_pose)
{
    Pose3D pose;
    pose.position = Eigen::Vector3d(ros_pose.position.x, ros_pose.position.y, ros_pose.position.z);
    pose.orientation = Eigen::Quaterniond(
        ros_pose.orientation.w,
        ros_pose.orientation.x,
        ros_pose.orientation.y,
        ros_pose.orientation.z);
    return pose;
}

/**
 * @brief Convert OdometryMeasurement to ROS Odometry message
 */
inline nav_msgs::msg::Odometry toRosOdometry(const OdometryMeasurement& odom)
{
    nav_msgs::msg::Odometry ros_odom;

    // Header
    ros_odom.header.stamp    = rclcpp::Time(static_cast<int64_t>(odom.timestamp * 1e9));
    ros_odom.header.frame_id = odom.frame_id;
    ros_odom.child_frame_id  = odom.child_frame_id;

    // Pose
    ros_odom.pose.pose = eigenToRosPose(odom.pose);

    // Pose covariance (6x6 row-major)
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            ros_odom.pose.covariance[i * 6 + j] = odom.pose_covariance(i, j);
        }
    }

    // Twist (velocity)
    ros_odom.twist.twist.linear.x  = odom.velocity.linear.x();
    ros_odom.twist.twist.linear.y  = odom.velocity.linear.y();
    ros_odom.twist.twist.linear.z  = odom.velocity.linear.z();
    ros_odom.twist.twist.angular.x = odom.velocity.angular.x();
    ros_odom.twist.twist.angular.y = odom.velocity.angular.y();
    ros_odom.twist.twist.angular.z = odom.velocity.angular.z();

    return ros_odom;
}

/**
 * @brief Convert ROS Odometry message to OdometryMeasurement
 */
inline OdometryMeasurement fromRosOdometry(const nav_msgs::msg::Odometry& ros_odom)
{
    OdometryMeasurement odom;

    // Timestamp
    odom.timestamp      = ros_odom.header.stamp.sec + ros_odom.header.stamp.nanosec * 1e-9;
    odom.frame_id       = ros_odom.header.frame_id;
    odom.child_frame_id = ros_odom.child_frame_id;

    // Pose
    odom.pose           = rosPoseToEigen(ros_odom.pose.pose);
    odom.pose.timestamp = odom.timestamp;

    // Pose covariance
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            odom.pose_covariance(i, j) = ros_odom.pose.covariance[i * 6 + j];
        }
    }

    // Velocity
    odom.velocity.linear = Eigen::Vector3d(
        ros_odom.twist.twist.linear.x,
        ros_odom.twist.twist.linear.y,
        ros_odom.twist.twist.linear.z);
    odom.velocity.angular = Eigen::Vector3d(
        ros_odom.twist.twist.angular.x,
        ros_odom.twist.twist.angular.y,
        ros_odom.twist.twist.angular.z);
    odom.velocity.timestamp = odom.timestamp;

    return odom;
}

/**
 * @brief Compose two poses: result = pose1 * pose2
 */
inline Pose3D composePoses(const Pose3D& pose1, const Pose3D& pose2)
{
    Pose3D result;
    result.position    = pose1.position + pose1.orientation * pose2.position;
    result.orientation = pose1.orientation * pose2.orientation;
    result.timestamp   = pose2.timestamp;  // Use the latest timestamp
    return result;
}

/**
 * @brief Compute inverse of a pose
 */
inline Pose3D inversePose(const Pose3D& pose)
{
    Pose3D result;
    result.orientation = pose.orientation.inverse();
    result.position    = -(result.orientation * pose.position);
    result.timestamp   = pose.timestamp;
    return result;
}

/**
 * @brief Compute relative pose: T_1_inv * T_2
 */
inline Pose3D relativePose(const Pose3D& pose1, const Pose3D& pose2)
{
    return composePoses(inversePose(pose1), pose2);
}

/**
 * @brief Convert Euler angles (roll, pitch, yaw) to quaternion
 */
inline Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw)
{
    Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());

    return yaw_angle * pitch_angle * roll_angle;
}

/**
 * @brief Convert quaternion to Euler angles (roll, pitch, yaw)
 */
inline Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q)
{
    return q.toRotationMatrix().eulerAngles(0, 1, 2);  // XYZ convention
}

/**
 * @brief Interpolate between two poses using SLERP
 * @param pose1 Start pose
 * @param pose2 End pose
 * @param alpha Interpolation factor [0, 1]
 * @return Interpolated pose
 */
inline Pose3D interpolatePose(const Pose3D& pose1, const Pose3D& pose2, double alpha)
{
    Pose3D result;
    result.position    = (1.0 - alpha) * pose1.position + alpha * pose2.position;
    result.orientation = pose1.orientation.slerp(alpha, pose2.orientation);
    result.timestamp   = (1.0 - alpha) * pose1.timestamp + alpha * pose2.timestamp;
    return result;
}

/**
 * @brief Create a 6x6 covariance matrix with diagonal values
 */
inline Eigen::Matrix<double, 6, 6> createDiagonalCovariance(
    double pos_x, double pos_y, double pos_z, double rot_x, double rot_y, double rot_z)
{
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
    cov(0, 0)                       = pos_x * pos_x;
    cov(1, 1)                       = pos_y * pos_y;
    cov(2, 2)                       = pos_z * pos_z;
    cov(3, 3)                       = rot_x * rot_x;
    cov(4, 4)                       = rot_y * rot_y;
    cov(5, 5)                       = rot_z * rot_z;
    return cov;
}

/**
 * @brief Scale covariance matrix by a factor
 */
inline Eigen::Matrix<double, 6, 6>
scaleCovariance(const Eigen::Matrix<double, 6, 6>& cov, double scale_factor)
{
    return cov * scale_factor;
}

}  // namespace transform_utils
}  // namespace olive

#endif  // OLIVE_COMMON_TRANSFORM_UTILS_HPP_