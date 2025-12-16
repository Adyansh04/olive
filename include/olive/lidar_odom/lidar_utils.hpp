/**
 * @file lidar_utils.hpp
 * @brief Utility functions for LiDAR odometry
 *
 * Contains pure helper functions that don't depend on node state:
 * - 2D pose enforcement for ground robots
 * - Motion prediction using constant velocity model
 * - Velocity estimation with EMA filtering
 * - Covariance generation
 */

#ifndef OLIVE_LIDAR_LIDAR_UTILS_HPP_
#define OLIVE_LIDAR_LIDAR_UTILS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "olive/common/types.hpp"

namespace olive
{
namespace lidar_utils
{

/**
 * @brief Enforce 2D ground robot constraints on a pose
 *
 * Sets z = 0, roll = 0, pitch = 0, preserving only (x, y, yaw).
 * @param pose Pose to modify in-place
 */
inline void enforce2DConstraint(Pose3D& pose)
{
    // Zero out z position
    pose.position.z() = 0.0;

    // Extract yaw from current orientation
    Eigen::Matrix3d rot = pose.orientation.toRotationMatrix();
    double          yaw = std::atan2(rot(1, 0), rot(0, 0));

    // Reconstruct quaternion with only yaw (roll=0, pitch=0)
    pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    pose.orientation.normalize();
}

/**
 * @brief Enforce 2D constraint on a transformation matrix
 *
 * @param transform 4x4 transformation matrix to modify in-place
 */
inline void enforce2DConstraint(Eigen::Matrix4f& transform)
{
    // Zero out z translation
    transform(2, 3) = 0.0f;

    // Extract yaw from rotation
    Eigen::Matrix3f rot = transform.block<3, 3>(0, 0);
    float           yaw = std::atan2(rot(1, 0), rot(0, 0));

    // Reconstruct rotation with only yaw
    float cy = std::cos(yaw);
    float sy = std::sin(yaw);

    transform.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
    transform(0, 0)             = cy;
    transform(0, 1)             = -sy;
    transform(1, 0)             = sy;
    transform(1, 1)             = cy;
}

/**
 * @brief Extract yaw angle from quaternion
 *
 * Uses atan2 for robust extraction without gimbal lock issues.
 *
 * @param q Input quaternion
 * @return Yaw angle in radians [-pi, pi]
 */
inline double extractYaw(const Eigen::Quaterniond& q)
{
    Eigen::Matrix3d rot = q.toRotationMatrix();
    return std::atan2(rot(1, 0), rot(0, 0));
}

/**
 * @brief Create quaternion from yaw angle only (for ground robots)
 *
 * @param yaw Yaw angle in radians
 * @return Quaternion representing rotation about Z axis
 */
inline Eigen::Quaterniond yawToQuaternion(double yaw)
{
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

/**
 * @brief Predict motion using constant velocity model (2D ground robot)
 *
 * This provides a good initial guess for ICP, significantly improving
 * convergence speed and robustness.
 * Motion model:
 *   x(t+dt) = x(t) + v * dt
 *   y(t+dt) = y(t) + v * dt
 *   yaw(t+dt) = yaw(t) + omega * dt
 *
 * @param linear_velocity Current linear velocity estimate (m/s)
 * @param angular_velocity Current angular velocity estimate (rad/s)
 * @param dt Time delta (seconds)
 * @return Predicted transformation matrix
 */
inline Eigen::Matrix4f predictMotion2D(
    const Eigen::Vector3d& linear_velocity,
    const Eigen::Vector3d& angular_velocity,
    double                 dt)
{
    Eigen::Matrix4f prediction = Eigen::Matrix4f::Identity();

    if (dt <= 0.0 || dt > 1.0)
    {
        return prediction;
    }

    // Translation prediction (x, y only)
    prediction(0, 3) = static_cast<float>(linear_velocity.x() * dt);
    prediction(1, 3) = static_cast<float>(linear_velocity.y() * dt);
    prediction(2, 3) = 0.0f;  // z = 0 for ground robot

    // Rotation prediction (yaw only)
    double yaw_rate  = angular_velocity.z();
    double yaw_delta = yaw_rate * dt;

    if (std::abs(yaw_delta) > 1e-6)
    {
        float cy         = static_cast<float>(std::cos(yaw_delta));
        float sy         = static_cast<float>(std::sin(yaw_delta));
        prediction(0, 0) = cy;
        prediction(0, 1) = -sy;
        prediction(1, 0) = sy;
        prediction(1, 1) = cy;
    }

    return prediction;
}

/**
 * @brief Update velocity estimate using exponential moving average
 *
 * EMA formula: v_new = alpha * v_measured + (1 - alpha) * v_old
 *
 * @param current_linear Current linear velocity estimate
 * @param current_angular Current angular velocity estimate
 * @param delta_transform Measured transformation
 * @param dt Time delta (seconds)
 * @param alpha EMA smoothing factor (0-1)
 * @param max_linear_vel Maximum acceptable linear velocity (m/s)
 * @param max_angular_vel Maximum acceptable angular velocity (rad/s)
 * @return Pair of updated (linear, angular) velocities
 */
inline std::pair<Eigen::Vector3d, Eigen::Vector3d> updateVelocityEMA(
    const Eigen::Vector3d& current_linear,
    const Eigen::Vector3d& current_angular,
    const Eigen::Matrix4f& delta_transform,
    double                 dt,
    double                 alpha,
    double                 max_linear_vel,
    double                 max_angular_vel)
{
    if (dt <= 0.001)
    {
        return { current_linear, current_angular };
    }

    // Extract measured linear velocity
    Eigen::Vector3d measured_linear(
        delta_transform(0, 3) / dt,
        delta_transform(1, 3) / dt,
        delta_transform(2, 3) / dt);

    // Extract measured angular velocity
    Eigen::Matrix3d   rot = delta_transform.block<3, 3>(0, 0).cast<double>();
    Eigen::AngleAxisd angle_axis(rot);
    Eigen::Vector3d   measured_angular = angle_axis.axis() * angle_axis.angle() / dt;

    // Reject unreasonable velocities
    if (measured_linear.norm() > max_linear_vel || measured_angular.norm() > max_angular_vel)
    {
        return { current_linear, current_angular };
    }

    // Apply EMA filter
    Eigen::Vector3d new_linear  = alpha * measured_linear + (1.0 - alpha) * current_linear;
    Eigen::Vector3d new_angular = alpha * measured_angular + (1.0 - alpha) * current_angular;

    return { new_linear, new_angular };
}

/**
 * @brief Create diagonal 6x6 covariance matrix
 *
 * @param pos_std Position standard deviation (m)
 * @param rot_std Rotation standard deviation (rad)
 * @return 6x6 covariance matrix
 */
inline Eigen::Matrix<double, 6, 6> createCovariance(double pos_std, double rot_std)
{
    Eigen::Matrix<double, 6, 6> cov     = Eigen::Matrix<double, 6, 6>::Zero();
    double                      pos_var = pos_std * pos_std;
    double                      rot_var = rot_std * rot_std;

    cov(0, 0) = pos_var;
    cov(1, 1) = pos_var;
    cov(2, 2) = pos_var;
    cov(3, 3) = rot_var;
    cov(4, 4) = rot_var;
    cov(5, 5) = rot_var;

    return cov;
}

/**
 * @brief Scale covariance based on registration quality
 *
 * @param fitness Registration fitness score
 * @param threshold Good fitness threshold
 * @param poor_scale Scale factor for poor fitness
 * @param degenerate Whether environment is geometrically degenerate
 * @return Scale factor to apply to covariance
 */
inline double
    computeCovarianceScale(double fitness, double threshold, double poor_scale, bool degenerate)
{
    double scale = 1.0;

    if (fitness > threshold)
    {
        double excess = (fitness - threshold) / threshold;
        scale         = poor_scale * (1.0 + excess * excess);
    }

    if (degenerate)
    {
        scale *= 2.0;
    }

    return scale;
}

/**
 * @brief Ensure quaternion consistency to avoid sign flips
 *
 * If the dot product with previous quaternion is negative,
 * negate the quaternion (same rotation, opposite sign).
 *
 * @param current Current quaternion
 * @param previous Previous quaternion
 * @return Consistent quaternion
 */
inline Eigen::Quaterniond ensureQuaternionConsistency(
    const Eigen::Quaterniond& current,
    const Eigen::Quaterniond& previous)
{
    if (current.dot(previous) < 0.0)
    {
        return Eigen::Quaterniond(-current.coeffs());
    }
    return current;
}

/**
 * @brief Check if motion exceeds sanity bounds
 *
 * @param transform Transformation to check
 * @param max_translation Maximum acceptable translation (m)
 * @param max_rotation Maximum acceptable rotation (rad)
 * @return True if motion is within bounds
 */
inline bool
    isMotionReasonable(const Eigen::Matrix4f& transform, double max_translation, double max_rotation)
{
    // Check translation
    double trans = transform.block<3, 1>(0, 3).norm();
    if (trans > max_translation)
    {
        return false;
    }

    // Check rotation
    Eigen::Matrix3f   rot = transform.block<3, 3>(0, 0);
    Eigen::AngleAxisf aa(rot);

    return std::abs(aa.angle()) <= max_rotation;
}

}  // namespace lidar_utils

}  // namespace olive
#endif  // OLIVE_LIDAR_LIDAR_UTILS_HPP_