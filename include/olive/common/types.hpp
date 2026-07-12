/**
 * @file types.hpp
 * @brief Common type definitions for the Olive sensor fusion system
 *
 * Define common data structures, types, and constants used across all nodes
 * in the sensor fusion system.
 */

#ifndef OLIVE_COMMON_TYPES_HPP_
#define OLIVE_COMMON_TYPES_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <numbers>
#include <string>

namespace olive
{

/**
 * @brief 3D Pose representation (Position + Orientation)
 */
struct Pose3D
{
    Eigen::Vector3d    position    = Eigen::Vector3d::Zero();         ///< 3D position (x, y, z)
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  ///< Orientation (w, x, y, z)
    double             timestamp   = 0.0;                             ///< Timestamp in seconds
};

/**
 * @brief Velocity representation (linear + angular)
 */
struct Velocity3D
{
    Eigen::Vector3d linear    = Eigen::Vector3d::Zero();  ///< Linear velocity (vx, vy, vz)
    Eigen::Vector3d angular   = Eigen::Vector3d::Zero();  ///< Angular velocity (wx, wy, wz)
    double          timestamp = 0.0;                      ///< Timestamp in seconds
};

/**
 * @brief IMU measurement data
 */
struct ImuData
{
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();  ///< Linear accel (ax, ay, az)
    Eigen::Vector3d angular_velocity    = Eigen::Vector3d::Zero();  ///< Angular vel (wx, wy, wz)
    double          timestamp           = 0.0;                      ///< Timestamp in seconds
};

/**
 * @brief Wheel encoder data
 */
struct WheelData
{
    double left_ticks  = 0.0;  ///< Left wheel encoder ticks
    double right_ticks = 0.0;  ///< Right wheel encoder ticks
    double timestamp   = 0.0;  ///< Timestamp in seconds
};

/**
 * @brief Odometry measurement with covariance
 */
struct OdometryMeasurement
{
    Pose3D                      pose;      ///< Estimated pose
    Velocity3D                  velocity;  ///< Estimated velocity
    Eigen::Matrix<double, 6, 6> pose_covariance =
        Eigen::Matrix<double, 6, 6>::Identity();  ///< Pose covariance (6x6)
    double      timestamp      = 0.0;             ///< Timestamp in seconds
    std::string frame_id       = "map";           ///< Reference frame
    std::string child_frame_id = "base_link";     ///< Child frame
};

/**
 * @brief Marker (WhyCode) detection data
 */
struct MarkerDetection
{
    int                         marker_id = -1;     ///< Unique marker ID
    Pose3D                      pose_camera_frame;  ///< Pose in camera frame
    Eigen::Matrix<double, 6, 6> covariance =
        Eigen::Matrix<double, 6, 6>::Identity();  ///< Measurement covariance
    double confidence = 0.0;                      ///< Detection confidence [0, 1]
    double timestamp  = 0.0;                      ///< Timestamp in seconds
};

/**
 * @brief Sensor health status
 */
enum class SensorHealth
{
    EXCELLENT,  ///< High confidence, nominal operation
    GOOD,       ///< Normal operation
    DEGRADED,   ///< Reduced quality, still usable
    POOR,       ///< Low quality, minimal trust
    FAILED      ///< Sensor failure, do not use
};

/**
 * @brief Sensor type enumeration
 */
enum class SensorType
{
    WHEEL_IMU,
    LIDAR,
    VISUAL,
    MARKER
};

/**
 * @brief Constants used throughout the system
 */
namespace constants
{
// The POSIX macros are replaced by std::numbers everywhere; these asserts pin
// the swap to the exact same IEEE-754 doubles so fused output cannot move.
static_assert(std::numbers::pi == M_PI);
static_assert(std::numbers::pi / 2.0 == M_PI_2);

constexpr double GRAVITY             = 9.81;                      ///< Gravity (m/s^2)
constexpr double DEG_TO_RAD          = std::numbers::pi / 180.0;  ///< Degree to radian conversion
constexpr double RAD_TO_DEG          = 180.0 / std::numbers::pi;  ///< Radian to degree conversion
constexpr double DEFAULT_ODOM_RATE   = 100.0;                     ///< Default odometry rate (Hz)
constexpr double DEFAULT_FUSION_RATE = 10.0;                      ///< Default fusion rate (Hz)
constexpr double DEFAULT_TIME_STEP   = 0.1;   ///< Default time step for calculations (s)
constexpr double MAX_TIME_DIFF       = 1.0;   ///< Maximum acceptable time difference (s)
constexpr double MIN_SCALE_FACTOR    = 0.1;   ///< Minimum scale factor for visual odometry
constexpr double MAX_SCALE_FACTOR    = 10.0;  ///< Maximum scale factor for visual odometry
}  // namespace constants

}  // namespace olive

#endif  // OLIVE_COMMON_TYPES_HPP_