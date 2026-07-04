/**
 * @file covariance_utils.hpp
 * @brief Covariance-matrix helpers shared by all fusion front-ends and the backend
 *
 * ROS pose covariances are ordered [translation(3), rotation(3)] while GTSAM
 * Pose3 tangent-space covariances are ordered [rotation(3), translation(3)].
 * Every 6x6 covariance crossing that boundary must pass through the block
 * permutation implemented here, or the resulting noise models are silently
 * mis-weighted.
 */

#ifndef OLIVE_COMMON_COVARIANCE_UTILS_HPP_
#define OLIVE_COMMON_COVARIANCE_UTILS_HPP_

#include <Eigen/Dense>
#include <array>

namespace olive
{
namespace covariance_utils
{

using Matrix6d = Eigen::Matrix<double, 6, 6>;

/**
 * @brief Swap the translation and rotation 3x3 blocks of a 6x6 covariance.
 *
 * Maps [A B; C D] -> [D C; B A] where A is the upper-left 3x3 block. The
 * permutation is an involution: applying it twice returns the input.
 */
inline Matrix6d swapCovarianceBlocks(const Matrix6d& cov)
{
    Matrix6d swapped;
    swapped.block<3, 3>(0, 0) = cov.block<3, 3>(3, 3);
    swapped.block<3, 3>(0, 3) = cov.block<3, 3>(3, 0);
    swapped.block<3, 3>(3, 0) = cov.block<3, 3>(0, 3);
    swapped.block<3, 3>(3, 3) = cov.block<3, 3>(0, 0);
    return swapped;
}

/**
 * @brief ROS [trans, rot] covariance -> GTSAM Pose3 [rot, trans] covariance
 */
inline Matrix6d rosToGtsamCovariance(const Matrix6d& ros_cov)
{
    return swapCovarianceBlocks(ros_cov);
}

/**
 * @brief GTSAM Pose3 [rot, trans] covariance -> ROS [trans, rot] covariance
 */
inline Matrix6d gtsamToRosCovariance(const Matrix6d& gtsam_cov)
{
    return swapCovarianceBlocks(gtsam_cov);
}

/**
 * @brief Convert a ROS message covariance array (row-major 6x6) to Eigen
 */
inline Matrix6d fromRosArray(const std::array<double, 36>& cov)
{
    return Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(cov.data());
}

/**
 * @brief Convert an Eigen 6x6 covariance to a ROS message covariance array
 */
inline std::array<double, 36> toRosArray(const Matrix6d& cov)
{
    std::array<double, 36> out;
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(out.data()) = cov;
    return out;
}

}  // namespace covariance_utils
}  // namespace olive

#endif  // OLIVE_COMMON_COVARIANCE_UTILS_HPP_
