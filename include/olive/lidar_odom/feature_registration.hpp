/**
 * @file feature_registration.hpp
 * @brief Feature-based registration using point-to-edge and point-to-plane residuals
 *
 * Implements Gauss-Newton optimization minimizing:
 * - Point-to-line distance for edge features (2 DOF per point)
 * - Point-to-plane distance for planar features (1 DOF per point)
 *
 * Combined residuals provide full 6-DOF constraints with strong
 * rotation sensitivity from edge features.
 *
 * For ground robots: Constrained to (x, y, yaw) - roll, pitch, z are zeroed.
 */

#ifndef OLIVE_LIDAR_FEATURE_REGISTRATION_HPP_
#define OLIVE_LIDAR_FEATURE_REGISTRATION_HPP_

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include "olive/lidar_odom/registration_types.hpp"

namespace olive
{

/**
 * @brief Feature-based registration optimizer
 *
 * Uses Gauss-Newton with Levenberg-Marquardt damping.
 * Targets are set once when the local map updates.
 * Source features are aligned to targets for each frame.
 */
class FeatureRegistration
{
public:
    using PointCloud         = pcl::PointCloud<pcl::PointXYZ>;
    using PointCloudPtr      = PointCloud::Ptr;
    using PointCloudConstPtr = PointCloud::ConstPtr;

    /**
     * @brief Construct with configuration
     */
    explicit FeatureRegistration(
        const FeatureRegistrationConfig& config = FeatureRegistrationConfig());

    /**
     * @brief Set target edge features from local map
     * Rebuilds KD-tree for correspondence search.
     */
    void setTargetEdges(const PointCloudConstPtr& edges);

    /**
     * @brief Set target planar features from local map
     * Rebuilds KD-tree for correspondence search.
     */
    void setTargetPlanars(const PointCloudConstPtr& planars);

    /**
     * @brief Align source features to target map
     *
     * @param source_edges Edge features from current scan
     * @param source_planars Planar features from current scan
     * @param initial_guess Initial transformation estimate
     * @return Registration result with transformation and metrics
     */
    FeatureRegistrationResult alignToLocalMap(
        const PointCloudConstPtr& source_edges,
        const PointCloudConstPtr& source_planars,
        const Eigen::Matrix4f&    initial_guess = Eigen::Matrix4f::Identity());

    /**
     * @brief Update configuration
     */
    void setConfig(const FeatureRegistrationConfig& config) { config_ = config; }

    /**
     * @brief Get current configuration
     */
    const FeatureRegistrationConfig& getConfig() const { return config_; }

private:
    /**
     * @brief Single iteration of Gauss-Newton optimization
     * @param transform Current transformation estimate
     * @param source_edges Source edge points
     * @param source_planars Source planar points
     * @param delta Output: computed delta (6-vector)
     * @param edge_residual Output: edge RMS error
     * @param planar_residual Output: planar RMS error
     * @return Number of valid correspondences
     */
    int optimizationStep(
        const Eigen::Matrix4f&       transform,
        const PointCloudConstPtr&    source_edges,
        const PointCloudConstPtr&    source_planars,
        Eigen::Matrix<double, 6, 1>& delta,
        double&                      edge_residual,
        double&                      planar_residual);

    /**
     * @brief Compute edge residual and Jacobian for a single point
     * @param point Source point (in source frame)
     * @param transform Current transformation
     * @param residual Output: residual (2-vector for point-to-line)
     * @param jacobian Output: Jacobian (2x6 matrix)
     * @return True if valid correspondence found
     */
    bool computeEdgeResidual(
        const pcl::PointXYZ&         point,
        const Eigen::Matrix4f&       transform,
        Eigen::Vector2d&             residual,
        Eigen::Matrix<double, 2, 6>& jacobian);

    /**
     * @brief Compute planar residual and Jacobian for a single point
     * @param point Source point (in source frame)
     * @param transform Current transformation
     * @param residual Output: residual (scalar for point-to-plane)
     * @param jacobian Output: Jacobian (1x6 vector)
     * @return True if valid correspondence found
     */
    bool computePlanarResidual(
        const pcl::PointXYZ&         point,
        const Eigen::Matrix4f&       transform,
        double&                      residual,
        Eigen::Matrix<double, 1, 6>& jacobian);

    /**
     * @brief Transform a point by the given transformation
     */
    pcl::PointXYZ transformPoint(const pcl::PointXYZ& point, const Eigen::Matrix4f& transform) const;

    /**
     * @brief Apply delta update to transformation (using Lie algebra)
     */
    Eigen::Matrix4f updateTransform(
        const Eigen::Matrix4f&             transform,
        const Eigen::Matrix<double, 6, 1>& delta) const;

    /**
     * @brief Check for degenerate configuration
     */
    bool isFrameDegenerate(const Eigen::Matrix<double, 6, 6>& H, int edge_count, int planar_count)
        const;

    /**
     * @brief Compute skew-symmetric matrix for a vector
     *
     * @param v Input 3D vector
     * @return Eigen::Matrix3d Skew-symmetric matrix
     */
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) const;

    FeatureRegistrationConfig config_;

    // Target feature maps and KD-trees
    PointCloudConstPtr                   target_edges_;
    PointCloudConstPtr                   target_planars_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr edge_kdtree_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr planar_kdtree_;

    // Preallocated buffers to avoid per-iteration allocation
    Eigen::Matrix<double, 6, 6> H_buffer_;
    Eigen::Matrix<double, 6, 1> b_buffer_;
};

}  // namespace olive

#endif  // OLIVE_LIDAR_FEATURE_REGISTRATION_HPP_
