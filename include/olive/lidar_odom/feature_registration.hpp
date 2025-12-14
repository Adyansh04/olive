/**
 * @file feature_registration.hpp
 * @brief Feature-based point cloud registration using LOAM-style residuals
 *
 * Implements point-to-edge and point-to-plane registration for improved
 * accuracy, especially in rotation estimation.
 *
 * Key advantages over standard GICP:
 * - Edge features provide strong angular constraints
 * - Separate treatment of translation and rotation
 * - More robust to symmetric environments
 *
 * Reference: LOAM, LeGO-LOAM, LIO-SAM
 */

#ifndef OLIVE_LIDAR_FEATURE_REGISTRATION_HPP_
#define OLIVE_LIDAR_FEATURE_REGISTRATION_HPP_

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <memory>

#include "olive/common/types.hpp"
#include "olive/lidar_odom/feature_extractor.hpp"

namespace olive
{

/**
 * @brief Configuration for feature-based registration
 */
struct FeatureRegistrationConfig
{
    // Optimization parameters
    int max_iterations = 10;              ///< Maximum Gauss-Newton iterations
    double convergence_threshold = 1e-4;  ///< Convergence threshold for delta
    double lambda = 0.1;                  ///< Levenberg-Marquardt damping

    // Ground robot mode (2D constraint)
    bool is_2d_mode = true;               ///< Constrain to x, y, yaw only (ground robot)
    double ground_height_threshold = 0.3; ///< Height above ground to consider (m)

    // Correspondence search
    double edge_search_radius = 1.0;      ///< Search radius for edge correspondences (m)
    double planar_search_radius = 1.0;    ///< Search radius for planar correspondences (m)
    int edge_neighbors = 5;               ///< Number of neighbors for edge line fitting
    int planar_neighbors = 5;             ///< Number of neighbors for plane fitting

    // Outlier rejection
    double max_edge_residual = 0.5;       ///< Max residual for edge correspondences (m)
    double max_planar_residual = 0.3;     ///< Max residual for planar correspondences (m)
    double min_eigenvalue_ratio = 3.0;    ///< Min eigenvalue ratio for valid line/plane

    // Weighting
    double edge_weight = 1.0;             ///< Weight for edge residuals
    double planar_weight = 0.5;           ///< Weight for planar residuals

    // Minimum correspondences
    int min_edge_correspondences = 10;
    int min_planar_correspondences = 20;
};

/**
 * @brief Result of feature-based registration
 */
struct FeatureRegistrationResult
{
    Eigen::Matrix4f transformation;       ///< Final transformation
    double edge_fitness;                  ///< RMS error for edge features
    double planar_fitness;                ///< RMS error for planar features
    double overall_fitness;               ///< Combined fitness score
    bool converged;                       ///< Did optimization converge?
    int iterations;                       ///< Number of iterations used
    int edge_correspondences;             ///< Number of edge correspondences
    int planar_correspondences;           ///< Number of planar correspondences
    bool degenerate;                      ///< Is the registration degenerate?

    FeatureRegistrationResult()
        : transformation(Eigen::Matrix4f::Identity())
        , edge_fitness(std::numeric_limits<double>::max())
        , planar_fitness(std::numeric_limits<double>::max())
        , overall_fitness(std::numeric_limits<double>::max())
        , converged(false)
        , iterations(0)
        , edge_correspondences(0)
        , planar_correspondences(0)
        , degenerate(false)
    {}
};

/**
 * @brief Feature-based registration using point-to-edge and point-to-plane residuals
 *
 * This class implements a Gauss-Newton optimizer that minimizes:
 * - Point-to-line distance for edge features (2 DOF constraint per point)
 * - Point-to-plane distance for planar features (1 DOF constraint per point)
 *
 * The combination provides full 6-DOF constraints with strong rotation sensitivity.
 */
class FeatureRegistration
{
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using PointCloudPtr = PointCloud::Ptr;
    using PointCloudConstPtr = PointCloud::ConstPtr;

    /**
     * @brief Construct with configuration
     */
    explicit FeatureRegistration(
        const FeatureRegistrationConfig& config = FeatureRegistrationConfig());

    /**
     * @brief Set target edge features (map)
     */
    void setTargetEdges(const PointCloudConstPtr& edges);

    /**
     * @brief Set target planar features (map)
     */
    void setTargetPlanars(const PointCloudConstPtr& planars);

    /**
     * @brief Register source features to target
     * @param source_edges Edge features from current scan
     * @param source_planars Planar features from current scan
     * @param initial_guess Initial transformation guess
     * @return Registration result
     */
    FeatureRegistrationResult align(
        const PointCloudConstPtr& source_edges,
        const PointCloudConstPtr& source_planars,
        const Eigen::Matrix4f& initial_guess = Eigen::Matrix4f::Identity());

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
        const Eigen::Matrix4f& transform,
        const PointCloudConstPtr& source_edges,
        const PointCloudConstPtr& source_planars,
        Eigen::Matrix<double, 6, 1>& delta,
        double& edge_residual,
        double& planar_residual);

    /**
     * @brief Compute edge residual and Jacobian for a single point
     * @param point Source point (in source frame)
     * @param transform Current transformation
     * @param residual Output: residual (2-vector for point-to-line)
     * @param jacobian Output: Jacobian (2x6 matrix)
     * @return True if valid correspondence found
     */
    bool computeEdgeResidual(
        const pcl::PointXYZ& point,
        const Eigen::Matrix4f& transform,
        Eigen::Vector2d& residual,
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
        const pcl::PointXYZ& point,
        const Eigen::Matrix4f& transform,
        double& residual,
        Eigen::Matrix<double, 1, 6>& jacobian);

    /**
     * @brief Transform a point by the given transformation
     */
    pcl::PointXYZ transformPoint(
        const pcl::PointXYZ& point,
        const Eigen::Matrix4f& transform) const;

    /**
     * @brief Apply delta update to transformation (using Lie algebra)
     */
    Eigen::Matrix4f updateTransform(
        const Eigen::Matrix4f& transform,
        const Eigen::Matrix<double, 6, 1>& delta) const;

    /**
     * @brief Check for degenerate configuration
     */
    bool checkDegeneracy(
        const Eigen::Matrix<double, 6, 6>& H,
        int edge_count,
        int planar_count) const;

    FeatureRegistrationConfig config_;

    // Target feature maps and KD-trees
    PointCloudConstPtr target_edges_;
    PointCloudConstPtr target_planars_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr edge_kdtree_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr planar_kdtree_;
};

}  // namespace olive

#endif  // OLIVE_LIDAR_FEATURE_REGISTRATION_HPP_
