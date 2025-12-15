/**
 * @file registration_types.hpp
 * @brief Type definitions for point cloud registration
 *
 * Two registration methods are supported:
 * 1. Feature-based: Point-to-edge + point-to-plane residuals (primary)
 * 2. GICP fallback: When features are insufficient
 *
 * For ground robots, registration is constrained to (x, y, yaw).
 */

#ifndef OLIVE_LIDAR_REGISTRATION_TYPES_HPP_
#define OLIVE_LIDAR_REGISTRATION_TYPES_HPP_

#include <Eigen/Dense>
#include <limits>

namespace olive
{

/**
 * @brief Configuration for feature-based registration optimizer
 */
struct FeatureRegistrationConfig
{
    // Gauss-Newton optimization
    int    max_iterations{ 10 };           ///< Max optimization iterations
    double convergence_threshold{ 1e-4 };  ///< Delta norm for convergence
    double lambda{ 0.1 };                  ///< Levenberg-Marquardt damping

    // Ground robot constraint
    bool   is_2d_mode{ true };              ///< Constrain to (x, y, yaw) only
    double ground_height_threshold{ 0.3 };  ///< Height above ground to consider (m)

    // Correspondence search
    double edge_search_radius{ 1.0 };    ///< KNN search radius for edges (m)
    double planar_search_radius{ 1.0 };  ///< KNN search radius for planars (m)
    int    edge_neighbors{ 5 };          ///< Neighbors for edge line fitting
    int    planar_neighbors{ 5 };        ///< Neighbors for plane fitting

    // Outlier rejection
    double max_edge_residual{ 0.5 };     ///< Max acceptable edge residual (m)
    double max_planar_residual{ 0.3 };   ///< Max acceptable planar residual (m)
    double min_eigenvalue_ratio{ 3.0 };  ///< Min ratio for valid line/plane fit

    // Residual weighting
    double edge_weight{ 1.0 };    ///< Weight for edge residuals
    double planar_weight{ 0.5 };  ///< Weight for planar residuals

    // Minimum correspondences for valid registration
    int min_edge_correspondences{ 10 };
    int min_planar_correspondences{ 20 };
};

/**
 * @brief Result from feature-based registration
 *
 * Contains transformation, quality metrics, and diagnostic info.
 */
struct FeatureRegistrationResult
{
    Eigen::Matrix4f transformation{ Eigen::Matrix4f::Identity() };

    // Quality metrics
    double edge_rmse{ std::numeric_limits<double>::max() };   ///< RMS error for edge residuals
    double plane_rmse{ std::numeric_limits<double>::max() };  ///< RMS error for plane residuals
    double overall_fitness{ std::numeric_limits<double>::max() };

    // Correspondence counts
    int inliers_edge{ 0 };   ///< Number of valid edge correspondences
    int inliers_plane{ 0 };  ///< Number of valid planar correspondences

    // Status flags
    bool converged{ false };   ///< Did optimizer converge?
    bool degenerate{ false };  ///< Is environment geometrically degenerate?
    int  iterations{ 0 };      ///< Iterations used
};

/**
 * @brief Result from GICP fallback registration
 */
struct GICPRegistrationResult
{
    Eigen::Matrix4f transformation{ Eigen::Matrix4f::Identity() };
    double          fitness_score{ std::numeric_limits<double>::max() };
    bool            converged{ false };
    bool            degenerate{ false };
    int             num_correspondences{ 0 };
};

/**
 * @brief Unified registration result (wraps either method)
 */
struct RegistrationResult
{
    Eigen::Matrix4f transformation{ Eigen::Matrix4f::Identity() };
    double          fitness_score{ std::numeric_limits<double>::max() };
    bool            converged{ false };
    bool            degenerate{ false };
    int             num_correspondences{ 0 };

    // Source tracking
    enum class Method
    {
        FEATURE,
        GICP,
        PREDICTION
    };
    Method method{ Method::PREDICTION };

    // Feature-specific metrics (valid when method == FEATURE)
    double edge_rmse{ 0.0 };
    double plane_rmse{ 0.0 };
    int    inliers_edge{ 0 };
    int    inliers_plane{ 0 };
};

}  // namespace olive

#endif  // OLIVE_LIDAR_REGISTRATION_TYPES_HPP_