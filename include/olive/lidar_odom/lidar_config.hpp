/**
 * @file lidar_config.hpp
 * @brief Configuration structures for LiDAR odometry system
 *
 * Design: Two-tier registration with 2D ground robot constraints
 * - Primary: Feature-based scan-to-map alignment using edge/planar features
 * - Fallback: GICP when feature registration fails or is insufficient
 * - All poses constrained to (x, y, yaw) for ground robots
 */

#ifndef OLIVE_LIDAR_LIDAR_CONFIG_HPP_
#define OLIVE_LIDAR_LIDAR_CONFIG_HPP_

#include <string>

namespace olive
{
/**
 * @brief Core configuration for LiDAR odometry node
 *
 * Parameters are grouped by function:
 * - Keyframe management: When to update the local map
 * - GICP fallback: Parameters for fallback registration
 * - Motion model: Velocity estimation and prediction
 * - Quality thresholds: When to trust registration results
 */
struct LidarConfig
{
    // === Sensor Configuration ===
    bool is_3d{ true };  ///< True for 3D LiDAR, false for 2D laser scan

    // === Keyframe Management ===
    double keyframe_distance{ 0.3 };   ///< Min translation to trigger keyframe (m)
    double keyframe_rotation{ 0.15 };  ///< Min rotation to trigger keyframe (rad)

    // === Point Cloud Filtering ===
    double voxel_leaf_size{ 0.15 };     ///< Voxel grid leaf size for downsampling (m)
    int    min_points_threshold{ 50 };  ///< Minimum points for valid registration

    // === GICP Fallback Parameters ===
    double max_correspondence_dist{ 1.0 };  ///< Max point correspondence distance (m)
    int    max_iterations{ 15 };            ///< Max GICP iterations
    double transformation_epsilon{ 1e-4 };  ///< Convergence threshold for transform
    double fitness_epsilon{ 1e-2 };         ///< Convergence threshold for fitness

    // === Covariance Estimation ===
    double nominal_pos_std{ 0.02 };   ///< Base position uncertainty (m)
    double nominal_rot_std{ 0.02 };   ///< Base rotation uncertainty (rad)
    double poor_fit_scale{ 3.0 };     ///< Covariance multiplier for poor fitness
    double fitness_threshold{ 0.8 };  ///< Threshold for "good" registration

    // === Frame-to-Frame Tracking ===
    double frame_fitness_threshold{ 1.0 };  ///< Acceptable fitness for frame-to-frame
    double max_frame_distance{ 1.0 };       ///< Max expected motion between frames (m)
    double velocity_filter_alpha{ 0.3 };    ///< EMA coefficient for velocity smoothing
    double degeneracy_threshold{ 100.0 };   ///< Eigenvalue ratio for degeneracy detection

    // === Feature-Based Registration ===
    bool   use_feature_registration{ true };    ///< Enable edge/planar feature matching
    double edge_curvature_threshold{ 0.1 };     ///< Curvature threshold for edge points
    double planar_curvature_threshold{ 0.01 };  ///< Curvature threshold for planar points
    int    max_edge_features{ 400 };            ///< Max edge features per scan
    int    max_planar_features{ 800 };          ///< Max planar features per scan

    // === Local Map Configuration ===
    int    local_map_size{ 10 };         ///< Number of keyframes in sliding window
    double local_map_voxel_size{ 0.2 };  ///< Voxel size for map downsampling (m)
};

/**
 * @brief Frame ID configuration
 */
struct FrameConfig
{
    std::string odom_frame_id{ "odom" };
    std::string lidar_frame_id{ "lidar_link" };
};

}  // namespace olive

#endif  // OLIVE_LIDAR_LIDAR_CONFIG_HPP_