/**
 * @file parameter_loader.hpp
 * @brief Centralized parameter loading for LiDAR odometry
 *
 * Consolidates all parameter declaration and loading into a single
 * location for maintainability and consistency.
 */

#ifndef OLIVE_LIDAR_PARAMETER_LOADER_HPP_
#define OLIVE_LIDAR_PARAMETER_LOADER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "olive/lidar_odom/feature_types.hpp"
#include "olive/lidar_odom/lidar_config.hpp"
#include "olive/lidar_odom/map_types.hpp"
#include "olive/lidar_odom/registration_types.hpp"

namespace olive
{
namespace parameter_loader
{

/**
 * @brief Declare and load all LiDAR odometry parameters
 *
 * @param node ROS node to declare parameters on
 * @param lidar_config Output: Core LiDAR config
 * @param frame_config Output: Frame ID config
 * @param feature_config Output: Feature extraction config
 * @param registration_config Output: Registration config
 * @param map_config Output: Local map config
 */
inline void loadAllParameters(
    rclcpp_lifecycle::LifecycleNode& node,
    LidarConfig&                     lidar_config,
    FrameConfig&                     frame_config,
    FeatureExtractionConfig&         feature_config,
    FeatureRegistrationConfig&       registration_config,
    LocalMapConfig&                  map_config)
{
    // === Core LiDAR Parameters ===
    node.declare_parameter("is_3d", lidar_config.is_3d);
    node.declare_parameter("keyframe_distance", lidar_config.keyframe_distance);
    node.declare_parameter("keyframe_rotation", lidar_config.keyframe_rotation);
    node.declare_parameter("voxel_leaf_size", lidar_config.voxel_leaf_size);
    node.declare_parameter("min_points_threshold", lidar_config.min_points_threshold);

    node.get_parameter("is_3d", lidar_config.is_3d);
    node.get_parameter("keyframe_distance", lidar_config.keyframe_distance);
    node.get_parameter("keyframe_rotation", lidar_config.keyframe_rotation);
    node.get_parameter("voxel_leaf_size", lidar_config.voxel_leaf_size);
    node.get_parameter("min_points_threshold", lidar_config.min_points_threshold);

    // === GICP Fallback Parameters ===
    node.declare_parameter("max_correspondence_dist", lidar_config.max_correspondence_dist);
    node.declare_parameter("max_iterations", lidar_config.max_iterations);
    node.declare_parameter("transformation_epsilon", lidar_config.transformation_epsilon);
    node.declare_parameter("fitness_epsilon", lidar_config.fitness_epsilon);

    node.get_parameter("max_correspondence_dist", lidar_config.max_correspondence_dist);
    node.get_parameter("max_iterations", lidar_config.max_iterations);
    node.get_parameter("transformation_epsilon", lidar_config.transformation_epsilon);
    node.get_parameter("fitness_epsilon", lidar_config.fitness_epsilon);

    // === Covariance Parameters ===
    node.declare_parameter("nominal_pos_std", lidar_config.nominal_pos_std);
    node.declare_parameter("nominal_rot_std", lidar_config.nominal_rot_std);
    node.declare_parameter("poor_fit_scale", lidar_config.poor_fit_scale);
    node.declare_parameter("fitness_threshold", lidar_config.fitness_threshold);

    node.get_parameter("nominal_pos_std", lidar_config.nominal_pos_std);
    node.get_parameter("nominal_rot_std", lidar_config.nominal_rot_std);
    node.get_parameter("poor_fit_scale", lidar_config.poor_fit_scale);
    node.get_parameter("fitness_threshold", lidar_config.fitness_threshold);

    // === Frame-to-Frame Parameters ===
    node.declare_parameter("frame_fitness_threshold", lidar_config.frame_fitness_threshold);
    node.declare_parameter("max_frame_distance", lidar_config.max_frame_distance);
    node.declare_parameter("velocity_filter_alpha", lidar_config.velocity_filter_alpha);
    node.declare_parameter("degeneracy_threshold", lidar_config.degeneracy_threshold);

    node.get_parameter("frame_fitness_threshold", lidar_config.frame_fitness_threshold);
    node.get_parameter("max_frame_distance", lidar_config.max_frame_distance);
    node.get_parameter("velocity_filter_alpha", lidar_config.velocity_filter_alpha);
    node.get_parameter("degeneracy_threshold", lidar_config.degeneracy_threshold);

    // === Feature Registration Parameters ===
    node.declare_parameter("use_feature_registration", lidar_config.use_feature_registration);
    node.declare_parameter("edge_curvature_threshold", lidar_config.edge_curvature_threshold);
    node.declare_parameter("planar_curvature_threshold", lidar_config.planar_curvature_threshold);
    node.declare_parameter("max_edge_features", lidar_config.max_edge_features);
    node.declare_parameter("max_planar_features", lidar_config.max_planar_features);

    node.get_parameter("use_feature_registration", lidar_config.use_feature_registration);
    node.get_parameter("edge_curvature_threshold", lidar_config.edge_curvature_threshold);
    node.get_parameter("planar_curvature_threshold", lidar_config.planar_curvature_threshold);
    node.get_parameter("max_edge_features", lidar_config.max_edge_features);
    node.get_parameter("max_planar_features", lidar_config.max_planar_features);

    // === Local Map Parameters ===
    node.declare_parameter("local_map_size", lidar_config.local_map_size);
    node.declare_parameter("local_map_voxel_size", lidar_config.local_map_voxel_size);

    node.get_parameter("local_map_size", lidar_config.local_map_size);
    node.get_parameter("local_map_voxel_size", lidar_config.local_map_voxel_size);

    // === Frame IDs ===
    node.declare_parameter("odom_frame_id", frame_config.odom_frame_id);
    node.declare_parameter("lidar_frame_id", frame_config.lidar_frame_id);

    node.get_parameter("odom_frame_id", frame_config.odom_frame_id);
    node.get_parameter("lidar_frame_id", frame_config.lidar_frame_id);

    // === Propagate to sub-configs ===
    feature_config.edge_threshold   = lidar_config.edge_curvature_threshold;
    feature_config.planar_threshold = lidar_config.planar_curvature_threshold;
    feature_config.max_edge_features_per_line =
        lidar_config.max_edge_features / feature_config.num_scan_lines;
    feature_config.max_planar_features_per_line =
        lidar_config.max_planar_features / feature_config.num_scan_lines;

    registration_config.is_2d_mode = true;  // Always 2D for ground robot

    map_config.max_keyframes     = lidar_config.local_map_size;
    map_config.edge_voxel_size   = lidar_config.local_map_voxel_size;
    map_config.planar_voxel_size = lidar_config.local_map_voxel_size * 2.0;
    map_config.full_voxel_size   = lidar_config.local_map_voxel_size * 1.5;

    // === Sanity checks ===
    if (lidar_config.keyframe_distance < 0.05)
    {
        RCLCPP_WARN(
            node.get_logger(),
            "keyframe_distance (%.3f) is very small, using 0.05",
            lidar_config.keyframe_distance);
        lidar_config.keyframe_distance = 0.05;
    }

    if (lidar_config.voxel_leaf_size < 0.01)
    {
        RCLCPP_WARN(
            node.get_logger(),
            "voxel_leaf_size (%.3f) is very small, using 0.01",
            lidar_config.voxel_leaf_size);
        lidar_config.voxel_leaf_size = 0.01;
    }
}

/**
 * @brief Log all loaded parameters
 */
inline void
    logParameters(const rclcpp::Logger& logger, const LidarConfig& config, const FrameConfig& frames)
{
    RCLCPP_INFO(
        logger,
        "LiDAR Odometry Configuration:\n"
        "  Sensor: %s\n"
        "  Keyframe: dist=%.3fm, rot=%.3frad\n"
        "  Voxel: %.3fm, min_points=%d\n"
        "  GICP: max_corr=%.2fm, max_iter=%d\n"
        "  Covariance: pos_std=%.3fm, rot_std=%.3frad\n"
        "  Features: enabled=%s, edges=%d, planars=%d\n"
        "  Local map: size=%d, voxel=%.2fm\n"
        "  Frames: odom=%s, lidar=%s",
        config.is_3d ? "3D" : "2D",
        config.keyframe_distance,
        config.keyframe_rotation,
        config.voxel_leaf_size,
        config.min_points_threshold,
        config.max_correspondence_dist,
        config.max_iterations,
        config.nominal_pos_std,
        config.nominal_rot_std,
        config.use_feature_registration ? "true" : "false",
        config.max_edge_features,
        config.max_planar_features,
        config.local_map_size,
        config.local_map_voxel_size,
        frames.odom_frame_id.c_str(),
        frames.lidar_frame_id.c_str());
}

}  // namespace parameter_loader
}  // namespace olive

#endif  // OLIVE_LIDAR_PARAMETER_LOADER_HPP_