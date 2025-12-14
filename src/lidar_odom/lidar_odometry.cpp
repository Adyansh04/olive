/**
 * @file lidar_odometry.cpp
 * @brief Implementation of LiDAR odometry with frame-to-frame and keyframe registration
 *
 * This implementation uses a two-tier registration approach:
 * 1. Frame-to-frame registration for continuous, high-frequency odometry
 * 2. Keyframe registration for drift correction and recovery
 *
 * Motion prediction using constant velocity model improves ICP convergence.
 */

#include "olive/lidar_odom/lidar_odometry.hpp"

#include <Eigen/src/Geometry/Quaternion.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>

#include <Eigen/Eigenvalues>
#include <chrono>
#include <cmath>
#include <memory>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <pcl/impl/point_types.hpp>

#include "olive/common/transform_utils.hpp"
#include "olive/common/types.hpp"
namespace olive
{

LidarOdometry::LidarOdometry(const rclcpp::NodeOptions& options)
  : LifecycleNode("lidar_odom_node", options)
{
    RCLCPP_INFO(get_logger(), "LidarOdometry node created");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LidarOdometry::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Configuring LidarOdometry");

    // Declare and get parameters
    declare_parameter("is_3d", config_.is_3d);
    declare_parameter("keyframe_distance", config_.keyframe_distance);
    declare_parameter("keyframe_rotation", config_.keyframe_rotation);
    declare_parameter("voxel_leaf_size", config_.voxel_leaf_size);
    declare_parameter("max_correspondence_dist", config_.max_correspondence_dist);
    declare_parameter("max_iterations", config_.max_iterations);
    declare_parameter("transformation_epsilon", config_.transformation_epsilon);
    declare_parameter("fitness_epsilon", config_.fitness_epsilon);
    declare_parameter("nominal_pos_std", config_.nominal_pos_std);
    declare_parameter("nominal_rot_std", config_.nominal_rot_std);
    declare_parameter("poor_fit_scale", config_.poor_fit_scale);
    declare_parameter("fitness_threshold", config_.fitness_threshold);
    declare_parameter("odom_frame_id", odom_frame_id_);
    declare_parameter("lidar_frame_id", lidar_frame_id_);
    declare_parameter("frame_fitness_threshold", config_.frame_fitness_threshold);
    declare_parameter("max_frame_distance", config_.max_frame_distance);
    declare_parameter("velocity_filter_alpha", config_.velocity_filter_alpha);
    declare_parameter("min_points_threshold", config_.min_points_threshold);
    declare_parameter("degeneracy_threshold", config_.degeneracy_threshold);

    // Feature-based registration parameters (NEW)
    declare_parameter("use_feature_registration", config_.use_feature_registration);
    declare_parameter("edge_curvature_threshold", config_.edge_curvature_threshold);
    declare_parameter("planar_curvature_threshold", config_.planar_curvature_threshold);
    declare_parameter("max_edge_features", config_.max_edge_features);
    declare_parameter("max_planar_features", config_.max_planar_features);
    declare_parameter("local_map_size", config_.local_map_size);
    declare_parameter("local_map_voxel_size", config_.local_map_voxel_size);

    get_parameter("is_3d", config_.is_3d);
    get_parameter("keyframe_distance", config_.keyframe_distance);
    get_parameter("keyframe_rotation", config_.keyframe_rotation);
    get_parameter("voxel_leaf_size", config_.voxel_leaf_size);
    get_parameter("max_correspondence_dist", config_.max_correspondence_dist);
    get_parameter("max_iterations", config_.max_iterations);
    get_parameter("transformation_epsilon", config_.transformation_epsilon);
    get_parameter("fitness_epsilon", config_.fitness_epsilon);
    get_parameter("nominal_pos_std", config_.nominal_pos_std);
    get_parameter("nominal_rot_std", config_.nominal_rot_std);
    get_parameter("poor_fit_scale", config_.poor_fit_scale);
    get_parameter("fitness_threshold", config_.fitness_threshold);
    get_parameter("odom_frame_id", odom_frame_id_);
    get_parameter("lidar_frame_id", lidar_frame_id_);
    get_parameter("frame_fitness_threshold", config_.frame_fitness_threshold);
    get_parameter("max_frame_distance", config_.max_frame_distance);
    get_parameter("velocity_filter_alpha", config_.velocity_filter_alpha);
    get_parameter("min_points_threshold", config_.min_points_threshold);
    get_parameter("degeneracy_threshold", config_.degeneracy_threshold);

    // Get feature-based registration parameters (NEW)
    get_parameter("use_feature_registration", config_.use_feature_registration);
    get_parameter("edge_curvature_threshold", config_.edge_curvature_threshold);
    get_parameter("planar_curvature_threshold", config_.planar_curvature_threshold);
    get_parameter("max_edge_features", config_.max_edge_features);
    get_parameter("max_planar_features", config_.max_planar_features);
    get_parameter("local_map_size", config_.local_map_size);
    get_parameter("local_map_voxel_size", config_.local_map_voxel_size);

    // Log all parameters with names and formatting
    RCLCPP_INFO(
        get_logger(),
        "LidarOdometry parameters:\n"
        "  is_3d: %s\n"
        "  keyframe_distance: %.4f m\n"
        "  keyframe_rotation: %.4f rad\n"
        "  voxel_leaf_size: %.4f m\n"
        "  max_correspondence_dist: %.4f m\n"
        "  max_iterations: %d\n"
        "  transformation_epsilon: %.2e\n"
        "  fitness_epsilon: %.2e\n"
        "  fitness_threshold: %.4f\n"
        "  frame_fitness_threshold: %.4f\n"
        "  max_frame_distance: %.4f m\n"
        "  velocity_filter_alpha: %.4f\n"
        "  min_points_threshold: %d\n"
        "  odom_frame_id: %s\n"
        "  lidar_frame_id: %s",
        config_.is_3d ? "true" : "false",
        config_.keyframe_distance,
        config_.keyframe_rotation,
        config_.voxel_leaf_size,
        config_.max_correspondence_dist,
        config_.max_iterations,
        config_.transformation_epsilon,
        config_.fitness_epsilon,
        config_.fitness_threshold,
        config_.frame_fitness_threshold,
        config_.max_frame_distance,
        config_.velocity_filter_alpha,
        config_.min_points_threshold,
        odom_frame_id_.c_str(),
        lidar_frame_id_.c_str());

    // Configure GICP
    gicp_.setMaxCorrespondenceDistance(config_.max_correspondence_dist);
    gicp_.setMaximumIterations(config_.max_iterations);
    gicp_.setTransformationEpsilon(config_.transformation_epsilon);
    gicp_.setEuclideanFitnessEpsilon(config_.fitness_epsilon);

    // GICP-specific settings for SPEED (reduced accuracy tradeoff)
    gicp_.setRotationEpsilon(1e-3);           // Relaxed from 1e-4
    gicp_.setCorrespondenceRandomness(5);     // Reduced from 20 - major speedup!
    gicp_.setMaximumOptimizerIterations(10);  // Limit optimizer iterations

    // Configure voxel filter
    voxel_filter_.setLeafSize(
        config_.voxel_leaf_size,
        config_.voxel_leaf_size,
        config_.voxel_leaf_size);

    // Initialize feature-based components (NEW)
    if (config_.use_feature_registration)
    {
        RCLCPP_INFO(get_logger(), "Initializing feature-based registration (LOAM-style)");

        // Configure feature extractor
        FeatureExtractionConfig fe_config;
        fe_config.edge_threshold = config_.edge_curvature_threshold;
        fe_config.planar_threshold = config_.planar_curvature_threshold;
        fe_config.max_edge_features_per_line = config_.max_edge_features / 16;  // Per scan line
        fe_config.max_planar_features_per_line = config_.max_planar_features / 16;
        fe_config.min_range = 0.5;
        fe_config.max_range = 50.0;
        // Ground filtering for ground robots - skip ground plane points
        fe_config.filter_ground = true;
        fe_config.ground_height_min = -0.5;  // Below sensor
        fe_config.ground_height_max = 0.15;   // Just above ground plane
        fe_config.sensor_height = 0.3;       // LiDAR height above ground
        feature_extractor_ = std::make_unique<FeatureExtractor>(fe_config);

        // Configure feature registration
        FeatureRegistrationConfig fr_config;
        fr_config.max_iterations = 10;
        fr_config.convergence_threshold = 1e-4;
        fr_config.edge_search_radius = 1.0;
        fr_config.planar_search_radius = 1.0;
        fr_config.max_edge_residual = 0.5;
        fr_config.max_planar_residual = 0.3;
        fr_config.edge_weight = 1.0;
        fr_config.planar_weight = 0.5;
        // Enable 2D mode for ground robots - constrain to x, y, yaw only
        fr_config.is_2d_mode = true;
        fr_config.ground_height_threshold = 0.3;
        feature_registration_ = std::make_unique<FeatureRegistration>(fr_config);

        // Configure local map
        LocalMapConfig lm_config;
        lm_config.max_keyframes = config_.local_map_size;
        lm_config.edge_voxel_size = config_.local_map_voxel_size;
        lm_config.planar_voxel_size = config_.local_map_voxel_size * 2.0;
        lm_config.update_distance = config_.keyframe_distance;
        lm_config.update_rotation = config_.keyframe_rotation;
        local_map_ = std::make_unique<LocalMap>(lm_config);

        RCLCPP_INFO(get_logger(), 
            "Feature extraction: edge_thresh=%.3f, planar_thresh=%.3f\n"
            "Local map: size=%d, voxel=%.2f m\n"
            "2D Mode: ENABLED (ground robot constraints)",
            config_.edge_curvature_threshold,
            config_.planar_curvature_threshold,
            config_.local_map_size,
            config_.local_map_voxel_size);
    }

    // Initialize covariance
    current_covariance_ = transform_utils::createDiagonalCovariance(
        config_.nominal_pos_std,
        config_.nominal_pos_std,
        config_.nominal_pos_std,
        config_.nominal_rot_std,
        config_.nominal_rot_std,
        config_.nominal_rot_std);

    // Initialize state
    current_pose_        = Pose3D();
    last_keyframe_pose_  = Pose3D();
    previous_pose_       = Pose3D();
    last_keyframe_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    previous_cloud_      = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    current_cloud_       = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // Reset tracking state
    has_previous_frame_        = false;
    consecutive_failures_      = 0;
    cumulative_drift_estimate_ = 0.0;
    linear_velocity_           = Eigen::Vector3d::Zero();
    angular_velocity_          = Eigen::Vector3d::Zero();

    // Create subscribers based on LiDAR type
    if (config_.is_3d)
    {
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "lidar/points",
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
                pointCloudCallback(msg);
            });
    }
    else
    {
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
                laserScanCallback(msg);
            });
    }

    // Create publisher
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("lidar/odom", 10);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LidarOdometry::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Activating LidarOdometry");
    odom_pub_->on_activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LidarOdometry::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Deactivating LidarOdometry");
    odom_pub_->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LidarOdometry::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Cleaning up LidarOdometry");
    cloud_sub_.reset();
    scan_sub_.reset();
    odom_pub_.reset();
    initialized_        = false;
    has_previous_frame_ = false;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void LidarOdometry::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    auto wall_time_start = std::chrono::high_resolution_clock::now();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    double msg_timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    double now_time = this->now().seconds();
    double msg_delay = now_time - msg_timestamp;
    
    RCLCPP_WARN(get_logger(), 
        "[CALLBACK] Received cloud: %zu pts, msg_stamp=%.3f, now=%.3f, delay=%.3f s",
        cloud->size(), msg_timestamp, now_time, msg_delay);
    
    processPointCloud(cloud, msg_timestamp);
    
    auto wall_time_end = std::chrono::high_resolution_clock::now();
    double processing_time_ms = std::chrono::duration<double, std::milli>(wall_time_end - wall_time_start).count();
    
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "[TIMING] Total callback: %.1f ms, cloud: %zu pts, msg_delay: %.3f s",
        processing_time_ms, cloud->size(), msg_delay);
}

void LidarOdometry::laserScanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
    auto   cloud     = laserScanToPointCloud(msg);
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    processPointCloud(cloud, timestamp);
}

Eigen::Matrix4f LidarOdometry::predictMotion(double dt)
{
    /**
     * Constant velocity motion model for GROUND ROBOT (2D):
     * x(t+dt) = x(t) + v * dt
     * R(t+dt) = R(t) * exp(omega * dt)
     *
     * This provides a good initial guess for ICP, significantly improving
     * convergence speed and robustness.
     * 
     * For ground robots: z = 0, only yaw rotation (no roll/pitch)
     */

    if (dt <= 0.0 || dt > 1.0)
    {
        // Invalid dt, return identity
        return Eigen::Matrix4f::Identity();
    }

    Eigen::Matrix4f prediction = Eigen::Matrix4f::Identity();

    // Predicted translation (x, y only for ground robot)
    prediction(0, 3) = static_cast<float>(linear_velocity_.x() * dt);
    prediction(1, 3) = static_cast<float>(linear_velocity_.y() * dt);
    prediction(2, 3) = 0.0f;  // Force z = 0 for ground robot

    // Predicted rotation - YAW ONLY for ground robot
    double yaw_rate = angular_velocity_.z();  // Only z-axis rotation (yaw)
    double yaw_angle = yaw_rate * dt;
    
    if (std::abs(yaw_angle) > 1e-6)
    {
        Eigen::AngleAxisd rotation(yaw_angle, Eigen::Vector3d::UnitZ());
        prediction.block<3, 3>(0, 0) = rotation.toRotationMatrix().cast<float>();
    }

    return prediction;
}

void LidarOdometry::updateVelocityEstimate(const Eigen::Matrix4f& delta_transform, double dt)
{
    /**
     * Update velocity estimate using exponential moving average (EMA):
     * v_new = alpha * v_measured + (1 - alpha) * v_old
     *
     * This smooths out noise in the velocity estimate while remaining
     * responsive to actual motion changes.
     */

    if (dt <= 0.001)
    {
        return;  // Avoid division by zero
    }

    // Extract translation
    Eigen::Vector3d measured_linear_vel(
        delta_transform(0, 3) / dt,
        delta_transform(1, 3) / dt,
        delta_transform(2, 3) / dt);

    // Extract rotation and compute angular velocity
    Eigen::Matrix3d   R = delta_transform.block<3, 3>(0, 0).cast<double>();
    Eigen::AngleAxisd angle_axis(R);
    Eigen::Vector3d   measured_angular_vel = angle_axis.axis() * angle_axis.angle() / dt;

    // Sanity check: reject unreasonable velocities
    double max_linear_vel  = config_.max_frame_distance / dt;
    double max_angular_vel = M_PI / dt;  // Max 180 deg in dt seconds

    if (measured_linear_vel.norm() > max_linear_vel || measured_angular_vel.norm() > max_angular_vel)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Rejecting unreasonable velocity: linear=%.2f m/s, angular=%.2f rad/s",
            measured_linear_vel.norm(),
            measured_angular_vel.norm());
        return;
    }

    // Apply EMA filter
    double alpha      = config_.velocity_filter_alpha;
    linear_velocity_  = alpha * measured_linear_vel + (1.0 - alpha) * linear_velocity_;
    angular_velocity_ = alpha * measured_angular_vel + (1.0 - alpha) * angular_velocity_;
}

RegistrationResult LidarOdometry::performRegistration(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target, const Eigen::Matrix4f& initial_guess)
{
    auto reg_start = std::chrono::high_resolution_clock::now();
    RegistrationResult result;

    // Check for sufficient points
    if (source->size() < static_cast<size_t>(config_.min_points_threshold) ||
        target->size() < static_cast<size_t>(config_.min_points_threshold))
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Insufficient points: source=%zu, target=%zu, min=%d",
            source->size(),
            target->size(),
            config_.min_points_threshold);
        return result;
    }

    RCLCPP_WARN(get_logger(), "[GICP] Starting: source=%zu, target=%zu pts",
        source->size(), target->size());

    // Perform GICP alignment
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

    gicp_.setInputSource(source);
    gicp_.setInputTarget(target);
    gicp_.align(*aligned, initial_guess);

    auto reg_end = std::chrono::high_resolution_clock::now();
    double reg_time_ms = std::chrono::duration<double, std::milli>(reg_end - reg_start).count();

    result.transformation = gicp_.getFinalTransformation();
    result.fitness_score  = gicp_.getFitnessScore();
    result.converged      = gicp_.hasConverged();
    
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[GICP] Done in %.1f ms: converged=%d, fitness=%.4f, src=%zu, tgt=%zu",
        reg_time_ms, result.converged, result.fitness_score, source->size(), target->size());

    // Estimate number of correspondences (approximate)
    result.num_correspondences = static_cast<int>(aligned->size());

    // Check for degeneracy
    result.degenerate = checkDegeneracy(source);

    return result;
}

RegistrationResult LidarOdometry::performFeatureRegistration(
    const ExtractedFeatures& current_features,
    const Eigen::Matrix4f& initial_guess)
{
    /**
     * Feature-based registration using LOAM-style point-to-edge and point-to-plane.
     * 
     * This method provides better rotation estimation than standard GICP by:
     * 1. Extracting edge features (high curvature) that constrain rotation strongly
     * 2. Extracting planar features (low curvature) that constrain translation
     * 3. Using separate residual types with appropriate Jacobians
     */
    auto reg_start = std::chrono::high_resolution_clock::now();
    RegistrationResult result;

    // Check if local map is ready
    if (!local_map_ || !local_map_->isReady())
    {
        RCLCPP_DEBUG(get_logger(), "Local map not ready for feature registration");
        return result;
    }

    // Check if we have features
    if (!current_features.hasFeatures())
    {
        RCLCPP_DEBUG(get_logger(), "No features extracted from current scan");
        return result;
    }

    // Set target maps
    feature_registration_->setTargetEdges(local_map_->getEdgeMap());
    feature_registration_->setTargetPlanars(local_map_->getPlanarMap());

    // Perform feature-based alignment
    auto feature_result = feature_registration_->align(
        current_features.edge_points,
        current_features.planar_points,
        initial_guess);

    auto reg_end = std::chrono::high_resolution_clock::now();
    double reg_time_ms = std::chrono::duration<double, std::milli>(reg_end - reg_start).count();

    // Convert to RegistrationResult
    result.transformation = feature_result.transformation;
    result.fitness_score = feature_result.overall_fitness;
    result.converged = feature_result.converged;
    result.degenerate = feature_result.degenerate;
    result.num_correspondences = feature_result.edge_correspondences + 
                                  feature_result.planar_correspondences;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[FEATURE] Done in %.1f ms: converged=%d, fitness=%.4f, "
        "edge_corr=%d, planar_corr=%d, iter=%d",
        reg_time_ms, result.converged, result.fitness_score,
        feature_result.edge_correspondences, feature_result.planar_correspondences,
        feature_result.iterations);

    return result;
}

bool LidarOdometry::checkDegeneracy(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    /**
     * Degeneracy detection based on point distribution analysis.
     *
     * In degenerate cases (e.g., long corridor), the point cloud has
     * insufficient constraints in certain directions. We detect this
     * by analyzing the eigenvalues of the point covariance matrix.
     *
     * If the ratio of largest to smallest eigenvalue exceeds a threshold,
     * the registration is considered degenerate in that direction.
     */

    if (cloud->size() < 10)
    {
        return true;  // Too few points
    }

    // Compute centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);

    // Compute covariance matrix
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);

    // Compute eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    Eigen::Vector3f                                eigenvalues = solver.eigenvalues();

    // Sort eigenvalues (should already be sorted, but ensure)
    std::sort(eigenvalues.data(), eigenvalues.data() + 3);

    // Check ratio of largest to smallest eigenvalue
    float min_eigenvalue = std::max(eigenvalues(0), 1e-6f);
    float max_eigenvalue = eigenvalues(2);
    float ratio          = max_eigenvalue / min_eigenvalue;

    if (ratio > config_.degeneracy_threshold)
    {
        RCLCPP_WARN(
            get_logger(),
            "Degeneracy detected: eigenvalue ratio = %.2f (threshold = %.2f)",
            ratio,
            config_.degeneracy_threshold);
        return true;
    }

    return false;
}

void LidarOdometry::processPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double timestamp)
{
    auto process_start = std::chrono::high_resolution_clock::now();
    
    // Filter the incoming cloud
    auto filter_start = std::chrono::high_resolution_clock::now();
    auto filtered_cloud = filterPointCloud(cloud);
    auto filter_end = std::chrono::high_resolution_clock::now();
    double filter_time_ms = std::chrono::duration<double, std::milli>(filter_end - filter_start).count();
    
    RCLCPP_WARN(get_logger(), "[FILTER] %zu -> %zu pts in %.1f ms",
        cloud->size(), filtered_cloud->size(), filter_time_ms);

    if (filtered_cloud->empty() ||
        filtered_cloud->size() < static_cast<size_t>(config_.min_points_threshold))
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Filtered cloud too small: %zu points",
            filtered_cloud->size());
        return;
    }

    // Initialization: first cloud
    if (!initialized_)
    {
        last_keyframe_cloud_ = filtered_cloud;
        previous_cloud_      = filtered_cloud;
        last_keyframe_pose_  = current_pose_;
        previous_pose_       = current_pose_;
        last_timestamp_      = timestamp;
        has_previous_frame_  = true;
        initialized_         = true;

        // Initialize feature-based components with first scan
        if (config_.use_feature_registration && feature_extractor_ && local_map_)
        {
            auto features = feature_extractor_->extractUnorganized(filtered_cloud, timestamp);
            local_map_->addKeyframe(features, current_pose_);
            RCLCPP_INFO(get_logger(), 
                "Feature extraction initialized: %zu edges, %zu planars",
                features.edge_points->size(), features.planar_points->size());
        }

        current_pose_.timestamp = timestamp;
        publishOdometry(timestamp);

        RCLCPP_INFO(
            get_logger(),
            "LiDAR odometry initialized with %zu points",
            filtered_cloud->size());
        return;
    }

    // Calculate time delta
    double dt = timestamp - last_timestamp_;
    
    RCLCPP_WARN(get_logger(), 
        "[TIME] msg_stamp=%.3f, last_stamp=%.3f, dt=%.4f s",
        timestamp, last_timestamp_, dt);
    
    if (dt <= 0.0)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Non-positive time delta: %.4f s (msg=%.3f, last=%.3f), skipping",
            dt, timestamp, last_timestamp_);
        return;
    }
    if (dt > 1.0)
    {
        RCLCPP_WARN(get_logger(), 
            "Large time gap: %.2f s (msg_stamp=%.3f, last_stamp=%.3f) - likely slow processing!",
            dt, timestamp, last_timestamp_);
        linear_velocity_  = Eigen::Vector3d::Zero();
        angular_velocity_ = Eigen::Vector3d::Zero();
        dt                = 0.1;  // Use nominal dt for prediction
    }

    // Predict motion for initial guess
    Eigen::Matrix4f motion_prediction = predictMotion(dt);

    bool               registration_success = false;
    RegistrationResult result;
    Pose3D             reference_pose;

    // Extract features for feature-based registration
    ExtractedFeatures current_features;
    if (config_.use_feature_registration && feature_extractor_)
    {
        auto feature_start = std::chrono::high_resolution_clock::now();
        current_features = feature_extractor_->extractUnorganized(filtered_cloud, timestamp);
        auto feature_end = std::chrono::high_resolution_clock::now();
        double feature_time_ms = std::chrono::duration<double, std::milli>(feature_end - feature_start).count();

        RCLCPP_DEBUG(get_logger(), 
            "[FEATURES] Extracted %zu edges, %zu planars in %.1f ms",
            current_features.edge_points->size(),
            current_features.planar_points->size(),
            feature_time_ms);
    }

    // Strategy 0: Feature-based scan-to-map registration (NEW, best for rotation)
    if (config_.use_feature_registration && 
        local_map_ && local_map_->isReady() && 
        current_features.hasFeatures())
    {
        result = performFeatureRegistration(current_features, motion_prediction);

        if (result.converged && result.fitness_score < config_.fitness_threshold)
        {
            double translation_mag = result.transformation.block<3, 1>(0, 3).norm();

            if (translation_mag < config_.max_frame_distance * 2.0)  // More lenient for scan-to-map
            {
                registration_success = true;
                reference_pose = Pose3D();  // Feature registration gives global pose delta
                reference_pose.position = Eigen::Vector3d::Zero();
                reference_pose.orientation = Eigen::Quaterniond::Identity();
                consecutive_failures_ = 0;

                updateVelocityEstimate(result.transformation, dt);

                RCLCPP_DEBUG(get_logger(),
                    "Feature-based: fitness=%.4f, translation=%.4f m",
                    result.fitness_score, translation_mag);
            }
        }
    }

    // Strategy 1: Frame-to-frame GICP registration (fallback)
    if (!registration_success)
    {
        RCLCPP_DEBUG(get_logger(), "[STRATEGY1] Frame-to-frame: has_prev=%d, prev_size=%zu",
            has_previous_frame_, previous_cloud_ ? previous_cloud_->size() : 0);
        
        if (has_previous_frame_ &&
            previous_cloud_->size() >= static_cast<size_t>(config_.min_points_threshold))
        {
            result = performRegistration(filtered_cloud, previous_cloud_, motion_prediction);
            
            RCLCPP_DEBUG(get_logger(), 
                "[STRATEGY1] Result: converged=%d, fitness=%.4f (thresh=%.4f)",
                result.converged, result.fitness_score, config_.frame_fitness_threshold);

            if (result.converged && result.fitness_score < config_.frame_fitness_threshold)
            {
                // Validate the transformation magnitude
                double translation_mag = result.transformation.block<3, 1>(0, 3).norm();

                if (translation_mag < config_.max_frame_distance)
                {
                    registration_success  = true;
                    reference_pose        = previous_pose_;
                    consecutive_failures_ = 0;

                    // Update velocity estimate for next prediction
                    updateVelocityEstimate(result.transformation, dt);

                    RCLCPP_DEBUG(
                        get_logger(),
                        "Frame-to-frame: fitness=%.4f, translation=%.4f m",
                        result.fitness_score,
                        translation_mag);
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(),
                        *get_clock(),
                        500,
                        "Frame-to-frame translation too large: %.2f m (max: %.2f m)",
                        translation_mag,
                        config_.max_frame_distance);
                }
            }
        }
    }

    // Strategy 2: Keyframe registration (fallback, more robust)
    if (!registration_success)
    {
        RCLCPP_WARN(get_logger(), "Falling back to keyframe registration");

        // For keyframe registration, compute proper initial guess
        // T_keyframe_to_current = T_keyframe_to_previous * T_previous_to_current (predicted)

        // Compute relative pose from keyframe to previous
        Pose3D delta_from_keyframe =
            transform_utils::relativePose(last_keyframe_pose_, previous_pose_);

        // Convert motion prediction to Pose3D
        Pose3D predicted_motion;
        predicted_motion.position = Eigen::Vector3d(
            motion_prediction(0, 3),
            motion_prediction(1, 3),
            motion_prediction(2, 3));
        predicted_motion.orientation =
            Eigen::Quaterniond(motion_prediction.block<3, 3>(0, 0).cast<double>());
        predicted_motion.orientation.normalize();

        // Properly compose: T_kf_curr = T_kf_prev * T_prev_curr
        Pose3D expected_delta =
            transform_utils::composePoses(delta_from_keyframe, predicted_motion);

        // Convert back to Matrix4f for GICP initial guess
        Eigen::Matrix4f keyframe_guess = Eigen::Matrix4f::Identity();
        keyframe_guess.block<3, 3>(0, 0) =
            expected_delta.orientation.toRotationMatrix().cast<float>();
        keyframe_guess.block<3, 1>(0, 3) = expected_delta.position.cast<float>();

        result = performRegistration(filtered_cloud, last_keyframe_cloud_, keyframe_guess);

        if (result.converged && result.fitness_score < config_.fitness_threshold)
        {
            registration_success  = true;
            reference_pose        = last_keyframe_pose_;
            consecutive_failures_ = 0;

            RCLCPP_WARN(get_logger(), "Keyframe registration: fitness=%.4f", result.fitness_score);
        }
        else
        {
            consecutive_failures_++;
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                500,
                "Keyframe registration failed: converged=%d, fitness=%.4f, failures=%d",
                result.converged,
                result.fitness_score,
                consecutive_failures_);
        }
    }

    // Update pose based on registration result
    if (registration_success)
    {
        // Extract transformation components
        Eigen::Matrix3d R = result.transformation.block<3, 3>(0, 0).cast<double>();
        Eigen::Vector3d t = result.transformation.block<3, 1>(0, 3).cast<double>();

        // Create delta pose
        Pose3D delta_pose;
        delta_pose.position    = t;
        delta_pose.orientation = Eigen::Quaterniond(R);
        delta_pose.orientation.normalize();

        // Compose with reference pose to get current pose
        current_pose_ = transform_utils::composePoses(reference_pose, delta_pose);

        // ENFORCE 2D CONSTRAINT for ground robots
        // Keep z at 0, remove roll and pitch from orientation
        current_pose_.position.z() = 0.0;
        
        // Extract yaw using atan2 (avoids euler angle 180° discontinuities)
        // For rotation matrix R_pose: yaw = atan2(R(1,0), R(0,0))
        Eigen::Matrix3d R_pose = current_pose_.orientation.toRotationMatrix();
        double yaw = std::atan2(R_pose(1, 0), R_pose(0, 0));
        current_pose_.orientation = Eigen::Quaterniond(
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        current_pose_.orientation.normalize();

        // Update covariance based on fitness
        updateCovariance(result.fitness_score, result.degenerate);
    }
    else
    {
        // Registration failed - use motion prediction (dead reckoning)
        Pose3D predicted_delta;
        predicted_delta.position = Eigen::Vector3d(
            motion_prediction(0, 3),
            motion_prediction(1, 3),
            0.0);  // Force z = 0 for ground robot
        predicted_delta.orientation =
            Eigen::Quaterniond(motion_prediction.block<3, 3>(0, 0).cast<double>());
        predicted_delta.orientation.normalize();

        current_pose_ = transform_utils::composePoses(previous_pose_, predicted_delta);

        // ENFORCE 2D CONSTRAINT for ground robots
        current_pose_.position.z() = 0.0;
        
        // Extract yaw using atan2 (avoids euler angle 180° discontinuities)
        Eigen::Matrix3d R_pose = current_pose_.orientation.toRotationMatrix();
        double yaw = std::atan2(R_pose(1, 0), R_pose(0, 0));
        current_pose_.orientation = Eigen::Quaterniond(
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        current_pose_.orientation.normalize();

        // FIX: Reset to base covariance then scale (don't compound)
        double failure_scale = std::min(10.0, 1.0 + consecutive_failures_ * 2.0);
        current_covariance_  = transform_utils::createDiagonalCovariance(
            config_.nominal_pos_std * failure_scale,
            config_.nominal_pos_std * failure_scale,
            config_.nominal_pos_std * failure_scale,
            config_.nominal_rot_std * failure_scale,
            config_.nominal_rot_std * failure_scale,
            config_.nominal_rot_std * failure_scale);

        // Track cumulative drift
        cumulative_drift_estimate_ += predicted_delta.position.norm() * 0.1;  // 10% estimated drift

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            500,
            "Using dead reckoning, cumulative drift estimate: %.3f m",
            cumulative_drift_estimate_);
    }

    current_pose_.timestamp = timestamp;

    // Update keyframe if needed
    if (shouldCreateKeyframe(current_pose_, last_keyframe_pose_))
    {
        last_keyframe_cloud_       = filtered_cloud;
        last_keyframe_pose_        = current_pose_;
        cumulative_drift_estimate_ = 0.0;  // Reset drift estimate on keyframe

        // Update local map with new keyframe features
        if (config_.use_feature_registration && local_map_ && current_features.hasFeatures())
        {
            local_map_->addKeyframe(current_features, current_pose_);
            RCLCPP_DEBUG(get_logger(),
                "Local map updated: %zu keyframes, %zu edges, %zu planars",
                local_map_->numKeyframes(),
                local_map_->numEdgePoints(),
                local_map_->numPlanarPoints());
        }

        RCLCPP_INFO(
            get_logger(),
            "Keyframe updated at position (%.2f, %.2f, %.2f)",
            current_pose_.position.x(),
            current_pose_.position.y(),
            current_pose_.position.z());
    }

    // Always update previous frame
    previous_cloud_     = filtered_cloud;
    previous_pose_      = current_pose_;
    last_timestamp_     = timestamp;
    has_previous_frame_ = true;

    // Publish odometry
    publishOdometry(timestamp);
    
    auto process_end = std::chrono::high_resolution_clock::now();
    double total_time_ms = std::chrono::duration<double, std::milli>(process_end - process_start).count();
    
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "[SUMMARY] Process: %.1f ms, pose=(%.2f, %.2f, %.2f), pts=%zu, success=%d",
        total_time_ms,
        current_pose_.position.x(),
        current_pose_.position.y(),
        current_pose_.position.z(),
        filtered_cloud->size(),
        registration_success);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
LidarOdometry::filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());

    // Remove NaN and infinite points first
    pcl::PointCloud<pcl::PointXYZ>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    clean_cloud->reserve(cloud->size());

    for (const auto& pt : cloud->points)
    {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
        {
            // Remove points too close (sensor noise) or too far
            float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (dist > 0.5f && dist < 30.0f)  // Tighter range: 0.5m to 30m
            {
                clean_cloud->push_back(pt);
            }
        }
    }

    if (clean_cloud->empty())
    {
        return filtered;
    }

    // Apply voxel filter
    voxel_filter_.setInputCloud(clean_cloud);
    voxel_filter_.filter(*filtered);

    // Additional random downsampling if still too many points
    // Target: ~500-800 points for fast GICP
    const size_t MAX_POINTS = 800;
    if (filtered->size() > MAX_POINTS)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        downsampled->reserve(MAX_POINTS);
        
        // Uniform sampling
        size_t step = filtered->size() / MAX_POINTS;
        for (size_t i = 0; i < filtered->size() && downsampled->size() < MAX_POINTS; i += step)
        {
            downsampled->push_back(filtered->points[i]);
        }
        return downsampled;
    }

    return filtered;
}

bool LidarOdometry::shouldCreateKeyframe(const Pose3D& current, const Pose3D& keyframe)
{
    // Calculate translation distance
    double distance = (current.position - keyframe.position).norm();

    // Calculate rotation difference
    Eigen::Quaterniond delta_q  = keyframe.orientation.inverse() * current.orientation;
    double             rotation = 2.0 * std::acos(std::min(1.0, std::abs(delta_q.w())));

    return (distance >= config_.keyframe_distance) || (rotation >= config_.keyframe_rotation);
}

Eigen::Matrix4f LidarOdometry::estimateTransformation(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target, double& fitness_score)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

    gicp_.setInputSource(source);
    gicp_.setInputTarget(target);
    gicp_.align(*aligned);

    fitness_score = gicp_.getFitnessScore();

    if (!gicp_.hasConverged())
    {
        RCLCPP_WARN(get_logger(), "GICP did not converge, fitness: %f", fitness_score);
    }

    return gicp_.getFinalTransformation();
}

void LidarOdometry::updateCovariance(double fitness_score, bool degenerate)
{
    /**
     * Covariance estimation based on registration quality.
     *
     * The covariance is scaled based on:
     * 1. Fitness score (higher = more uncertainty)
     * 2. Degeneracy status (degenerate = higher uncertainty in affected directions)
     *
     * For a more rigorous approach, one could compute the Hessian of the
     * ICP cost function, but this approximation works well in practice.
     */

    double scale = 1.0;

    if (fitness_score > config_.fitness_threshold)
    {
        // Poor fitness - scale quadratically with fitness excess
        double excess = (fitness_score - config_.fitness_threshold) / config_.fitness_threshold;
        scale         = config_.poor_fit_scale * (1.0 + excess * excess);

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Poor LiDAR fitness: %.4f - inflating covariance by %.1fx",
            fitness_score,
            scale);
    }

    if (degenerate)
    {
        // Additional inflation for degeneracy
        scale *= 2.0;
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Degenerate scan - additional covariance inflation");
    }

    current_covariance_ = transform_utils::createDiagonalCovariance(
        config_.nominal_pos_std * scale,
        config_.nominal_pos_std * scale,
        config_.nominal_pos_std * scale,
        config_.nominal_rot_std * scale,
        config_.nominal_rot_std * scale,
        config_.nominal_rot_std * scale);
}

void LidarOdometry::publishOdometry(double timestamp)
{
    if (!odom_pub_->is_activated())
    {
        return;
    }

    nav_msgs::msg::Odometry odom_msg;

    // Header
    odom_msg.header.stamp.sec = static_cast<int32_t>(timestamp);
    odom_msg.header.stamp.nanosec =
        static_cast<uint32_t>((timestamp - odom_msg.header.stamp.sec) * 1e9);
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id  = lidar_frame_id_;

    // Pose
    odom_msg.pose.pose.position.x    = current_pose_.position.x();
    odom_msg.pose.pose.position.y    = current_pose_.position.y();
    odom_msg.pose.pose.position.z    = current_pose_.position.z();
    odom_msg.pose.pose.orientation.w = current_pose_.orientation.w();
    odom_msg.pose.pose.orientation.x = current_pose_.orientation.x();
    odom_msg.pose.pose.orientation.y = current_pose_.orientation.y();
    odom_msg.pose.pose.orientation.z = current_pose_.orientation.z();

    // Pose covariance (6x6, row-major)
    for (size_t i = 0; i < 6; ++i)
    {
        for (size_t j = 0; j < 6; ++j)
        {
            odom_msg.pose.covariance[i * 6 + j] = current_covariance_(i, j);
        }
    }

    // Twist (velocity in child frame)
    odom_msg.twist.twist.linear.x  = linear_velocity_.x();
    odom_msg.twist.twist.linear.y  = linear_velocity_.y();
    odom_msg.twist.twist.linear.z  = linear_velocity_.z();
    odom_msg.twist.twist.angular.x = angular_velocity_.x();
    odom_msg.twist.twist.angular.y = angular_velocity_.y();
    odom_msg.twist.twist.angular.z = angular_velocity_.z();

    // Twist covariance (simplified - use same as pose scaled)
    double vel_cov_scale = 10.0;  // Velocity is less certain
    for (size_t i = 0; i < 6; ++i)
    {
        for (size_t j = 0; j < 6; ++j)
        {
            odom_msg.twist.covariance[i * 6 + j] = current_covariance_(i, j) * vel_cov_scale;
        }
    }

    odom_pub_->publish(odom_msg);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
LidarOdometry::laserScanToPointCloud(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        float range = scan->ranges[i];
        if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max)
        {
            float         angle = scan->angle_min + i * scan->angle_increment;
            pcl::PointXYZ point;
            point.x = range * std::cos(angle);
            point.y = range * std::sin(angle);
            point.z = 0.0;
            cloud->points.push_back(point);
        }
    }

    cloud->width    = cloud->points.size();
    cloud->height   = 1;
    cloud->is_dense = false;

    return cloud;
}

}  // namespace olive

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(olive::LidarOdometry)
