/**
 * @file lidar_odometry.cpp
 * @brief Implementation of LiDAR odometry with two-tier registration
 *
 * Pipeline:
 *   PointCloud -> Filter -> Extract Features -> Align to Map -> Update Pose -> Publish
 *
 * Registration Strategy:
 * 1. Primary: Feature-based scan-to-map using edge/planar features
 *    - Edge features (high curvature) constrain rotation
 *    - Planar features (low curvature) constrain translation
 * 2. Fallback: GICP when features are insufficient
 *
 * All poses are constrained to (x, y, yaw) for ground robots.
 */

#include "olive/lidar_odom/lidar_odometry.hpp"

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <Eigen/Eigenvalues>
#include <chrono>

#include "olive/common/transform_utils.hpp"
#include "olive/lidar_odom/lidar_utils.hpp"
#include "olive/lidar_odom/parameter_loader.hpp"
namespace olive
{

LidarOdometry::LidarOdometry(const rclcpp::NodeOptions& options)
  : LifecycleNode("lidar_odom_node", options)
{
    RCLCPP_INFO(get_logger(), "LidarOdometry node created");
}

// ============================================================================
// Lifecycle Callbacks
// ============================================================================

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    LidarOdometry::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Configuring LidarOdometry");

    // Load all parameters using centralized loader
    parameter_loader::loadAllParameters(
        *this,
        lidar_config_,
        frame_config_,
        feature_config_,
        registration_config_,
        map_config_);

    parameter_loader::logParameters(get_logger(), lidar_config_, frame_config_);

    // Configure GICP for fallback registration
    gicp_.setMaxCorrespondenceDistance(lidar_config_.max_correspondence_dist);
    gicp_.setMaximumIterations(lidar_config_.max_iterations);
    gicp_.setTransformationEpsilon(lidar_config_.transformation_epsilon);
    gicp_.setEuclideanFitnessEpsilon(lidar_config_.fitness_epsilon);
    gicp_.setRotationEpsilon(1e-3);
    gicp_.setCorrespondenceRandomness(5);
    gicp_.setMaximumOptimizerIterations(10);

    // Configure voxel filter
    voxel_filter_.setLeafSize(
        static_cast<float>(lidar_config_.voxel_leaf_size),
        static_cast<float>(lidar_config_.voxel_leaf_size),
        static_cast<float>(lidar_config_.voxel_leaf_size));

    // Initialize feature-based components
    if (lidar_config_.use_feature_registration)
    {
        feature_extractor_    = std::make_unique<FeatureExtractor>(feature_config_);
        feature_registration_ = std::make_unique<FeatureRegistration>(registration_config_);
        local_map_            = std::make_unique<LocalMap>(map_config_);

        RCLCPP_INFO(get_logger(), "Feature-based registration enabled");
    }

    // Initialize covariance
    current_covariance_ =
        lidar_utils::createCovariance(lidar_config_.nominal_pos_std, lidar_config_.nominal_rot_std);

    // Initialize state
    current_pose_         = Pose3D();
    previous_pose_        = Pose3D();
    last_keyframe_pose_   = Pose3D();
    previous_orientation_ = Eigen::Quaterniond::Identity();

    last_keyframe_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    previous_cloud_      = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // Reset tracking state
    initialized_               = false;
    has_previous_frame_        = false;
    consecutive_failures_      = 0;
    cumulative_drift_estimate_ = 0.0;
    linear_velocity_           = Eigen::Vector3d::Zero();
    angular_velocity_          = Eigen::Vector3d::Zero();
    last_timestamp_            = 0.0;

    // Pre-initialize reusable odometry message
    odom_msg_.header.frame_id = frame_config_.odom_frame_id;
    odom_msg_.child_frame_id  = frame_config_.lidar_frame_id;

    // Create subscribers
    if (lidar_config_.is_3d)
    {
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "lidar/points",
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
                pointCloudCallback(msg);
            });
        RCLCPP_INFO(get_logger(), "Subscribed to 3D point cloud: lidar/points");
    }
    else
    {
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
                laserScanCallback(msg);
            });
        RCLCPP_INFO(get_logger(), "Subscribed to 2D laser scan: scan");
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

    feature_extractor_.reset();
    feature_registration_.reset();
    local_map_.reset();

    last_keyframe_cloud_.reset();
    previous_cloud_.reset();

    initialized_        = false;
    has_previous_frame_ = false;

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// ============================================================================
// Sensor Callbacks
// ============================================================================

void LidarOdometry::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*msg, *cloud);

    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    processPointCloud(cloud, timestamp);
}

void LidarOdometry::laserScanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
    auto cloud = laserScanToPointCloud(msg);
    if (!cloud || cloud->empty())
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Failed to convert laser scan");
        return;
    }

    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    processPointCloud(cloud, timestamp);
}

// ============================================================================
// Main Processing Pipeline
// ============================================================================

void LidarOdometry::processPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double                                     timestamp)
{
    auto pipeline_start = std::chrono::high_resolution_clock::now();

    // Step 1: Filter point cloud
    auto filtered_cloud = filterPointCloud(cloud);
    if (!filtered_cloud ||
        filtered_cloud->size() < static_cast<size_t>(lidar_config_.min_points_threshold))
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Insufficient points after filtering: %zu",
            filtered_cloud ? filtered_cloud->size() : 0);
        return;
    }

    // Step 2: Extract features (if enabled)
    ExtractedFeatures features;
    if (lidar_config_.use_feature_registration && feature_extractor_)
    {
        features = extractFeatures(filtered_cloud, timestamp);
    }

    // Step 3: Handle initialization
    if (!initialized_)
    {
        // First frame - initialize state
        last_keyframe_cloud_  = filtered_cloud;
        previous_cloud_       = filtered_cloud;
        last_keyframe_pose_   = current_pose_;
        previous_pose_        = current_pose_;
        last_timestamp_       = timestamp;
        has_previous_frame_   = true;
        initialized_          = true;
        previous_orientation_ = current_pose_.orientation;

        // Initialize local map with first keyframe
        if (local_map_ && features.hasFeatures())
        {
            local_map_->addKeyframe(features, current_pose_);

            // Set initial targets for feature registration
            if (feature_registration_)
            {
                feature_registration_->setTargetEdges(local_map_->getEdgeMap());
                feature_registration_->setTargetPlanars(local_map_->getPlanarMap());
            }
        }

        current_pose_.timestamp = timestamp;
        publishOdometry(timestamp);

        RCLCPP_INFO(
            get_logger(),
            "LiDAR odometry initialized with %zu points, %zu edges, %zu planars",
            filtered_cloud->size(),
            features.numEdges(),
            features.numPlanars());
        return;
    }

    // Step 4: Compute time delta and initial guess
    double dt = timestamp - last_timestamp_;
    if (dt <= 0.0)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Non-positive time delta: %.4f s", dt);
        return;
    }

    if (dt > 1.0)
    {
        RCLCPP_WARN(get_logger(), "Large time gap: %.2f s, resetting velocity", dt);
        linear_velocity_  = Eigen::Vector3d::Zero();
        angular_velocity_ = Eigen::Vector3d::Zero();
        dt                = 0.1;  // Use nominal dt
    }

    Eigen::Matrix4f initial_guess = computeInitialGuess(dt);

    // Step 5: Registration
    RegistrationResult result;
    bool               registration_success = false;
    Pose3D             reference_pose       = previous_pose_;

    // Debug: Log feature registration preconditions
    bool feat_reg_enabled = lidar_config_.use_feature_registration;
    bool local_map_exists = (local_map_ != nullptr);
    bool local_map_ready = local_map_exists && local_map_->isReady();
    
    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "[DEBUG] Feature reg preconditions: enabled=%d, map_exists=%d, map_ready=%d, "
        "features: edges=%zu, planars=%zu, map_edges=%zu, map_planars=%zu",
        feat_reg_enabled,
        local_map_exists,
        local_map_ready,
        features.numEdges(),
        features.numPlanars(),
        local_map_exists ? local_map_->numEdgePoints() : 0,
        local_map_exists ? local_map_->numPlanarPoints() : 0);

    // Try feature-based registration first (primary method)
    if (feat_reg_enabled && local_map_ready)
    {
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "[DEBUG] Attempting feature registration...");
            
        result = tryFeatureAlign(features, initial_guess);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "[DEBUG] Feature align result: converged=%d, fitness=%.4f, threshold=%.4f, "
            "degenerate=%d, edge_rmse=%.4f, plane_rmse=%.4f",
            result.converged,
            result.fitness_score,
            lidar_config_.fitness_threshold,
            result.degenerate,
            result.edge_rmse,
            result.plane_rmse);

        if (result.converged && result.fitness_score < lidar_config_.fitness_threshold)
        {
            registration_success = true;
            reference_pose       = last_keyframe_pose_;  // Feature registration is relative to map

            RCLCPP_INFO(
                get_logger(),
                "Feature registration SUCCESS: fitness=%.4f, edges=%d, planars=%d",
                result.fitness_score,
                result.inliers_edge,
                result.inliers_plane);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "[DEBUG] Feature registration FAILED: converged=%d, fitness=%.4f > threshold=%.4f",
                result.converged,
                result.fitness_score,
                lidar_config_.fitness_threshold);
        }
    }
    else if (feat_reg_enabled && !local_map_ready)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "[DEBUG] Feature reg enabled but local map NOT ready! keyframes=%zu",
            local_map_exists ? local_map_->numKeyframes() : 0);
    }

    // Fallback to GICP if feature registration failed
    if (!registration_success && has_previous_frame_)
    {
        result = alignFallbackGICP(filtered_cloud, previous_cloud_, initial_guess);

        if (result.converged && result.fitness_score < lidar_config_.frame_fitness_threshold)
        {
            // Validate motion magnitude
            double translation_mag = result.transformation.block<3, 1>(0, 3).norm();

            if (translation_mag < lidar_config_.max_frame_distance)
            {
                registration_success = true;
                reference_pose       = previous_pose_;
                result.method        = RegistrationResult::Method::GICP;

                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "GICP fallback: fitness=%.4f, translation=%.3f m",
                    result.fitness_score,
                    translation_mag);
            }
            else
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    500,
                    "GICP translation too large: %.2f m",
                    translation_mag);
            }
        }
    }

    // Step 6: Update pose
    if (registration_success)
    {
        updatePoseFromRegistration(result, reference_pose);
        updateVelocityEstimate(result.transformation, dt);
        consecutive_failures_ = 0;
    }
    else
    {
        // Dead reckoning fallback
        updatePoseFromPrediction(initial_guess);
        consecutive_failures_++;

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            500,
            "Registration failed, using prediction. Failures: %d",
            consecutive_failures_);
    }

    // Step 7: Enforce 2D constraint for ground robot
    lidar_utils::enforce2DConstraint(current_pose_);

    // Step 8: Ensure quaternion consistency
    current_pose_.orientation =
        lidar_utils::ensureQuaternionConsistency(current_pose_.orientation, previous_orientation_);
    previous_orientation_ = current_pose_.orientation;

    current_pose_.timestamp = timestamp;

    // Step 9: Update covariance
    updateCovariance(result.fitness_score, result.degenerate);

    // Step 10: Maybe add keyframe
    maybeAddKeyframe(features, current_pose_);

    // Step 11: Update state for next frame
    previous_cloud_     = filtered_cloud;
    previous_pose_      = current_pose_;
    last_timestamp_     = timestamp;
    has_previous_frame_ = true;

    // Step 12: Publish
    publishOdometry(timestamp);

    // Log timing (throttled)
    auto   pipeline_end = std::chrono::high_resolution_clock::now();
    double pipeline_ms =
        std::chrono::duration<double, std::milli>(pipeline_end - pipeline_start).count();

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Pipeline: %.1f ms, pos=(%.2f, %.2f), yaw=%.2f deg, method=%s",
        pipeline_ms,
        current_pose_.position.x(),
        current_pose_.position.y(),
        lidar_utils::extractYaw(current_pose_.orientation) * 180.0 / M_PI,
        result.method == RegistrationResult::Method::FEATURE ? "FEATURE" :
        result.method == RegistrationResult::Method::GICP    ? "GICP" :
                                                               "PREDICTION");
}

// ============================================================================
// Pipeline Stages
// ============================================================================

pcl::PointCloud<pcl::PointXYZ>::Ptr
    LidarOdometry::filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (!cloud || cloud->empty())
    {
        return filtered;
    }

    // Remove invalid points and apply range filter
    auto clean_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    clean_cloud->reserve(cloud->size());

    for (const auto& pt : cloud->points)
    {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
        {
            continue;
        }

        float range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (range > 0.3f && range < 50.0f)
        {
            clean_cloud->push_back(pt);
        }
    }

    if (clean_cloud->empty())
    {
        return filtered;
    }

    // Apply voxel filter
    voxel_filter_.setInputCloud(clean_cloud);
    voxel_filter_.filter(*filtered);

    return filtered;
}

ExtractedFeatures LidarOdometry::extractFeatures(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double                                     timestamp)
{
    if (!feature_extractor_)
    {
        return ExtractedFeatures();
    }

    return feature_extractor_->extractUnorganized(cloud, timestamp);
}

Eigen::Matrix4f LidarOdometry::computeInitialGuess(double dt)
{
    return lidar_utils::predictMotion2D(linear_velocity_, angular_velocity_, dt);
}

// ============================================================================
// Registration Methods
// ============================================================================

RegistrationResult
    LidarOdometry::tryFeatureAlign(const ExtractedFeatures& features, const Eigen::Matrix4f& guess)
{
    RegistrationResult result;
    result.method = RegistrationResult::Method::FEATURE;

    if (!feature_registration_ || !local_map_)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "[DEBUG] tryFeatureAlign: components null - reg=%d, map=%d",
            feature_registration_ != nullptr,
            local_map_ != nullptr);
        return result;
    }

    // Check if we have enough features - require EITHER edges OR planars
    // Changed from AND to OR - we can register with just edges or just planars
    const size_t min_edges = 10;
    const size_t min_planars = 20;
    bool has_enough_edges = features.numEdges() >= min_edges;
    bool has_enough_planars = features.numPlanars() >= min_planars;
    
    if (!has_enough_edges && !has_enough_planars)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "[DEBUG] Insufficient features for registration: edges=%zu (need %zu), planars=%zu (need %zu)",
            features.numEdges(),
            min_edges,
            features.numPlanars(),
            min_planars);
        return result;
    }
    
    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "[DEBUG] Feature counts OK: edges=%zu, planars=%zu - proceeding with registration",
        features.numEdges(),
        features.numPlanars());

    // Perform feature-based alignment
    FeatureRegistrationResult feat_result =
        feature_registration_->alignToLocalMap(features.edge_points, features.planar_points, guess);

    // Convert to unified result
    result.transformation      = feat_result.transformation;
    result.fitness_score       = feat_result.overall_fitness;
    result.converged           = feat_result.converged;
    result.degenerate          = feat_result.degenerate;
    result.num_correspondences = feat_result.inliers_edge + feat_result.inliers_plane;
    result.edge_rmse           = feat_result.edge_rmse;
    result.plane_rmse          = feat_result.plane_rmse;
    result.inliers_edge        = feat_result.inliers_edge;
    result.inliers_plane       = feat_result.inliers_plane;

    // Enforce 2D constraint on transformation
    lidar_utils::enforce2DConstraint(result.transformation);

    return result;
}

RegistrationResult LidarOdometry::alignFallbackGICP(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const Eigen::Matrix4f&                     guess)
{
    RegistrationResult result;
    result.method = RegistrationResult::Method::GICP;

    if (!source || !target ||
        source->size() < static_cast<size_t>(lidar_config_.min_points_threshold) ||
        target->size() < static_cast<size_t>(lidar_config_.min_points_threshold))
    {
        return result;
    }

    // Perform GICP alignment
    auto aligned = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    gicp_.setInputSource(source);
    gicp_.setInputTarget(target);
    gicp_.align(*aligned, guess);

    result.transformation      = gicp_.getFinalTransformation();
    result.fitness_score       = gicp_.getFitnessScore();
    result.converged           = gicp_.hasConverged();
    result.num_correspondences = static_cast<int>(aligned->size());

    // Check for degeneracy
    result.degenerate = isFrameDegenerate(source);

    // Enforce 2D constraint
    lidar_utils::enforce2DConstraint(result.transformation);

    return result;
}

// ============================================================================
// Pose Update
// ============================================================================

void LidarOdometry::updatePoseFromRegistration(
    const RegistrationResult& result,
    const Pose3D&             reference)
{
    // Extract transformation components
    Eigen::Matrix3d rot = result.transformation.block<3, 3>(0, 0).cast<double>();
    Eigen::Vector3d t   = result.transformation.block<3, 1>(0, 3).cast<double>();

    // Create delta pose
    Pose3D delta_pose;
    delta_pose.position    = t;
    delta_pose.orientation = Eigen::Quaterniond(rot);
    delta_pose.orientation.normalize();

    // Compose with reference pose
    current_pose_ = transform_utils::composePoses(reference, delta_pose);
}

void LidarOdometry::updatePoseFromPrediction(const Eigen::Matrix4f& predicted_motion)
{
    // Create predicted delta pose
    Pose3D predicted_delta;
    predicted_delta.position =
        Eigen::Vector3d(predicted_motion(0, 3), predicted_motion(1, 3), predicted_motion(2, 3));
    predicted_delta.orientation =
        Eigen::Quaterniond(predicted_motion.block<3, 3>(0, 0).cast<double>());
    predicted_delta.orientation.normalize();

    // Compose with previous pose
    current_pose_ = transform_utils::composePoses(previous_pose_, predicted_delta);

    // Inflate covariance for dead reckoning
    double failure_scale = std::min(10.0, 1.0 + consecutive_failures_ * 2.0);
    current_covariance_ *= failure_scale;

    // Track cumulative drift estimate
    cumulative_drift_estimate_ += predicted_delta.position.norm() * 0.1;
}

// ============================================================================
// Keyframe Management
// ============================================================================

bool LidarOdometry::shouldCreateKeyframe(const Pose3D& current, const Pose3D& keyframe)
{
    // Check translation threshold
    double distance = (current.position - keyframe.position).norm();
    if (distance >= lidar_config_.keyframe_distance)
    {
        return true;
    }

    // Check rotation threshold
    Eigen::Quaterniond delta_q  = keyframe.orientation.inverse() * current.orientation;
    double             rotation = 2.0 * std::acos(std::clamp(std::abs(delta_q.w()), 0.0, 1.0));

    return rotation >= lidar_config_.keyframe_rotation;
}

void LidarOdometry::maybeAddKeyframe(const ExtractedFeatures& features, const Pose3D& pose)
{
    if (!shouldCreateKeyframe(pose, last_keyframe_pose_))
    {
        return;
    }

    // Update keyframe cloud for GICP fallback
    if (previous_cloud_ && !previous_cloud_->empty())
    {
        last_keyframe_cloud_ = previous_cloud_;
    }

    last_keyframe_pose_        = pose;
    cumulative_drift_estimate_ = 0.0;

    // Add to local map for feature registration
    if (local_map_ && features.hasFeatures())
    {
        bool added = local_map_->addKeyframe(features, pose);

        if (added && feature_registration_)
        {
            // Update registration targets
            feature_registration_->setTargetEdges(local_map_->getEdgeMap());
            feature_registration_->setTargetPlanars(local_map_->getPlanarMap());

            RCLCPP_INFO(
                get_logger(),
                "Keyframe added: %zu keyframes, %zu edges, %zu planars in map",
                local_map_->numKeyframes(),
                local_map_->numEdgePoints(),
                local_map_->numPlanarPoints());
        }
    }

    RCLCPP_INFO(
        get_logger(),
        "Keyframe at (%.2f, %.2f, %.2f)",
        pose.position.x(),
        pose.position.y(),
        pose.position.z());
}

// ============================================================================
// Quality Assessment
// ============================================================================

bool LidarOdometry::isFrameDegenerate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if (!cloud || cloud->size() < 10)
    {
        return true;
    }

    // Compute point cloud covariance
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);

    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);

    // Eigenvalue analysis
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    Eigen::Vector3f                                eigenvalues = solver.eigenvalues();

    // Sort eigenvalues
    std::sort(eigenvalues.data(), eigenvalues.data() + 3);

    // Check eigenvalue ratio
    float min_ev = std::max(eigenvalues(0), 1e-6f);
    float max_ev = eigenvalues(2);
    float ratio  = max_ev / min_ev;

    return ratio > lidar_config_.degeneracy_threshold;
}

void LidarOdometry::updateCovariance(double fitness, bool degenerate)
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

    double scale = lidar_utils::computeCovarianceScale(
        fitness,
        lidar_config_.fitness_threshold,
        lidar_config_.poor_fit_scale,
        degenerate);

    current_covariance_ = lidar_utils::createCovariance(
        lidar_config_.nominal_pos_std * scale,
        lidar_config_.nominal_rot_std * scale);
}

// ============================================================================
// Velocity Estimation
// ============================================================================

void LidarOdometry::updateVelocityEstimate(const Eigen::Matrix4f& delta, double dt)
{
    auto [new_linear, new_angular] = lidar_utils::updateVelocityEMA(
        linear_velocity_,
        angular_velocity_,
        delta,
        dt,
        lidar_config_.velocity_filter_alpha,
        lidar_config_.max_frame_distance / dt,
        M_PI / dt);

    linear_velocity_  = new_linear;
    angular_velocity_ = new_angular;
}

// ============================================================================
// Publishing
// ============================================================================

void LidarOdometry::publishOdometry(double timestamp)
{
    if (!odom_pub_->is_activated())
    {
        return;
    }

    // Update header timestamp
    odom_msg_.header.stamp.sec = static_cast<int32_t>(timestamp);
    odom_msg_.header.stamp.nanosec =
        static_cast<uint32_t>((timestamp - odom_msg_.header.stamp.sec) * 1e9);

    // Update pose
    odom_msg_.pose.pose.position.x    = current_pose_.position.x();
    odom_msg_.pose.pose.position.y    = current_pose_.position.y();
    odom_msg_.pose.pose.position.z    = current_pose_.position.z();
    odom_msg_.pose.pose.orientation.w = current_pose_.orientation.w();
    odom_msg_.pose.pose.orientation.x = current_pose_.orientation.x();
    odom_msg_.pose.pose.orientation.y = current_pose_.orientation.y();
    odom_msg_.pose.pose.orientation.z = current_pose_.orientation.z();

    // Update pose covariance
    for (size_t i = 0; i < 6; ++i)
    {
        for (size_t j = 0; j < 6; ++j)
        {
            odom_msg_.pose.covariance[i * 6 + j] = current_covariance_(i, j);
        }
    }

    // Update twist
    odom_msg_.twist.twist.linear.x  = linear_velocity_.x();
    odom_msg_.twist.twist.linear.y  = linear_velocity_.y();
    odom_msg_.twist.twist.linear.z  = linear_velocity_.z();
    odom_msg_.twist.twist.angular.x = angular_velocity_.x();
    odom_msg_.twist.twist.angular.y = angular_velocity_.y();
    odom_msg_.twist.twist.angular.z = angular_velocity_.z();

    // Twist covariance (scaled from pose covariance)
    constexpr double vel_cov_scale = 10.0;
    for (size_t i = 0; i < 6; ++i)
    {
        for (size_t j = 0; j < 6; ++j)
        {
            odom_msg_.twist.covariance[i * 6 + j] = current_covariance_(i, j) * vel_cov_scale;
        }
    }

    odom_pub_->publish(odom_msg_);
}

// ============================================================================
// Utilities
// ============================================================================

pcl::PointCloud<pcl::PointXYZ>::Ptr
    LidarOdometry::laserScanToPointCloud(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan)
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (!scan)
    {
        return cloud;
    }

    cloud->reserve(scan->ranges.size());

    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        float range = scan->ranges[i];

        if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max)
        {
            continue;
        }

        float angle = scan->angle_min + i * scan->angle_increment;

        pcl::PointXYZ pt;
        pt.x = range * std::cos(angle);
        pt.y = range * std::sin(angle);
        pt.z = 0.0f;

        cloud->push_back(pt);
    }

    return cloud;
}

}  // namespace olive

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(olive::LidarOdometry)