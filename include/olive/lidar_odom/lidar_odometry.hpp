/**
 * @file lidar_odometry.hpp
 * @brief LiDAR odometry node with two-tier registration
 *
 * Architecture:
 * 1. Primary: Feature-based scan-to-map alignment
 *    - Edge features constrain rotation (yaw)
 *    - Planar features constrain translation (x, y)
 * 2. Fallback: GICP when features are insufficient
 *
 * All poses are constrained to (x, y, yaw) for ground robots.
 *
 * Pipeline:
 *   PointCloud -> Filter -> Extract Features -> Align to Map -> Update Pose -> Publish
 *                                     |
 *                                     v
 *                              Maybe Add Keyframe
 */

#ifndef OLIVE_LIDAR_LIDAR_ODOMETRY_HPP_
#define OLIVE_LIDAR_LIDAR_ODOMETRY_HPP_

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "olive/common/types.hpp"
#include "olive/lidar_odom/feature_extractor.hpp"
#include "olive/lidar_odom/feature_registration.hpp"
#include "olive/lidar_odom/feature_types.hpp"
#include "olive/lidar_odom/lidar_config.hpp"
#include "olive/lidar_odom/local_map.hpp"
#include "olive/lidar_odom/registration_types.hpp"

namespace olive
{

class LidarOdometry : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit LidarOdometry(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    // Lifecycle callbacks
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State& state) override;

private:
    //=== Sensor Callbacks ===
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void laserScanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);

    // === Main Processing Pipeline ===
    void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double timestamp);

    // === Pipeline Stages ===
    pcl::PointCloud<pcl::PointXYZ>::Ptr
        filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    ExtractedFeatures
        extractFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double timestamp);
    Eigen::Matrix4f computeInitialGuess(double dt);

    // === Registration Methods ===
    RegistrationResult
        tryFeatureAlign(const ExtractedFeatures& features, const Eigen::Matrix4f& guess);
    RegistrationResult alignFallbackGICP(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
        const Eigen::Matrix4f&                     guess);

    // === Pose Update ===
    void updatePoseFromRegistration(const RegistrationResult& result, const Pose3D& reference);
    void updatePoseFromPrediction(const Eigen::Matrix4f& predicted_motion);

    // === Keyframe Management ===
    bool shouldCreateKeyframe(const Pose3D& current, const Pose3D& keyframe);
    void maybeAddKeyframe(const ExtractedFeatures& features, const Pose3D& pose);

    // === Quality Assessment ===
    bool isFrameDegenerate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void updateCovariance(double fitness, bool degenerate);

    // === Velocity Estimation ===
    void updateVelocityEstimate(const Eigen::Matrix4f& delta, double dt);

    // === Publishing ===
    void publishOdometry(double timestamp);

    // === Utilities ===
    pcl::PointCloud<pcl::PointXYZ>::Ptr
        laserScanToPointCloud(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan);

    // === Configuration ===
    LidarConfig               lidar_config_;
    FrameConfig               frame_config_;
    FeatureExtractionConfig   feature_config_;
    FeatureRegistrationConfig registration_config_;
    LocalMapConfig            map_config_;

    // === State: Pose Tracking ===
    Pose3D             current_pose_;
    Pose3D             previous_pose_;
    Pose3D             last_keyframe_pose_;
    Eigen::Quaterniond previous_orientation_;  // For quaternion consistency

    // === State: Point Clouds ===
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_keyframe_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;

    // === State: Velocity ===
    Eigen::Vector3d linear_velocity_{ Eigen::Vector3d::Zero() };
    Eigen::Vector3d angular_velocity_{ Eigen::Vector3d::Zero() };
    double          last_timestamp_{ 0.0 };

    // === State: Covariance ===
    Eigen::Matrix<double, 6, 6> current_covariance_;

    // === State: Tracking Quality ===
    bool   initialized_{ false };
    bool   has_previous_frame_{ false };
    int    consecutive_failures_{ 0 };
    double cumulative_drift_estimate_{ 0.0 };

    // === PCL Components ===
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_;
    pcl::VoxelGrid<pcl::PointXYZ>                                       voxel_filter_;

    // === Feature Components ===
    std::unique_ptr<FeatureExtractor>    feature_extractor_;
    std::unique_ptr<FeatureRegistration> feature_registration_;
    std::unique_ptr<LocalMap>            local_map_;

    // === ROS Interfaces ===
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr           cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr             scan_sub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // === Reusable Message (avoid per-callback allocation) ===
    nav_msgs::msg::Odometry odom_msg_;
};

}  // namespace olive

#endif  // OLIVE_LIDAR_LIDAR_ODOMETRY_HPP_