/**
 * @file lidar_odometry.hpp
 * @brief Lidar odometry using ICP/GICP/NDT
 *
 * Implements LiDAR-based odometry with:
 * - PL-ICP (Point-to-Line ICP) for 2D LiDAR
 * - GICP (Generalized ICP) for 3D LiDAR
 * - Keyframe selection heuristic for efficiency
 */

#ifndef OLIVE_LIDAR_LIDAR_ODOMETRY_HPP_
#define OLIVE_LIDAR_LIDAR_ODOMETRY_HPP_

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/msg/odometry.hpp>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "olive/common/types.hpp"

namespace olive
{

/**
 * @brief Configuration for Lidar Odometry
 */
struct LidarConfig
{
    bool   is_3d;                    ///< True for 3D Lidar, false for 2D
    double keyframe_distance;        ///< Minimum distance to create keyframe (m)
    double keyframe_rotation;        ///< Minimum rotation to create keyframe (rad)
    double voxel_leaf_size;          ///< Voxel filter leaf size (m)
    double max_correspondence_dist;  ///< Max correspondence distance for ICP (m)
    int    max_iterations;           ///< Max ICP iterations
    double transformation_epsilon;   ///< Transformation epsilon for convergence
    double fitness_epsilon;          ///< Fitness epsilon for convergence
    double nominal_pos_std;          ///< Nominal position std deviation(m)
    double nominal_rot_std;          ///< Nominal rotation std deviation(rad)
    double poor_fit_scale;           ///< Covariance scale for poor fitness
    double fitness_threshold;        ///< Threshold for good fitness

    double frame_fitness_threshold;  ///< Threshold for frame-to-frame fitness
    double max_frame_distance;       ///< Max expected motion between frames (m)
    double velocity_filter_alpha;    ///< EMA filter coefficient for velocity
    int    min_points_threshold;     ///< Minimum points for valid registration
    double degeneracy_threshold;     ///< Eigenvalue ratio for degeneracy detection

    LidarConfig()
      : is_3d(true)
      , keyframe_distance(0.15)       // Reduced from 0.5 for faster updates
      , keyframe_rotation(0.1)        // Reduced from 0.2
      , voxel_leaf_size(0.05)         // Reduced from 0.1 for more points
      , max_correspondence_dist(0.5)  // Reduced from 1.0
      , max_iterations(30)            // Reduced from 50
      , transformation_epsilon(1e-5)  // Slightly relaxed
      , fitness_epsilon(1e-3)         // Slightly relaxed
      , nominal_pos_std(0.01)         // Reduced from 0.02
      , nominal_rot_std(0.01)         // Reduced from 0.02
      , poor_fit_scale(3.0)           // Reduced from 5.0
      , fitness_threshold(0.5)        // Reduced from 1.0
      , frame_fitness_threshold(0.8)  // more lenient for frame-to-frame
      , max_frame_distance(0.5)       // sanity check for motion
      , velocity_filter_alpha(0.3)    // smoothing factor
      , min_points_threshold(100)     // minimum valid points
      , degeneracy_threshold(100.0)   // eigenvalue ratio threshold
    {}
};

/**
 * @brief Registration result with quality metrics
 */
struct RegistrationResult
{
    Eigen::Matrix4f transformation;
    double          fitness_score;
    bool            converged;
    bool            degenerate;
    int             num_correspondences;

    RegistrationResult()
      : transformation(Eigen::Matrix4f::Identity())
      , fitness_score(std::numeric_limits<double>::max())
      , converged(false)
      , degenerate(false)
      , num_correspondences(0)
    {}
};

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
    // Callback functions
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void laserScanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);

    // Processing functions
    void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double timestamp);

    // Registration methods
    RegistrationResult performRegistration(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target, const Eigen::Matrix4f& initial_guess);

    // Motion prediction using constant velocity model
    Eigen::Matrix4f predictMotion(double dt);

    // Update velocity estimate with exponential moving average
    void updateVelocityEstimate(const Eigen::Matrix4f& delta_transform, double dt);

    // Point cloud processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // Keyframe management
    bool shouldCreateKeyframe(const Pose3D& current, const Pose3D& last_keyframe);

    Eigen::Matrix4f estimateTransformation(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target, double& fitness_score);

    // Covariance estimation
    void updateCovariance(double fitness_score, bool degenerate);

    // Degeneracy detection using Hessian eigenvalue analysis
    bool checkDegeneracy(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // Publishing
    void publishOdometry(double timestamp);

    // Convert laser scan to point cloud (for 2D LiDAR)
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    laserScanToPointCloud(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan);

    // Configuration
    LidarConfig config_;
    std::string odom_frame_id_{ "odom" };
    std::string lidar_frame_id_{ "lidar_link" };

    // State - Keyframe tracking
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_keyframe_cloud_;
    Pose3D                              last_keyframe_pose_;

    // State - Frame-to-frame tracking (NEW)
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;
    Pose3D                              previous_pose_;
    bool                                has_previous_frame_{ false };

    // State - Velocity estimation (NEW)
    Eigen::Vector3d linear_velocity_{ Eigen::Vector3d::Zero() };
    Eigen::Vector3d angular_velocity_{ Eigen::Vector3d::Zero() };
    double          last_timestamp_{ 0.0 };

    // State - Current pose and uncertainty
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;
    Pose3D                              current_pose_;
    Eigen::Matrix<double, 6, 6>         current_covariance_;
    bool                                initialized_{ false };

    // State - Tracking quality
    int    consecutive_failures_{ 0 };
    double cumulative_drift_estimate_{ 0.0 };

    // PCL components
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_;
    pcl::VoxelGrid<pcl::PointXYZ>                                       voxel_filter_;

    // ROS Interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr           cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr             scan_sub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

}  // namespace olive

#endif  // OLIVE_LIDAR_LIDAR_ODOMETRY_HPP_