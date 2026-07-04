/**
 * @file fusion_node.hpp
 * @brief Lifecycle node running the OLIVE fusion pipeline
 *
 * Consumes /lidar/points and /imu/data, runs the LiDAR-inertial front-end
 * (preprocess -> features -> scan-to-map) and maintains the keyframe factor
 * graph. Publishes the graph estimate as odometry. Wheel, marker and visual
 * factors attach to the same graph in later stages.
 */

#ifndef OLIVE_FUSION_FUSION_NODE_HPP_
#define OLIVE_FUSION_FUSION_NODE_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_map>
#include <visualization_msgs/msg/marker_array.hpp>
#include <whycode_vision/msg/why_code_pose_array.hpp>

#include "olive/fusion/feature_extractor.hpp"
#include "olive/fusion/fusion_types.hpp"
#include "olive/fusion/imu_buffer.hpp"
#include "olive/fusion/keyframe_map.hpp"
#include "olive/fusion/marker_gate.hpp"
#include "olive/fusion/pose_graph.hpp"
#include "olive/fusion/scan_matcher.hpp"
#include "olive/fusion/scan_preprocessor.hpp"
#include "olive/fusion/wheel_odom_buffer.hpp"

namespace olive
{

class FusionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit FusionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

private:
    void declareParameters();
    void loadConfiguration();

    // Hot path: one full pipeline pass per scan
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void markerCallback(const whycode_vision::msg::WhyCodePoseArray::SharedPtr msg);

    void         bootstrapFirstKeyframe(const FeatureClouds& features);
    gtsam::Pose3 predictPose(double scan_stamp) const;
    void         publishOdometry(const gtsam::Pose3& pose, double stamp);

    // IMU startup initialization (gyro bias + sanity checks; gates scans)
    void handleImuInit(double stamp);
    void reestimateGyroBias();

    // Debug / RViz visualization (all gated by parameters, live-switchable)
    void publishKeyframeDebug(bool trajectory_corrected, double stamp);
    void publishScanDebug(double stamp);
    void publishFiducialDebug(double stamp);

    // Configuration
    std::string                            points_topic_;
    std::string                            imu_topic_;
    std::string                            odom_topic_;
    std::string                            odom_frame_;
    std::string                            base_frame_;
    std::string                            map_frame_;
    std::string                            wheel_odom_topic_;
    bool                                   planar_motion_    = true;
    bool                                   use_wheel_odom_   = true;
    bool                                   use_planar_prior_ = true;
    bool                                   publish_map_tf_   = true;
    bool                                   use_markers_      = true;
    bool                                   use_vo_           = false;
    std::string                            vo_topic_;
    FactorSigmas                           vo_between_sigmas_{};
    std::string                            marker_topic_;
    gtsam::Pose3                           base_from_camera_;
    double                                 marker_sigma_m_      = 0.10;
    double                                 marker_stamp_window_ = 0.25;
    std::unordered_map<int, gtsam::Point3> known_markers_;

    // Debug toggles (live-updatable via `ros2 param set`)
    bool                  debug_enabled_       = false;
    bool                  debug_path_          = true;
    bool                  debug_keyframes_     = true;
    bool                  debug_local_map_     = true;
    bool                  debug_scan_features_ = true;
    bool                  debug_fiducials_     = true;
    FactorSigmas          lidar_between_sigmas_{};
    FactorSigmas          wheel_between_sigmas_{};
    std::array<double, 3> planar_prior_sigmas_{};

    PreprocessorConfig preprocessor_config_;
    FeatureConfig      feature_config_;
    MatcherConfig      matcher_config_;
    KeyframeConfig     keyframe_config_;

    // Pipeline modules
    std::unique_ptr<ScanPreprocessor> preprocessor_;
    std::unique_ptr<FeatureExtractor> feature_extractor_;
    std::unique_ptr<ScanMatcher>      scan_matcher_;
    std::unique_ptr<KeyframeMap>      keyframe_map_;
    std::unique_ptr<PoseGraph>        pose_graph_;
    ImuBuffer                         imu_buffer_;
    WheelOdomBuffer                   wheel_buffer_;
    WheelOdomBuffer                   vo_buffer_;
    std::unique_ptr<MarkerGate>       marker_gate_;

    // Per-scan state (reused buffers; hot path must not allocate)
    ScanImage     scan_image_;
    FeatureClouds features_;

    gtsam::Pose3 last_scan_pose_;
    gtsam::Pose3 last_increment_;
    double       last_scan_stamp_ = -1.0;

    // IMU initialization state
    bool   imu_init_done_          = false;
    double imu_init_window_start_  = -1.0;
    double imu_init_first_stamp_   = -1.0;
    double first_scan_stamp_       = -1.0;
    double imu_init_duration_s_    = 1.5;
    double imu_init_max_wait_s_    = 10.0;
    double stationary_gyro_thresh_ = 0.02;
    double stationary_wheel_thresh_ = 0.005;
    bool   gyro_bias_reestimate_   = false;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr           points_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                   imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                 wheel_sub_;
    rclcpp::Subscription<whycode_vision::msg::WhyCodePoseArray>::SharedPtr   marker_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                 vo_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                           tf_broadcaster_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr     debug_path_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr
        debug_keyframes_pub_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        debug_map_edges_pub_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        debug_map_planars_pub_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        debug_scan_edges_pub_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        debug_scan_planars_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
                                 debug_fiducials_pub_;
    nav_msgs::msg::Odometry      odom_msg_;
    rclcpp::TimerBase::SharedPtr autostart_timer_;
    rclcpp::TimerBase::SharedPtr bias_reestimate_timer_;

    // Debug state
    nav_msgs::msg::Path             debug_path_msg_;
    Cloud                           debug_scan_cloud_;    ///< reused transform buffer
    std::unordered_map<int, double> anchor_event_times_;  ///< marker id -> last anchor stamp
    Cloud::Ptr                      last_edge_map_;
    Cloud::Ptr                      last_planar_map_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

}  // namespace olive

#endif  // OLIVE_FUSION_FUSION_NODE_HPP_
