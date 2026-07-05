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

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_map>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>
#include <whycode_vision/msg/why_code_pose_array.hpp>

#include "olive/fusion/feature_extractor.hpp"
#include "olive/fusion/fusion_types.hpp"
#include "olive/fusion/health_monitor.hpp"
#include "olive/fusion/imu_buffer.hpp"
#include "olive/fusion/keyframe_map.hpp"
#include "olive/fusion/loop_detector.hpp"
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

    // Smooth odometry (odom frame): jump-free stream + odom->base TF
    void updateMapOdomCorrection();
    void publishSmoothOdometry(const gtsam::Pose3& odom_from_child, double stamp);

    // IMU startup initialization (gyro bias + sanity checks; gates scans)
    void handleImuInit(double stamp);
    void loadExtrinsicsFromTf();
    void publishDiagnostics();
    void odomTick();
    void attemptLoopClosure(double stamp);
    void refreshAfterCorrection();
    void logSensorLatency(const char* sensor, double stamp);
    bool markerMotionGate(double stamp);
    void reestimateGyroBias();

    // Debug / RViz visualization (all gated by parameters, live-switchable)
    void publishKeyframeDebug(bool trajectory_corrected, double stamp);
    void publishScanDebug(double stamp);
    void publishFiducialDebug(double stamp);

    // Configuration
    std::string  points_topic_;
    std::string  imu_topic_;
    std::string  odom_topic_;
    std::string  odom_frame_;
    std::string  base_frame_;
    std::string  map_frame_;
    std::string  wheel_odom_topic_;
    bool         planar_motion_    = true;
    bool         use_wheel_odom_   = true;
    bool         use_planar_prior_ = true;
    bool         publish_map_tf_   = true;
    bool         publish_odom_tf_  = false;
    std::string  smooth_odom_topic_;
    std::string  odom_child_frame_;
    gtsam::Pose3 child_from_base_;
    double       smooth_odom_rate_hz_ = 50.0;
    bool         use_markers_         = true;
    bool         use_vo_              = false;
    std::string  vo_topic_;
    FactorSigmas vo_between_sigmas_{};
    std::size_t  vo_factors_added_   = 0;  ///< VO betweens that landed
    std::size_t  vo_factors_skipped_ = 0;  ///< VO betweens dropped (coverage)
    std::string  marker_topic_;
    gtsam::Pose3 base_from_camera_;
    double       marker_sigma_m_        = 0.10;
    double       marker_stamp_window_   = 0.25;
    bool         marker_landmark_mode_  = true;
    double       marker_survey_sigma_m_ = 0.05;
    bool         world_anchored_        = false;
    std::unordered_map<int, gtsam::Point3> known_markers_;

    // Debug toggles (live-updatable via `ros2 param set`)
    bool                  debug_enabled_       = false;
    bool                  debug_imu_state_     = true;
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

    // Smooth odometry state: odom->base_footprint, advanced ONLY by
    // correction-independent body increments (never by graph corrections).
    gtsam::Pose3                smooth_pose_;
    std::optional<gtsam::Pose3> wheel_origin_;   ///< first wheel sample (odom anchor)
    gtsam::Pose3                map_from_odom_;  ///< cached correction (jumps live here)
    bool                        map_odom_valid_ = false;
    geometry_msgs::msg::Twist   last_wheel_twist_;

    // IMU tight coupling (preintegration into the graph)
    bool                                                      imu_preintegration_      = false;
    double                                                    imu_preint_max_interval_ = 5.0;
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> pim_;

    // IMU initialization state
    bool   imu_init_done_           = false;
    double imu_init_window_start_   = -1.0;
    double imu_init_first_stamp_    = -1.0;
    double first_scan_stamp_        = -1.0;
    double imu_init_duration_s_     = 1.5;
    double imu_init_max_wait_s_     = 10.0;
    double stationary_gyro_thresh_  = 0.02;
    double stationary_wheel_thresh_ = 0.005;
    bool   gyro_bias_reestimate_    = false;
    bool   deskew_enabled_          = true;
    int    deskew_time_bins_        = 32;

    // Cross-sensor time offsets (lidar is the reference; corrected = msg + offset)
    double imu_time_offset_    = 0.0;
    double wheel_time_offset_  = 0.0;
    double camera_time_offset_ = 0.0;

    // Marker motion gating (camera blur guard on real hardware)
    double marker_max_yaw_rate_ = 0.6;
    double marker_max_speed_    = 1.0;

    // One-shot per-sensor latency characterization at startup
    std::unordered_map<std::string, int> latency_logged_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr           points_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                   imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                 wheel_sub_;
    rclcpp::Subscription<whycode_vision::msg::WhyCodePoseArray>::SharedPtr   marker_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                 vo_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                           tf_broadcaster_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr smooth_odom_pub_;
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
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::AccelStamped>::SharedPtr debug_bias_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
                                                      debug_velocity_pub_;
    nav_msgs::msg::Odometry                           odom_msg_;
    nav_msgs::msg::Odometry                           smooth_odom_msg_;
    std::vector<geometry_msgs::msg::TransformStamped> tf_batch_;  ///< reused broadcast buffer
    rclcpp::TimerBase::SharedPtr                      autostart_timer_;
    rclcpp::TimerBase::SharedPtr                      bias_reestimate_timer_;
    rclcpp::TimerBase::SharedPtr                      diagnostics_timer_;
    rclcpp::TimerBase::SharedPtr                      odom_timer_;
    rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        diagnostics_pub_;

    double optimize_budget_warn_ms_ = 50.0;

    // Health / degradation
    HealthMonitor health_monitor_;
    bool          last_match_ok_             = true;
    bool          last_match_degenerate_     = false;
    double        degenerate_sigma_scale_    = 10.0;
    double        match_fail_sigma_scale_    = 50.0;
    double        wheel_yaw_sigma_per_rad_   = 2.0;
    double        wheel_dist_sigma_per_m_    = 0.1;
    double        wheel_lidar_disagree_m_    = 0.15;
    bool          coast_on_dropout_          = true;
    double        lidar_dropout_timeout_     = 1.0;
    bool          dropout_keyframes_         = false;
    double        prediction_gap_fallback_s_ = 0.5;

    // Loop closure
    std::unique_ptr<LoopDetector> loop_detector_;
    bool                          loop_closure_enabled_ = true;
    double                        loop_min_interval_s_  = 5.0;
    double                        loop_sigma_floor_     = 0.05;
    double                        last_loop_attempt_    = -1.0;

    // Debug state
    nav_msgs::msg::Path                      debug_path_msg_;
    geometry_msgs::msg::PoseArray            debug_keyframes_msg_;
    Cloud                                    debug_scan_cloud_;    ///< reused transform buffer
    std::unordered_map<int64_t, double>      anchor_event_times_;  ///< landmark key -> last stamp
    Cloud::Ptr                               last_edge_map_;
    Cloud::Ptr                               last_planar_map_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

}  // namespace olive

#endif  // OLIVE_FUSION_FUSION_NODE_HPP_
