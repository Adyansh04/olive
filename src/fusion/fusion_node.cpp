#include "olive/fusion/fusion_node.hpp"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>

#include "olive/common/covariance_utils.hpp"
#include "olive/common/gtsam_conversions.hpp"

namespace olive
{

FusionNode::FusionNode(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("fusion_node", options)
{
    declareParameters();

    // Debug toggles react to `ros2 param set` at runtime — no restart needed.
    param_callback_ =
        add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
            for (const rclcpp::Parameter& p : params)
            {
                if (p.get_name() == "publish_debug")
                    debug_enabled_ = p.as_bool();
                else if (p.get_name() == "debug_path")
                    debug_path_ = p.as_bool();
                else if (p.get_name() == "debug_keyframes")
                    debug_keyframes_ = p.as_bool();
                else if (p.get_name() == "debug_local_map")
                    debug_local_map_ = p.as_bool();
                else if (p.get_name() == "debug_scan_features")
                    debug_scan_features_ = p.as_bool();
                else if (p.get_name() == "debug_fiducials")
                    debug_fiducials_ = p.as_bool();
                else if (p.get_name() == "debug_imu_state")
                    debug_imu_state_ = p.as_bool();
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        });

    // Self-managed bring-up: launch files stay free of lifecycle event
    // wiring, while autostart:=false keeps manual control available.
    if (get_parameter("autostart").as_bool())
    {
        autostart_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
            autostart_timer_->cancel();
            configure();
            activate();
        });
    }
}

void FusionNode::declareParameters()
{
    declare_parameter("autostart", true);
    declare_parameter("points_topic", "/lidar/points");
    declare_parameter("imu_topic", "/imu/data");
    declare_parameter("odom_topic", "/olive/odometry");
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("map_frame", "map");
    declare_parameter("wheel_odom_topic", "/odom");
    declare_parameter("use_wheel_odom", true);
    declare_parameter("use_planar_prior", true);
    declare_parameter("publish_map_tf", true);
    declare_parameter("publish_odom_tf", false);
    declare_parameter("smooth_odom_topic", "/olive/odometry_local");
    declare_parameter("smooth_odom_rate_hz", 50.0);
    declare_parameter("odom_child_frame", "base_footprint");
    declare_parameter("odom_child_to_base_xyz", std::vector<double>{ 0.0, 0.0, 0.06 });
    declare_parameter("wheel_between_sigmas", std::vector<double>{ 0.03, 0.03, 0.5, 0.5, 0.5, 0.2 });
    declare_parameter("planar_prior_sigmas", std::vector<double>{ 0.02, 0.009, 0.009 });

    declare_parameter("use_vo", false);
    declare_parameter("vo_topic", "/olive/visual_odom");
    declare_parameter("vo_between_sigmas", std::vector<double>{ 0.1, 0.1, 1.0, 1.0, 1.0, 0.1 });
    declare_parameter("use_markers", true);
    declare_parameter("marker_topic", "/whycode/poses");
    declare_parameter("camera_translation", std::vector<double>{ 0.2, 0.0, 0.06 });
    declare_parameter("camera_rpy", std::vector<double>{ -M_PI_2, 0.0, -M_PI_2 });
    declare_parameter("marker_position_sigma_m", 0.10);
    declare_parameter("marker_stamp_window_s", 0.25);
    declare_parameter("marker_min_range_m", 0.5);
    declare_parameter("marker_max_range_m", 6.0);
    declare_parameter("marker_min_track_frames", 3);
    declare_parameter("known_marker_ids", std::vector<int64_t>{});
    declare_parameter("known_marker_positions", std::vector<double>{});
    declare_parameter("marker_mode", "landmark");
    declare_parameter("marker_survey_sigma_m", 0.05);
    declare_parameter("marker_accept_unknown_ids", false);
    declare_parameter("marker_accept_undecoded_ids", false);

    declare_parameter("lidar_translation", std::vector<double>{ 0.0, 0.0, 0.145 });
    declare_parameter("lidar_rpy", std::vector<double>{ 0.0, 0.0, 0.0 });
    declare_parameter("imu_rpy", std::vector<double>{ 0.0, 0.0, 0.0 });

    declare_parameter("imu_init_duration_s", 1.5);
    declare_parameter("imu_init_max_wait_s", 10.0);
    declare_parameter("stationary_gyro_thresh_rad_s", 0.02);
    declare_parameter("stationary_wheel_thresh_m", 0.005);
    declare_parameter("gyro_bias_reestimate", false);

    declare_parameter("imu_preintegration", false);
    declare_parameter("accel_noise_sigma", 0.05);
    declare_parameter("gyro_noise_sigma", 0.005);
    declare_parameter("accel_bias_rw_sigma", 0.001);
    declare_parameter("gyro_bias_rw_sigma", 1.0e-4);
    declare_parameter("integration_sigma", 1.0e-4);
    declare_parameter("bias_acc_omega_int", 1.0e-4);
    declare_parameter("debug_imu_state", true);

    declare_parameter("imu_time_offset_s", 0.0);
    declare_parameter("wheel_time_offset_s", 0.0);
    declare_parameter("camera_time_offset_s", 0.0);
    declare_parameter("wheel_interp_slack_s", 0.05);
    declare_parameter("marker_max_yaw_rate_rad_s", 0.6);
    declare_parameter("marker_max_speed_m_s", 1.0);

    declare_parameter("extrinsics_from_tf", false);
    declare_parameter("lidar_frame", "lidar_link");
    declare_parameter("camera_frame", "camera_link");
    declare_parameter("extrinsics_tf_timeout_s", 5.0);

    declare_parameter("min_range", 0.3);
    declare_parameter("max_range", 12.0);
    declare_parameter("point_time_field", "auto");
    declare_parameter("ring_field", "auto");
    declare_parameter("deskew_enabled", true);
    declare_parameter("deskew_time_bins", 32);

    declare_parameter("planar_motion", true);
    declare_parameter("curvature_window", 2);
    declare_parameter("edge_threshold", 1.0);
    declare_parameter("planar_threshold", 0.1);
    declare_parameter("scan_planar_leaf_size", 0.4);

    declare_parameter("matcher_max_iterations", 30);
    declare_parameter("degeneracy_eigen_threshold", 100.0);

    declare_parameter("keyframe_translation_m", 0.5);
    declare_parameter("keyframe_rotation_deg", 12.0);
    declare_parameter("local_map_radius_m", 25.0);
    declare_parameter("local_map_recent_s", 10.0);
    declare_parameter("map_edge_leaf_size", 0.2);
    declare_parameter("map_planar_leaf_size", 0.4);
    declare_parameter("cloud_voxel_m", 0.0);
    declare_parameter("max_cloud_keyframes", 0);

    declare_parameter("publish_debug", false);
    declare_parameter("debug_path", true);
    declare_parameter("debug_keyframes", true);
    declare_parameter("debug_local_map", true);
    declare_parameter("debug_scan_features", true);
    declare_parameter("debug_fiducials", true);

    declare_parameter("loop_closure_enabled", true);
    declare_parameter("loop_search_radius_m", 3.0);
    declare_parameter("loop_min_time_diff_s", 30.0);
    declare_parameter("loop_submap_half_width", 12);
    declare_parameter("loop_fitness_threshold", 0.3);
    declare_parameter("loop_max_correction_m", 3.0);
    declare_parameter("loop_min_interval_s", 5.0);
    declare_parameter("loop_sigma_floor", 0.05);

    declare_parameter("optimize_budget_warn_ms", 50.0);

    declare_parameter("diagnostics_period_s", 1.0);
    declare_parameter("lidar_timeout_s", 1.0);
    declare_parameter("imu_timeout_s", 0.5);
    declare_parameter("wheel_timeout_s", 0.5);
    declare_parameter("degenerate_sigma_scale", 10.0);
    declare_parameter("match_fail_sigma_scale", 50.0);
    declare_parameter("wheel_yaw_sigma_per_rad", 2.0);
    declare_parameter("wheel_dist_sigma_per_m", 0.1);
    declare_parameter("wheel_lidar_disagree_m", 0.15);
    declare_parameter("coast_on_dropout", true);
    declare_parameter("lidar_dropout_timeout_s", 1.0);
    declare_parameter("dropout_keyframes", false);
    declare_parameter("prediction_gap_wheel_fallback_s", 0.5);
    declare_parameter("relinearize_threshold", 0.1);
    declare_parameter("relinearize_skip", 1);
    declare_parameter(
        "lidar_between_sigmas",
        std::vector<double>{ 0.01, 0.01, 0.01, 0.001, 0.001, 0.001 });
}

void FusionNode::loadConfiguration()
{
    points_topic_ = get_parameter("points_topic").as_string();
    imu_topic_    = get_parameter("imu_topic").as_string();
    odom_topic_   = get_parameter("odom_topic").as_string();
    odom_frame_   = get_parameter("odom_frame").as_string();
    base_frame_   = get_parameter("base_frame").as_string();
    map_frame_    = get_parameter("map_frame").as_string();

    wheel_odom_topic_ = get_parameter("wheel_odom_topic").as_string();
    use_wheel_odom_   = get_parameter("use_wheel_odom").as_bool();
    use_planar_prior_ = get_parameter("use_planar_prior").as_bool();
    publish_map_tf_   = get_parameter("publish_map_tf").as_bool();

    publish_odom_tf_     = get_parameter("publish_odom_tf").as_bool();
    smooth_odom_topic_   = get_parameter("smooth_odom_topic").as_string();
    smooth_odom_rate_hz_ = get_parameter("smooth_odom_rate_hz").as_double();
    odom_child_frame_    = get_parameter("odom_child_frame").as_string();
    const auto child_xyz = get_parameter("odom_child_to_base_xyz").as_double_array();
    child_from_base_ =
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(child_xyz[0], child_xyz[1], child_xyz[2]));

    const auto wheel_sigmas = get_parameter("wheel_between_sigmas").as_double_array();
    for (size_t i = 0; i < 6 && i < wheel_sigmas.size(); ++i)
        wheel_between_sigmas_[i] = wheel_sigmas[i];
    const auto planar_sigmas = get_parameter("planar_prior_sigmas").as_double_array();
    for (size_t i = 0; i < 3 && i < planar_sigmas.size(); ++i)
        planar_prior_sigmas_[i] = planar_sigmas[i];

    use_vo_              = get_parameter("use_vo").as_bool();
    vo_topic_            = get_parameter("vo_topic").as_string();
    const auto vo_sigmas = get_parameter("vo_between_sigmas").as_double_array();
    for (size_t i = 0; i < 6 && i < vo_sigmas.size(); ++i)
        vo_between_sigmas_[i] = vo_sigmas[i];

    use_markers_  = get_parameter("use_markers").as_bool();
    marker_topic_ = get_parameter("marker_topic").as_string();

    const auto cam_t   = get_parameter("camera_translation").as_double_array();
    const auto cam_rpy = get_parameter("camera_rpy").as_double_array();
    base_from_camera_  = gtsam::Pose3(
        gtsam::Rot3::Ypr(cam_rpy[2], cam_rpy[1], cam_rpy[0]),
        gtsam::Point3(cam_t[0], cam_t[1], cam_t[2]));

    marker_sigma_m_        = get_parameter("marker_position_sigma_m").as_double();
    marker_stamp_window_   = get_parameter("marker_stamp_window_s").as_double();
    marker_landmark_mode_  = get_parameter("marker_mode").as_string() != "anchor";
    marker_survey_sigma_m_ = get_parameter("marker_survey_sigma_m").as_double();

    MarkerGateConfig gate_config;
    gate_config.min_range = get_parameter("marker_min_range_m").as_double();
    gate_config.max_range = get_parameter("marker_max_range_m").as_double();
    gate_config.min_track_frames =
        static_cast<int>(get_parameter("marker_min_track_frames").as_int());
    // Free landmarks (unsurveyed / undecoded) only exist in landmark mode;
    // the legacy anchor path can only consume surveyed ids.
    gate_config.accept_unknown_ids =
        marker_landmark_mode_ && get_parameter("marker_accept_unknown_ids").as_bool();
    gate_config.accept_undecoded_ids =
        marker_landmark_mode_ && get_parameter("marker_accept_undecoded_ids").as_bool();

    known_markers_.clear();
    // Free landmarks initialized before the first survey anchor would lock in
    // the spawn-frame gauge and then fight the anchor snap (a gauge conflict
    // that can fold the map). They are held back until the trajectory is
    // world-anchored — unless nothing is surveyed, in which case the spawn
    // gauge is the only gauge and there is no conflict.
    world_anchored_      = false;
    const auto ids       = get_parameter("known_marker_ids").as_integer_array();
    const auto positions = get_parameter("known_marker_positions").as_double_array();
    for (size_t i = 0; i < ids.size() && i * 3 + 2 < positions.size(); ++i)
    {
        const int id = static_cast<int>(ids[i]);
        known_markers_.emplace(
            id,
            gtsam::Point3(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]));
        gate_config.known_ids.insert(id);
    }
    if (known_markers_.empty())
        world_anchored_ = true;  // no surveys: the spawn gauge is the gauge
    marker_gate_ = std::make_unique<MarkerGate>(gate_config);

    if (get_parameter("extrinsics_from_tf").as_bool())
        loadExtrinsicsFromTf();

    const auto translation               = get_parameter("lidar_translation").as_double_array();
    const auto rpy                       = get_parameter("lidar_rpy").as_double_array();
    preprocessor_config_.base_from_lidar = pcl::getTransformation(
        static_cast<float>(translation[0]),
        static_cast<float>(translation[1]),
        static_cast<float>(translation[2]),
        static_cast<float>(rpy[0]),
        static_cast<float>(rpy[1]),
        static_cast<float>(rpy[2]));
    const auto imu_rpy = get_parameter("imu_rpy").as_double_array();
    imu_buffer_.setMountingRotation(Eigen::Quaterniond(
        Eigen::AngleAxisd(imu_rpy[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(imu_rpy[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(imu_rpy[0], Eigen::Vector3d::UnitX())));

    imu_init_duration_s_     = get_parameter("imu_init_duration_s").as_double();
    imu_init_max_wait_s_     = get_parameter("imu_init_max_wait_s").as_double();
    stationary_gyro_thresh_  = get_parameter("stationary_gyro_thresh_rad_s").as_double();
    stationary_wheel_thresh_ = get_parameter("stationary_wheel_thresh_m").as_double();
    gyro_bias_reestimate_    = get_parameter("gyro_bias_reestimate").as_bool();

    imu_preintegration_ = get_parameter("imu_preintegration").as_bool();
    debug_imu_state_    = get_parameter("debug_imu_state").as_bool();
    if (imu_preintegration_)
    {
        // Combined preintegration: accel/gyro white noise + bias random walks
        // all live inside one 6-key factor. body_P_sensor stays identity —
        // the buffer already rotates samples into base axes (the translation
        // lever arm is unmodeled; ~omega^2*r, negligible at this platform's
        // rates).
        const auto sq   = [](double v) { return v * v; };
        auto pim_params = gtsam::PreintegrationCombinedParams::MakeSharedU(constants::GRAVITY);
        pim_params->accelerometerCovariance =
            gtsam::I_3x3 * sq(get_parameter("accel_noise_sigma").as_double());
        pim_params->gyroscopeCovariance =
            gtsam::I_3x3 * sq(get_parameter("gyro_noise_sigma").as_double());
        pim_params->integrationCovariance =
            gtsam::I_3x3 * sq(get_parameter("integration_sigma").as_double());
        pim_params->biasAccCovariance =
            gtsam::I_3x3 * sq(get_parameter("accel_bias_rw_sigma").as_double());
        pim_params->biasOmegaCovariance =
            gtsam::I_3x3 * sq(get_parameter("gyro_bias_rw_sigma").as_double());
        pim_params->biasAccOmegaInt =
            gtsam::I_6x6 * sq(get_parameter("bias_acc_omega_int").as_double());
        pim_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(pim_params);
    }

    imu_time_offset_    = get_parameter("imu_time_offset_s").as_double();
    wheel_time_offset_  = get_parameter("wheel_time_offset_s").as_double();
    camera_time_offset_ = get_parameter("camera_time_offset_s").as_double();
    wheel_buffer_.setInterpolationSlack(get_parameter("wheel_interp_slack_s").as_double());
    marker_max_yaw_rate_ = get_parameter("marker_max_yaw_rate_rad_s").as_double();
    marker_max_speed_    = get_parameter("marker_max_speed_m_s").as_double();
    latency_logged_.clear();

    preprocessor_config_.min_range = static_cast<float>(get_parameter("min_range").as_double());
    preprocessor_config_.max_range = static_cast<float>(get_parameter("max_range").as_double());
    preprocessor_config_.point_time_field = get_parameter("point_time_field").as_string();
    preprocessor_config_.ring_field       = get_parameter("ring_field").as_string();
    deskew_enabled_                       = get_parameter("deskew_enabled").as_bool();
    deskew_time_bins_ = static_cast<int>(get_parameter("deskew_time_bins").as_int());

    planar_motion_                   = get_parameter("planar_motion").as_bool();
    feature_config_.curvature_window = static_cast<int>(get_parameter("curvature_window").as_int());
    feature_config_.edge_threshold =
        static_cast<float>(get_parameter("edge_threshold").as_double());
    feature_config_.planar_threshold =
        static_cast<float>(get_parameter("planar_threshold").as_double());
    feature_config_.planar_leaf_size =
        static_cast<float>(get_parameter("scan_planar_leaf_size").as_double());

    matcher_config_.max_iterations =
        static_cast<int>(get_parameter("matcher_max_iterations").as_int());
    matcher_config_.degeneracy_eigen_threshold =
        static_cast<float>(get_parameter("degeneracy_eigen_threshold").as_double());

    keyframe_config_.keyframe_translation = get_parameter("keyframe_translation_m").as_double();
    keyframe_config_.keyframe_rotation =
        get_parameter("keyframe_rotation_deg").as_double() * constants::DEG_TO_RAD;
    keyframe_config_.search_radius = get_parameter("local_map_radius_m").as_double();
    keyframe_config_.recent_window = get_parameter("local_map_recent_s").as_double();
    keyframe_config_.edge_leaf_size =
        static_cast<float>(get_parameter("map_edge_leaf_size").as_double());
    keyframe_config_.planar_leaf_size =
        static_cast<float>(get_parameter("map_planar_leaf_size").as_double());
    keyframe_config_.cloud_voxel = get_parameter("cloud_voxel_m").as_double();
    keyframe_config_.max_cloud_keyframes =
        static_cast<size_t>(std::max<int64_t>(0, get_parameter("max_cloud_keyframes").as_int()));

    loop_closure_enabled_ = get_parameter("loop_closure_enabled").as_bool();
    loop_min_interval_s_  = get_parameter("loop_min_interval_s").as_double();
    loop_sigma_floor_     = get_parameter("loop_sigma_floor").as_double();

    LoopDetectorConfig loop_config;
    loop_config.search_radius = get_parameter("loop_search_radius_m").as_double();
    loop_config.min_time_diff = get_parameter("loop_min_time_diff_s").as_double();
    loop_config.submap_half_width =
        static_cast<int>(get_parameter("loop_submap_half_width").as_int());
    loop_config.fitness_threshold = get_parameter("loop_fitness_threshold").as_double();
    loop_config.max_correction    = get_parameter("loop_max_correction_m").as_double();
    loop_config.planar            = planar_motion_;
    loop_detector_                = std::make_unique<LoopDetector>(loop_config);
    last_loop_attempt_            = -1.0;

    optimize_budget_warn_ms_ = get_parameter("optimize_budget_warn_ms").as_double();

    degenerate_sigma_scale_    = get_parameter("degenerate_sigma_scale").as_double();
    match_fail_sigma_scale_    = get_parameter("match_fail_sigma_scale").as_double();
    wheel_yaw_sigma_per_rad_   = get_parameter("wheel_yaw_sigma_per_rad").as_double();
    wheel_dist_sigma_per_m_    = get_parameter("wheel_dist_sigma_per_m").as_double();
    wheel_lidar_disagree_m_    = get_parameter("wheel_lidar_disagree_m").as_double();
    coast_on_dropout_          = get_parameter("coast_on_dropout").as_bool();
    lidar_dropout_timeout_     = get_parameter("lidar_dropout_timeout_s").as_double();
    dropout_keyframes_         = get_parameter("dropout_keyframes").as_bool();
    prediction_gap_fallback_s_ = get_parameter("prediction_gap_wheel_fallback_s").as_double();

    health_monitor_.configure({
        { "lidar", get_parameter("lidar_timeout_s").as_double() },
        { "imu", get_parameter("imu_timeout_s").as_double() },
        { "wheel", get_parameter("wheel_timeout_s").as_double() },
        { "markers", 0.0 },  // optional sources: age-reported, never FAILED
        { "vo", 0.0 },
    });

    debug_enabled_       = get_parameter("publish_debug").as_bool();
    debug_path_          = get_parameter("debug_path").as_bool();
    debug_keyframes_     = get_parameter("debug_keyframes").as_bool();
    debug_local_map_     = get_parameter("debug_local_map").as_bool();
    debug_scan_features_ = get_parameter("debug_scan_features").as_bool();
    debug_fiducials_     = get_parameter("debug_fiducials").as_bool();

    const auto sigmas = get_parameter("lidar_between_sigmas").as_double_array();
    for (size_t i = 0; i < 6 && i < sigmas.size(); ++i)
        lidar_between_sigmas_[i] = sigmas[i];
}

FusionNode::CallbackReturn FusionNode::on_configure(const rclcpp_lifecycle::State&)
{
    loadConfiguration();

    preprocessor_      = std::make_unique<ScanPreprocessor>(preprocessor_config_);
    feature_extractor_ = std::make_unique<FeatureExtractor>(feature_config_);
    scan_matcher_      = std::make_unique<ScanMatcher>(matcher_config_);
    keyframe_map_      = std::make_unique<KeyframeMap>(keyframe_config_);
    pose_graph_        = std::make_unique<PoseGraph>(
        get_parameter("relinearize_threshold").as_double(),
        static_cast<int>(get_parameter("relinearize_skip").as_int()));

    last_scan_pose_  = gtsam::Pose3();
    last_increment_  = gtsam::Pose3();
    last_scan_stamp_ = -1.0;

    smooth_pose_    = gtsam::Pose3();
    map_from_odom_  = gtsam::Pose3();
    map_odom_valid_ = false;
    wheel_origin_.reset();
    last_wheel_twist_ = geometry_msgs::msg::Twist();
    tf_batch_.reserve(2);

    imu_init_done_         = false;
    imu_init_window_start_ = -1.0;
    imu_init_first_stamp_  = -1.0;
    first_scan_stamp_      = -1.0;

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
    smooth_odom_pub_ =
        create_publisher<nav_msgs::msg::Odometry>(smooth_odom_topic_, rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Debug topics: keyframe-rate ones are transient-local so a late RViz
    // still receives the latest state; scan-rate ones stay volatile.
    const rclcpp::QoS latched = rclcpp::QoS(1).transient_local();
    debug_path_pub_           = create_publisher<nav_msgs::msg::Path>("/olive/debug/path", latched);
    debug_keyframes_pub_ =
        create_publisher<geometry_msgs::msg::PoseArray>("/olive/debug/keyframes", latched);
    debug_map_edges_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/olive/debug/map_edges", latched);
    debug_map_planars_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/olive/debug/map_planars", latched);
    debug_scan_edges_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/olive/debug/scan_edges", rclcpp::QoS(1));
    debug_scan_planars_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/olive/debug/scan_planars", rclcpp::QoS(1));
    debug_fiducials_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("/olive/debug/fiducials", latched);
    debug_bias_pub_ =
        create_publisher<geometry_msgs::msg::AccelStamped>("/olive/debug/bias", rclcpp::QoS(10));
    debug_velocity_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "/olive/debug/velocity",
        rclcpp::QoS(10));
    diagnostics_pub_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10));

    debug_path_msg_.poses.clear();
    debug_path_msg_.header.frame_id = map_frame_;
    debug_keyframes_msg_.poses.clear();
    anchor_event_times_.clear();
    last_edge_map_.reset();
    last_planar_map_.reset();

    // REP-105 split: the fused estimate lives in the map frame; the smooth
    // odom-frame stream carries only jump-free increments. With
    // publish_odom_tf the node owns BOTH map->odom and odom->base_footprint;
    // otherwise the odom frame stays owned by the wheel odometry source.
    odom_msg_.header.frame_id = map_frame_;
    odom_msg_.child_frame_id  = base_frame_;

    smooth_odom_msg_.header.frame_id = odom_frame_;
    smooth_odom_msg_.child_frame_id  = odom_child_frame_;

    RCLCPP_INFO(
        get_logger(),
        "Configured (points: %s, imu: %s -> %s)",
        points_topic_.c_str(),
        imu_topic_.c_str(),
        odom_topic_.c_str());
    return CallbackReturn::SUCCESS;
}

FusionNode::CallbackReturn FusionNode::on_activate(const rclcpp_lifecycle::State& state)
{
    LifecycleNode::on_activate(state);

    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        points_topic_,
        rclcpp::SensorDataQoS().keep_last(2),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { pointCloudCallback(msg); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_,
        rclcpp::SensorDataQoS().keep_last(100),
        [this](sensor_msgs::msg::Imu::SharedPtr msg) { imuCallback(msg); });
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        wheel_odom_topic_,
        rclcpp::SensorDataQoS().keep_last(50),
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { wheelOdomCallback(msg); });
    if (use_vo_)
        vo_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            vo_topic_,
            rclcpp::QoS(10),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                const double stamp =
                    static_cast<double>(msg->header.stamp.sec) + 1e-9 * msg->header.stamp.nanosec;
                vo_buffer_.push(stamp, gtsam_conversions::toGtsamPose(msg->pose.pose));
            });
    if (use_markers_)
        marker_sub_ = create_subscription<whycode_vision::msg::WhyCodePoseArray>(
            marker_topic_,
            rclcpp::QoS(10),
            [this](whycode_vision::msg::WhyCodePoseArray::SharedPtr msg) { markerCallback(msg); });

    if (gyro_bias_reestimate_ && imu_preintegration_)
        RCLCPP_WARN(
            get_logger(),
            "gyro_bias_reestimate is redundant with imu_preintegration (the graph estimates "
            "bias online) - skipping the stationary EMA timer");
    else if (gyro_bias_reestimate_)
        bias_reestimate_timer_ =
            create_wall_timer(std::chrono::seconds(1), [this]() { reestimateGyroBias(); });

    diagnostics_timer_ = create_wall_timer(
        std::chrono::duration<double>(get_parameter("diagnostics_period_s").as_double()),
        [this]() { publishDiagnostics(); });
    // Smooth-odom extension and dropout coasting share one tick: at the
    // smooth rate when this node owns odom->base, else the legacy 5 Hz coast.
    const double tick_hz = publish_odom_tf_ ? smooth_odom_rate_hz_ : 5.0;
    if (publish_odom_tf_ || coast_on_dropout_)
        odom_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / std::max(1.0, tick_hz)),
            [this]() { odomTick(); });

    RCLCPP_INFO(get_logger(), "Activated");
    return CallbackReturn::SUCCESS;
}

FusionNode::CallbackReturn FusionNode::on_deactivate(const rclcpp_lifecycle::State& state)
{
    points_sub_.reset();
    imu_sub_.reset();
    wheel_sub_.reset();
    marker_sub_.reset();
    vo_sub_.reset();
    bias_reestimate_timer_.reset();
    diagnostics_timer_.reset();
    odom_timer_.reset();
    LifecycleNode::on_deactivate(state);
    return CallbackReturn::SUCCESS;
}

FusionNode::CallbackReturn FusionNode::on_cleanup(const rclcpp_lifecycle::State&)
{
    points_sub_.reset();
    imu_sub_.reset();
    wheel_sub_.reset();
    marker_sub_.reset();
    vo_sub_.reset();
    odom_pub_.reset();
    smooth_odom_pub_.reset();
    tf_broadcaster_.reset();
    debug_path_pub_.reset();
    debug_keyframes_pub_.reset();
    debug_map_edges_pub_.reset();
    debug_map_planars_pub_.reset();
    debug_scan_edges_pub_.reset();
    debug_scan_planars_pub_.reset();
    debug_fiducials_pub_.reset();
    debug_bias_pub_.reset();
    debug_velocity_pub_.reset();
    preprocessor_.reset();
    feature_extractor_.reset();
    scan_matcher_.reset();
    keyframe_map_.reset();
    pose_graph_.reset();
    return CallbackReturn::SUCCESS;
}

void FusionNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    ImuData sample;
    sample.timestamp = static_cast<double>(msg->header.stamp.sec) +
                       1e-9 * msg->header.stamp.nanosec + imu_time_offset_;
    logSensorLatency("imu", sample.timestamp);
    health_monitor_.beat("imu", sample.timestamp);
    sample.angular_velocity =
        Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    sample.linear_acceleration = Eigen::Vector3d(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z);
    imu_buffer_.push(sample);

    if (!imu_init_done_)
        handleImuInit(sample.timestamp);
}

void FusionNode::handleImuInit(double stamp)
{
    if (imu_init_first_stamp_ < 0.0)
        imu_init_first_stamp_ = stamp;
    if (imu_init_window_start_ < 0.0)
        imu_init_window_start_ = stamp;

    if (stamp - imu_init_window_start_ < imu_init_duration_s_)
    {
        // Give up rather than deadlock: a robot that starts moving immediately
        // still runs, just without a bias estimate.
        if (stamp - imu_init_first_stamp_ > imu_init_max_wait_s_)
        {
            RCLCPP_WARN(
                get_logger(),
                "IMU init: no stationary window within %.1f s - proceeding with zero gyro bias",
                imu_init_max_wait_s_);
            imu_init_done_ = true;
        }
        return;
    }

    const auto stats = imu_buffer_.windowStats(imu_init_window_start_, stamp);

    bool stationary = stats.count >= 10 && stats.gyro_deviation < stationary_gyro_thresh_;
    if (stationary && wheel_buffer_.hasData())
    {
        const auto wheel_motion = wheel_buffer_.relativePose(imu_init_window_start_, stamp);
        if (wheel_motion && wheel_motion->translation().norm() > stationary_wheel_thresh_)
            stationary = false;
    }

    if (!stationary)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            3000,
            "IMU init: motion detected during bias window - restarting collection");
        imu_init_window_start_ = stamp;
        return;
    }

    imu_buffer_.setGyroBias(stats.gyro_mean);
    imu_init_done_ = true;

    // Sanity checks that catch the classic driver misconfigurations.
    const double accel_norm = stats.accel_mean.norm();
    if (std::abs(accel_norm - constants::GRAVITY) > 0.2 * constants::GRAVITY)
        RCLCPP_WARN(
            get_logger(),
            "IMU init: |accel| = %.2f m/s^2, expected ~9.8 - check units (g vs m/s^2)",
            accel_norm);
    if (stats.gyro_mean.norm() > 0.05)
        RCLCPP_WARN(
            get_logger(),
            "IMU init: gyro bias %.4f rad/s is unusually large - check units (deg/s vs rad/s)",
            stats.gyro_mean.norm());
    const double tilt =
        std::acos(std::clamp(stats.accel_mean.normalized().z(), -1.0, 1.0)) * constants::RAD_TO_DEG;
    if (tilt > 5.0)
        RCLCPP_WARN(
            get_logger(),
            "IMU init: gravity is %.1f deg off the base +z axis - check imu_rpy mounting "
            "rotation (or the robot is on a slope)",
            tilt);

    RCLCPP_INFO(
        get_logger(),
        "IMU init complete: gyro bias [%.5f %.5f %.5f] rad/s over %zu samples "
        "(|accel| %.2f, tilt %.1f deg)",
        stats.gyro_mean.x(),
        stats.gyro_mean.y(),
        stats.gyro_mean.z(),
        stats.count,
        accel_norm,
        tilt);
}

void FusionNode::logSensorLatency(const char* sensor, double stamp)
{
    // One-shot characterization aid: the first few messages per sensor get an
    // arrival-minus-stamp report so clock offsets and driver latency are
    // visible at bring-up without extra tooling.
    int& count = latency_logged_[sensor];
    if (count >= 3)
        return;
    ++count;
    const double now = static_cast<double>(get_clock()->now().nanoseconds()) * 1e-9;
    RCLCPP_INFO(
        get_logger(),
        "%s stamp-to-arrival latency: %+.1f ms%s",
        sensor,
        (now - stamp) * 1e3,
        count == 3 ? " (final report; tune *_time_offset_s if large)" : "");
}

bool FusionNode::markerMotionGate(double stamp)
{
    // Real cameras blur under fast motion and the detector's centroid lags;
    // anchoring from such frames imprints the error into the graph.
    const double yaw_rate = std::abs(imu_buffer_.rateNear(stamp).z());
    if (yaw_rate > marker_max_yaw_rate_)
    {
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Marker detections rejected: yaw rate %.2f rad/s above limit",
            yaw_rate);
        return false;
    }
    if (wheel_buffer_.hasData())
    {
        const auto delta = wheel_buffer_.relativePose(stamp - 0.2, stamp);
        if (delta && delta->translation().norm() / 0.2 > marker_max_speed_)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Marker detections rejected: speed above limit");
            return false;
        }
    }
    return true;
}

void FusionNode::loadExtrinsicsFromTf()
{
    const double      timeout      = get_parameter("extrinsics_tf_timeout_s").as_double();
    const std::string lidar_frame  = get_parameter("lidar_frame").as_string();
    const std::string camera_frame = get_parameter("camera_frame").as_string();

    // on_configure runs on this node's executor thread, so the listener must
    // spin its own internal node (spin_thread=true, the default) or the
    // static transforms could never be received while we wait here.
    tf2_ros::Buffer            buffer(get_clock());
    tf2_ros::TransformListener listener(buffer);

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(timeout));
    bool available = false;
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (buffer.canTransform(base_frame_, lidar_frame, tf2::TimePointZero) &&
            buffer.canTransform(base_frame_, camera_frame, tf2::TimePointZero))
        {
            available = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!available)
    {
        RCLCPP_WARN(
            get_logger(),
            "extrinsics_from_tf: %s->{%s, %s} not available within %.1f s - keeping the "
            "parameter-provided extrinsics",
            base_frame_.c_str(),
            lidar_frame.c_str(),
            camera_frame.c_str(),
            timeout);
        return;
    }

    const Eigen::Isometry3d base_from_lidar =
        tf2::transformToEigen(buffer.lookupTransform(base_frame_, lidar_frame, tf2::TimePointZero));
    preprocessor_config_.base_from_lidar = base_from_lidar.cast<float>();

    const Eigen::Isometry3d base_from_cam = tf2::transformToEigen(
        buffer.lookupTransform(base_frame_, camera_frame, tf2::TimePointZero));
    base_from_camera_ = gtsam::Pose3(
        gtsam::Rot3(base_from_cam.rotation()),
        gtsam::Point3(base_from_cam.translation()));

    // The odom->base TF child (base_footprint) sits a static offset from the
    // base frame; prefer the URDF's value when it is available on TF.
    if (publish_odom_tf_ && buffer.canTransform(odom_child_frame_, base_frame_, tf2::TimePointZero))
    {
        const Eigen::Isometry3d child_from_base = tf2::transformToEigen(
            buffer.lookupTransform(odom_child_frame_, base_frame_, tf2::TimePointZero));
        child_from_base_ = gtsam::Pose3(
            gtsam::Rot3(child_from_base.rotation()),
            gtsam::Point3(child_from_base.translation()));
    }

    RCLCPP_INFO(
        get_logger(),
        "Extrinsics from TF: lidar [%.3f %.3f %.3f], camera [%.3f %.3f %.3f] (frame '%s' - "
        "make sure the detector reports positions in this frame's convention)",
        base_from_lidar.translation().x(),
        base_from_lidar.translation().y(),
        base_from_lidar.translation().z(),
        base_from_cam.translation().x(),
        base_from_cam.translation().y(),
        base_from_cam.translation().z(),
        camera_frame.c_str());
}

void FusionNode::reestimateGyroBias()
{
    if (!imu_init_done_ || !imu_buffer_.hasData())
        return;

    const double now = static_cast<double>(get_clock()->now().nanoseconds()) * 1e-9;
    const double t0  = now - 1.0;

    // Only while demonstrably stationary: wheels quiet and gyro spread small.
    if (wheel_buffer_.hasData())
    {
        const auto wheel_motion = wheel_buffer_.relativePose(t0, now);
        if (!wheel_motion || wheel_motion->translation().norm() > stationary_wheel_thresh_)
            return;
    }
    const auto stats = imu_buffer_.windowStats(t0, now);
    if (stats.count < 10 || stats.gyro_deviation > stationary_gyro_thresh_)
        return;

    // Slow exponential update: tracks temperature drift without ever jumping.
    const Eigen::Vector3d updated = 0.9 * imu_buffer_.gyroBias() + 0.1 * stats.gyro_mean;
    imu_buffer_.setGyroBias(updated);
}

void FusionNode::wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    const double stamp = static_cast<double>(msg->header.stamp.sec) +
                         1e-9 * msg->header.stamp.nanosec + wheel_time_offset_;
    logSensorLatency("wheel", stamp);
    health_monitor_.beat("wheel", stamp);
    const gtsam::Pose3 wheel_pose = gtsam_conversions::toGtsamPose(msg->pose.pose);
    wheel_buffer_.push(stamp, wheel_pose);
    if (!wheel_origin_)
        wheel_origin_ = wheel_pose;  // anchors the smooth odom frame at node start
    last_wheel_twist_ = msg->twist.twist;
}

void FusionNode::markerCallback(const whycode_vision::msg::WhyCodePoseArray::SharedPtr msg)
{
    const double stamp = static_cast<double>(msg->header.stamp.sec) +
                         1e-9 * msg->header.stamp.nanosec + camera_time_offset_;
    logSensorLatency("camera", stamp);
    health_monitor_.beat("markers", stamp);
    if (!markerMotionGate(stamp))
        return;
    for (const auto& detection : msg->poses)
    {
        marker_gate_->push(
            stamp,
            detection.whycode_id,
            detection.tracking_id,
            detection.id_valid,
            gtsam_conversions::toGtsamPoint(detection.pose.position));
    }
}

void FusionNode::bootstrapFirstKeyframe(const FeatureClouds& features)
{
    const gtsam::Pose3 origin;
    pose_graph_->addFirstKeyframe(origin);
    if (imu_preintegration_)
    {
        // Stationary start (enforced by the IMU init gate): V(0) = 0 and
        // B(0) seeded with the measured gyro bias.
        pose_graph_->addImuPriors(imu_buffer_.gyroBias(), 0.1, 0.1, 0.01);
    }
    if (pose_graph_->optimize() == PoseGraph::OptimizeResult::FAILED)
    {
        RCLCPP_ERROR(get_logger(), "Bootstrap graph update failed - retrying on the next scan");
        return;
    }

    Cloud::Ptr edge_copy(new Cloud(*features.edge));
    Cloud::Ptr planar_copy(new Cloud(*features.planar));
    keyframe_map_->add(origin, edge_copy, planar_copy, features.stamp);

    last_scan_pose_  = origin;
    last_scan_stamp_ = features.stamp;

    // Seed the smooth stream with the motion accumulated since node start so
    // the odom frame stays anchored where the first wheel sample was taken.
    smooth_pose_ = gtsam::Pose3();
    if (wheel_origin_)
    {
        const auto wheel_now = wheel_buffer_.poseAt(features.stamp);
        if (wheel_now)
            smooth_pose_ = wheel_origin_->between(*wheel_now);
    }
    updateMapOdomCorrection();
}

gtsam::Pose3 FusionNode::predictPose(double scan_stamp) const
{
    // After a scan gap (sensor outage), the constant-velocity increment is
    // stale garbage — the wheel-measured motion over the gap replaces it.
    if (last_scan_stamp_ > 0.0 && scan_stamp - last_scan_stamp_ > prediction_gap_fallback_s_)
    {
        const auto wheel_relative = wheel_buffer_.relativePose(last_scan_stamp_, scan_stamp);
        if (wheel_relative)
            return last_scan_pose_ * (*wheel_relative);
    }

    // Translation prediction comes from the wheels when they cover the
    // interval — the platform's own velocity truth. A constant-velocity
    // extrapolation is only the fallback: in weakly-observable scenes
    // (corridors) the matcher cannot correct along the degenerate axis, so
    // an extrapolated guess feeds back into the estimate and pumps a
    // translation runaway. Rotation comes from the gyro either way.
    gtsam::Pose3 increment = last_increment_;
    if (last_scan_stamp_ > 0.0)
    {
        const auto wheel_relative = wheel_buffer_.relativePose(last_scan_stamp_, scan_stamp);
        if (wheel_relative)
            increment = *wheel_relative;
    }
    if (imu_buffer_.hasData() && last_scan_stamp_ > 0.0)
    {
        const Eigen::Quaterniond gyro_rotation =
            imu_buffer_.relativeRotation(last_scan_stamp_, scan_stamp);
        increment = gtsam::Pose3(gtsam::Rot3(gyro_rotation), increment.translation());
    }
    return last_scan_pose_ * increment;
}

void FusionNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    const auto pipeline_start = std::chrono::steady_clock::now();

    // Hold scans until the IMU bias window resolves; a robot without an IMU
    // still starts once the grace period passes.
    if (!imu_init_done_)
    {
        const double stamp =
            static_cast<double>(msg->header.stamp.sec) + 1e-9 * msg->header.stamp.nanosec;
        if (first_scan_stamp_ < 0.0)
            first_scan_stamp_ = stamp;
        if (!imu_buffer_.hasData() && stamp - first_scan_stamp_ > imu_init_max_wait_s_)
        {
            RCLCPP_WARN(
                get_logger(),
                "No IMU data after %.1f s - running without gyro",
                imu_init_max_wait_s_);
            imu_init_done_ = true;
        }
        else
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Waiting for IMU initialization before processing scans");
            return;
        }
    }

    if (!preprocessor_->process(*msg, scan_image_))
        return;

    logSensorLatency("lidar", scan_image_.stamp);
    health_monitor_.beat("lidar", scan_image_.stamp);

    if (deskew_enabled_ && !scan_image_.rel_time.empty() && imu_buffer_.hasData())
    {
        const auto [min_it, max_it] =
            std::minmax_element(scan_image_.rel_time.begin(), scan_image_.rel_time.end());
        const double t_min = *min_it;
        const double t_max = *max_it;
        if (t_max - t_min > 1e-4)  // no-op for clouds without a time field
        {
            deskewScan(
                scan_image_,
                imu_buffer_.sampleRotations(
                    scan_image_.stamp + t_min,
                    scan_image_.stamp + t_max,
                    deskew_time_bins_),
                t_min,
                t_max);
        }
    }

    feature_extractor_->extract(scan_image_, features_);

    if (keyframe_map_->empty())
    {
        bootstrapFirstKeyframe(features_);
        publishOdometry(last_scan_pose_, features_.stamp);
        publishSmoothOdometry(smooth_pose_, features_.stamp);
        return;
    }

    const gtsam::Pose3 guess = predictPose(features_.stamp);

    Cloud::Ptr edge_map;
    Cloud::Ptr planar_map;
    keyframe_map_->buildLocalMap(guess.translation(), features_.stamp, edge_map, planar_map);
    scan_matcher_->setTarget(edge_map, planar_map);
    last_edge_map_   = edge_map;
    last_planar_map_ = planar_map;

    MatcherPose matcher_pose =
        MatcherPose::fromAffine(Eigen::Affine3f(guess.matrix().cast<float>()));
    gtsam::Pose3 scan_pose = guess;
    last_match_ok_         = scan_matcher_->align(features_, matcher_pose);
    last_match_degenerate_ = scan_matcher_->isDegenerate();
    if (last_match_ok_ && last_match_degenerate_)
        health_monitor_.flagQuality("lidar", SensorHealth::DEGRADED, "degenerate geometry");
    else if (!last_match_ok_)
        health_monitor_.flagQuality("lidar", SensorHealth::POOR, "scan match failed");
    if (last_match_ok_)
    {
        if (planar_motion_)
        {
            // Ground robot: the graph estimates x, y, yaw; z / roll / pitch
            // are structurally zero and clamping them blocks slow vertical
            // drift in scenes where the floor is barely visible.
            matcher_pose.z     = 0.0F;
            matcher_pose.roll  = 0.0F;
            matcher_pose.pitch = 0.0F;
        }
        scan_pose = gtsam::Pose3(
            gtsam::Rot3::Ypr(matcher_pose.yaw, matcher_pose.pitch, matcher_pose.roll),
            gtsam::Point3(matcher_pose.x, matcher_pose.y, matcher_pose.z));
    }
    else
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Scan matching failed; coasting on prediction");
    }

    last_increment_ = last_scan_pose_.between(scan_pose);

    // Advance the smooth odom-frame pose by the correction-independent
    // body increment. After a long LiDAR gap the scan increment carries the
    // accumulated scan-vs-wheel correction of the whole outage — that
    // correction belongs in map->odom, so the wheel increment is used there.
    gtsam::Pose3 smooth_increment = last_increment_;
    if (last_scan_stamp_ > 0.0 && features_.stamp - last_scan_stamp_ > prediction_gap_fallback_s_)
    {
        const auto wheel_gap = wheel_buffer_.relativePose(last_scan_stamp_, features_.stamp);
        if (wheel_gap)
            smooth_increment = *wheel_gap;
    }
    smooth_pose_ = smooth_pose_ * smooth_increment;

    last_scan_pose_  = scan_pose;
    last_scan_stamp_ = features_.stamp;

    if (keyframe_map_->shouldAddKeyframe(scan_pose))
    {
        const double       previous_stamp = keyframe_map_->back().stamp;
        const gtsam::Pose3 relative       = keyframe_map_->back().pose.between(scan_pose);

        // A dead-reckoned or degenerate keyframe must not carry the tight
        // scan-match confidence, or it silently corrupts the graph.
        double lidar_scale = !last_match_ok_ ?
                                 match_fail_sigma_scale_ :
                                 (last_match_degenerate_ ? degenerate_sigma_scale_ : 1.0);

        const auto wheel_relative =
            use_wheel_odom_ ? wheel_buffer_.relativePose(previous_stamp, features_.stamp) :
                              std::nullopt;

        // Eigenvalue thresholds are scene-dependent; the wheels are a direct
        // witness. A scan match claiming substantially MORE or LESS motion
        // than the wheels measured loses its authority no matter what the
        // eigenvalues said. Only the distance is compared — direction
        // differences at turn keyframes are yaw effects the wheel factor
        // already handles, and vector comparison false-fires on them.
        bool wheel_disagrees = false;
        if (wheel_relative)
        {
            const double distance_gap =
                std::abs(relative.translation().norm() - wheel_relative->translation().norm());
            if (distance_gap > wheel_lidar_disagree_m_)
            {
                wheel_disagrees = true;
                lidar_scale     = std::max(lidar_scale, degenerate_sigma_scale_);
                health_monitor_.flagQuality(
                    "lidar",
                    SensorHealth::DEGRADED,
                    "disagrees with wheel odometry");
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "Scan match distance disagrees with wheels by %.2f m - "
                    "widening the LiDAR factor",
                    distance_gap);
            }
        }

        FactorSigmas lidar_sigmas = lidar_between_sigmas_;
        for (double& sigma : lidar_sigmas)
            sigma *= lidar_scale;
        pose_graph_->addKeyframe(scan_pose, relative, lidar_sigmas);

        if (use_wheel_odom_)
        {
            if (wheel_relative)
            {
                // Slip enters through rotation: widen the wheel factor with
                // the turning and distance actually covered this interval.
                const double turn = std::abs(
                    Eigen::AngleAxisd(imu_buffer_.relativeRotation(previous_stamp, features_.stamp))
                        .angle());
                const double dist = wheel_relative->translation().norm();

                FactorSigmas wheel_sigmas = wheel_between_sigmas_;
                wheel_sigmas[5] *= 1.0 + wheel_yaw_sigma_per_rad_ * turn;
                wheel_sigmas[0] *=
                    1.0 + wheel_dist_sigma_per_m_ * dist + wheel_yaw_sigma_per_rad_ * turn;
                wheel_sigmas[1] = wheel_sigmas[0];
                pose_graph_->addOdometryFactor(*wheel_relative, wheel_sigmas);
            }
        }
        if (use_vo_)
        {
            // VO increments are wheel-scaled and only trustworthy in-plane:
            // loose z/roll/pitch, robustified against tracking failures.
            const auto vo_relative = vo_buffer_.relativePose(previous_stamp, features_.stamp);
            if (vo_relative)
                pose_graph_->addOdometryFactor(*vo_relative, vo_between_sigmas_, true);
        }
        if (use_planar_prior_)
            pose_graph_->addPlanarPrior(planar_prior_sigmas_);

        if (imu_preintegration_)
        {
            // Preintegrate the RAW samples over the keyframe interval (kept
            // off the IMU hot path; ~200 samples at keyframe rate). Skip the
            // factor when the buffer doesn't cover the interval (long outage
            // vs. buffer history) — the lidar/wheel betweens still chain and
            // the graph re-seeds V/B at the next covered keyframe.
            const auto samples = imu_buffer_.samplesBetween(previous_stamp, features_.stamp);
            const bool covered =
                samples.size() >= 2 && samples.back().timestamp - previous_stamp >
                                           (features_.stamp - previous_stamp) - 0.1;
            if (covered)
            {
                pim_->resetIntegrationAndSetBias(pose_graph_->latestBias());
                double prev_t = previous_stamp;
                for (const ImuData& s : samples)
                {
                    const double dt = s.timestamp - prev_t;
                    if (dt > 0.0)
                        pim_->integrateMeasurement(s.linear_acceleration, s.angular_velocity, dt);
                    prev_t = s.timestamp;
                }
                pose_graph_->addCombinedImuFactor(*pim_, planar_motion_);
            }
        }

        int  anchors             = 0;
        bool surveyed_this_round = false;
        if (use_markers_)
        {
            for (const MarkerObservation& obs :
                 marker_gate_->collectNear(features_.stamp, marker_stamp_window_))
            {
                if (marker_landmark_mode_)
                {
                    // TagSLAM-style: the marker is a landmark variable.
                    // Surveyed ids carry a world prior (anchoring); everything
                    // else is a free landmark whose repeated sightings act as
                    // an odometry constraint.
                    const auto survey =
                        obs.decoded ? known_markers_.find(obs.marker_id) : known_markers_.end();
                    const bool surveyed = survey != known_markers_.end();
                    // Gauge guard: a free landmark initialized before the
                    // first survey anchor encodes the spawn frame and later
                    // fights the anchor snap — hold free landmarks back until
                    // the trajectory is world-anchored.
                    if (!surveyed && !world_anchored_)
                        continue;
                    surveyed_this_round = surveyed_this_round || surveyed;
                    pose_graph_->addMarkerObservation(
                        obs.landmark_key_id,
                        obs.position_in_camera,
                        base_from_camera_,
                        marker_sigma_m_,
                        surveyed ? std::optional<gtsam::Point3>(survey->second) : std::nullopt,
                        marker_survey_sigma_m_);
                }
                else
                {
                    pose_graph_->addMarkerAnchor(
                        obs.position_in_camera,
                        known_markers_.at(obs.marker_id),
                        base_from_camera_,
                        marker_sigma_m_);
                }
                anchor_event_times_[obs.landmark_key_id] = features_.stamp;
                ++anchors;
            }
        }

        const auto   optimize_start = std::chrono::steady_clock::now();
        const auto   result         = pose_graph_->optimize();
        const double optimize_ms    = std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() - optimize_start)
                                       .count();
        if (optimize_ms > optimize_budget_warn_ms_)
            RCLCPP_WARN(
                get_logger(),
                "Graph update took %.1f ms (budget %.0f ms)",
                optimize_ms,
                optimize_budget_warn_ms_);

        if (result == PoseGraph::OptimizeResult::FAILED)
        {
            // The keyframe was rolled back; keep publishing the scan-matched
            // pose and try again at the next keyframe trigger.
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Graph update failed - keyframe discarded, coasting on scan matching");
            updateMapOdomCorrection();
            publishOdometry(last_scan_pose_, features_.stamp);
            publishSmoothOdometry(smooth_pose_, features_.stamp);
            publishScanDebug(features_.stamp);
            return;
        }
        const bool corrected = result == PoseGraph::OptimizeResult::CORRECTED;
        if (surveyed_this_round)
            world_anchored_ = true;  // gauge fixed: free landmarks may enter

        if (imu_preintegration_)
        {
            // Feed the online bias estimate back to the buffer so deskew,
            // prediction and rateNear() all run with the graph's best bias.
            const auto bias = pose_graph_->latestBias();
            imu_buffer_.setGyroBias(bias.gyroscope());

            if (debug_imu_state_ && debug_bias_pub_->is_activated())
            {
                geometry_msgs::msg::AccelStamped bias_msg;
                bias_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(features_.stamp * 1e9));
                bias_msg.header.frame_id = base_frame_;
                bias_msg.accel.linear.x  = bias.accelerometer().x();
                bias_msg.accel.linear.y  = bias.accelerometer().y();
                bias_msg.accel.linear.z  = bias.accelerometer().z();
                bias_msg.accel.angular.x = bias.gyroscope().x();
                bias_msg.accel.angular.y = bias.gyroscope().y();
                bias_msg.accel.angular.z = bias.gyroscope().z();
                debug_bias_pub_->publish(bias_msg);

                geometry_msgs::msg::Vector3Stamped vel_msg;
                vel_msg.header      = bias_msg.header;
                const auto velocity = pose_graph_->latestVelocity();
                vel_msg.vector.x    = velocity.x();
                vel_msg.vector.y    = velocity.y();
                vel_msg.vector.z    = velocity.z();
                debug_velocity_pub_->publish(vel_msg);
            }
        }

        const gtsam::Pose3 optimized = pose_graph_->latestPose();
        Cloud::Ptr         edge_copy(new Cloud(*features_.edge));
        Cloud::Ptr         planar_copy(new Cloud(*features_.planar));
        keyframe_map_->add(
            optimized,
            edge_copy,
            planar_copy,
            features_.stamp,
            !last_match_ok_ || last_match_degenerate_ || wheel_disagrees);

        if (corrected)
        {
            refreshAfterCorrection();
            RCLCPP_INFO(
                get_logger(),
                "Global correction applied (%d marker observation%s) - trajectory corrected",
                anchors,
                anchors == 1 ? "" : "s");
        }

        last_scan_pose_ = corrected ? pose_graph_->latestPose() : optimized;
        publishKeyframeDebug(corrected, features_.stamp);

        if (loop_closure_enabled_)
            attemptLoopClosure(features_.stamp);
    }

    // The correction is re-derived at the scan stamp where the fused pose and
    // the smooth pose describe the same instant; any anchor/loop jump from
    // this cycle lands here (map->odom) and never in the smooth stream.
    updateMapOdomCorrection();
    publishOdometry(last_scan_pose_, features_.stamp);
    publishSmoothOdometry(smooth_pose_, features_.stamp);
    publishScanDebug(features_.stamp);

    const double pipeline_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - pipeline_start)
            .count();
    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "scan: %zu pts | edges %zu planars %zu | map %zu/%zu | kf %zu | %s | eig[%.0f %.0f %.0f] "
        "| %.1f ms",
        scan_image_.points->size(),
        features_.edge->size(),
        features_.planar->size(),
        edge_map->size(),
        planar_map->size(),
        keyframe_map_->size(),
        scan_matcher_->isDegenerate() ? "DEGEN" : "ok",
        static_cast<double>(scan_matcher_->constraintEigenvalues()(0)),
        static_cast<double>(scan_matcher_->constraintEigenvalues()(1)),
        static_cast<double>(scan_matcher_->constraintEigenvalues()(2)),
        pipeline_ms);
}

void FusionNode::refreshAfterCorrection()
{
    // A global factor bent the past trajectory: refresh every stored
    // keyframe pose and drop the transformed-cloud cache.
    const auto poses = pose_graph_->allPoses();
    for (size_t i = 0; i < poses.size(); ++i)
        keyframe_map_->updatePose(i, poses[i]);
    keyframe_map_->invalidateCache();
}

void FusionNode::attemptLoopClosure(double stamp)
{
    if (keyframe_map_->size() < 10)
        return;
    if (last_loop_attempt_ >= 0.0 && stamp - last_loop_attempt_ < loop_min_interval_s_)
        return;
    last_loop_attempt_ = stamp;

    const size_t current = keyframe_map_->size() - 1;
    const auto   loop    = loop_detector_->detect(*keyframe_map_, current);
    if (!loop)
        return;

    pose_graph_->addLoopFactor(
        loop->old_index,
        current,
        loop->relative,
        std::max(loop->fitness, loop_sigma_floor_));
    if (pose_graph_->optimize() != PoseGraph::OptimizeResult::CORRECTED)
        return;

    refreshAfterCorrection();
    last_scan_pose_ = pose_graph_->latestPose();
    publishKeyframeDebug(true, stamp);
    RCLCPP_INFO(
        get_logger(),
        "Loop closed: keyframe %zu revisits %zu (fitness %.3f) - trajectory corrected",
        current,
        loop->old_index,
        loop->fitness);
}

void FusionNode::publishDiagnostics()
{
    if (!diagnostics_pub_ || !diagnostics_pub_->is_activated())
        return;

    const double now = static_cast<double>(get_clock()->now().nanoseconds()) * 1e-9;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = get_clock()->now();

    for (const auto& status : health_monitor_.evaluate(now))
    {
        diagnostic_msgs::msg::DiagnosticStatus diag;
        diag.name        = "olive/" + status.name;
        diag.hardware_id = status.name;

        switch (status.health)
        {
            case SensorHealth::FAILED:
                diag.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                diag.message = status.age < 0.0 ? "no data" : "timed out";
                break;
            case SensorHealth::POOR:
            case SensorHealth::DEGRADED:
                diag.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                diag.message = status.detail.empty() ? "degraded" : status.detail;
                break;
            default:
                diag.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
                diag.message = "ok";
        }

        diagnostic_msgs::msg::KeyValue age;
        age.key   = "age_s";
        age.value = std::to_string(status.age);
        diag.values.push_back(age);

        if (status.name == "lidar" && scan_matcher_)
        {
            diagnostic_msgs::msg::KeyValue eig;
            eig.key            = "constraint_eigenvalues";
            const auto& values = scan_matcher_->constraintEigenvalues();
            eig.value          = std::to_string(values(0)) + " " + std::to_string(values(1)) + " " +
                        std::to_string(values(2));
            diag.values.push_back(eig);
        }
        array.status.push_back(diag);
    }
    diagnostics_pub_->publish(array);
}

void FusionNode::odomTick()
{
    // Before the first scan: wheel passthrough keeps the odom TF and local
    // odometry alive during IMU init (Nav2 can look up odom->base right away).
    if (last_scan_stamp_ < 0.0)
    {
        if (publish_odom_tf_ && wheel_origin_ && wheel_buffer_.hasData())
        {
            const double wheel_now = wheel_buffer_.latestStamp();
            const auto   latest    = wheel_buffer_.poseAt(wheel_now);
            if (latest)
                publishSmoothOdometry(wheel_origin_->between(*latest), wheel_now);
        }
        return;
    }

    if (!wheel_buffer_.hasData())
        return;
    const double wheel_now = wheel_buffer_.latestStamp();
    if (wheel_now <= last_scan_stamp_)
        return;

    const auto wheel_relative = wheel_buffer_.relativePose(last_scan_stamp_, wheel_now);
    if (!wheel_relative)
        return;

    // Transient extension of the smooth pose between scans: wheel translation
    // with gyro rotation (the predictPose recipe). State is not mutated — the
    // next scan increment supersedes this extension.
    gtsam::Pose3 extension = *wheel_relative;
    if (imu_buffer_.hasData())
        extension = gtsam::Pose3(
            gtsam::Rot3(imu_buffer_.relativeRotation(last_scan_stamp_, wheel_now)),
            extension.translation());
    const gtsam::Pose3 smooth_now = smooth_pose_ * extension;
    if (publish_odom_tf_)
        publishSmoothOdometry(smooth_now, wheel_now);

    // LiDAR dropout: keep the map-frame output alive on wheel dead-reckoning
    // while the LiDAR is silent (tier-1); the graph itself is left untouched.
    if (!coast_on_dropout_ || wheel_now - last_scan_stamp_ < lidar_dropout_timeout_)
        return;

    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "No LiDAR for %.1f s - coasting on wheel odometry",
        wheel_now - last_scan_stamp_);
    // In odom-owning mode the coast pose follows the TF chain exactly (the
    // correction stays frozen: no new global information during an outage).
    const gtsam::Pose3 coast_pose = publish_odom_tf_ && map_odom_valid_ ?
                                        map_from_odom_ * smooth_now * child_from_base_ :
                                        last_scan_pose_ * (*wheel_relative);
    publishOdometry(coast_pose, wheel_now);

    // Tier 2 (optional): wheel-paced keyframes so marker anchors can still
    // land during the outage.
    if (dropout_keyframes_ && !keyframe_map_->empty() &&
        keyframe_map_->shouldAddKeyframe(coast_pose))
    {
        const double previous_stamp    = keyframe_map_->back().stamp;
        const auto   keyframe_relative = wheel_buffer_.relativePose(previous_stamp, wheel_now);
        if (!keyframe_relative)
            return;

        pose_graph_->addKeyframe(coast_pose, *keyframe_relative, wheel_between_sigmas_);
        if (use_planar_prior_)
            pose_graph_->addPlanarPrior(planar_prior_sigmas_);
        if (pose_graph_->optimize() == PoseGraph::OptimizeResult::FAILED)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Coast keyframe rejected by the graph");
            return;
        }
        keyframe_map_
            ->add(pose_graph_->latestPose(), nullptr, nullptr, wheel_now, /*low_quality=*/true);
        // Keep the smooth stream in step with the advanced scan state, then
        // re-derive the correction from the optimized pose.
        smooth_pose_     = smooth_pose_ * extension;
        last_scan_pose_  = pose_graph_->latestPose();
        last_scan_stamp_ = wheel_now;
        updateMapOdomCorrection();
    }
}

namespace
{

void toCloudMsg(
    const Cloud&                   cloud,
    const std::string&             frame,
    const rclcpp::Time&            stamp,
    sensor_msgs::msg::PointCloud2& msg)
{
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = frame;
    msg.header.stamp    = stamp;
}

}  // namespace

void FusionNode::publishKeyframeDebug(bool trajectory_corrected, double stamp)
{
    if (!debug_enabled_)
        return;
    const rclcpp::Time ros_stamp(static_cast<int64_t>(stamp * 1e9));

    if (debug_path_)
    {
        if (trajectory_corrected)
        {
            // Past poses moved: rebuild the whole path so it stays honest.
            debug_path_msg_.poses.clear();
            for (const gtsam::Pose3& pose : pose_graph_->allPoses())
            {
                geometry_msgs::msg::PoseStamped ps;
                ps.header.frame_id = map_frame_;
                ps.pose            = gtsam_conversions::toRosPose(pose);
                debug_path_msg_.poses.push_back(ps);
            }
        }
        else
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id = map_frame_;
            ps.header.stamp    = ros_stamp;
            ps.pose            = gtsam_conversions::toRosPose(pose_graph_->latestPose());
            debug_path_msg_.poses.push_back(ps);
        }
        debug_path_msg_.header.stamp = ros_stamp;
        debug_path_pub_->publish(debug_path_msg_);
    }

    if (debug_keyframes_)
    {
        // Incrementally appended; rebuilt only when past poses moved — a
        // full allPoses() sweep per keyframe would defeat the incremental
        // graph query path.
        if (trajectory_corrected)
        {
            debug_keyframes_msg_.poses.clear();
            for (const gtsam::Pose3& pose : pose_graph_->allPoses())
                debug_keyframes_msg_.poses.push_back(gtsam_conversions::toRosPose(pose));
        }
        else
        {
            debug_keyframes_msg_.poses.push_back(
                gtsam_conversions::toRosPose(pose_graph_->latestPose()));
        }
        debug_keyframes_msg_.header.frame_id = map_frame_;
        debug_keyframes_msg_.header.stamp    = ros_stamp;
        debug_keyframes_pub_->publish(debug_keyframes_msg_);
    }

    if (debug_local_map_ && last_edge_map_ && last_planar_map_)
    {
        sensor_msgs::msg::PointCloud2 msg;
        toCloudMsg(*last_edge_map_, map_frame_, ros_stamp, msg);
        debug_map_edges_pub_->publish(msg);
        toCloudMsg(*last_planar_map_, map_frame_, ros_stamp, msg);
        debug_map_planars_pub_->publish(msg);
    }

    publishFiducialDebug(stamp);
}

void FusionNode::publishScanDebug(double stamp)
{
    if (!debug_enabled_ || !debug_scan_features_)
        return;
    const bool want_edges   = debug_scan_edges_pub_->get_subscription_count() > 0;
    const bool want_planars = debug_scan_planars_pub_->get_subscription_count() > 0;
    if (!want_edges && !want_planars)
        return;

    const rclcpp::Time    ros_stamp(static_cast<int64_t>(stamp * 1e9));
    const Eigen::Affine3f transform(last_scan_pose_.matrix().cast<float>());

    sensor_msgs::msg::PointCloud2 msg;
    if (want_edges)
    {
        pcl::transformPointCloud(*features_.edge, debug_scan_cloud_, transform);
        toCloudMsg(debug_scan_cloud_, map_frame_, ros_stamp, msg);
        debug_scan_edges_pub_->publish(msg);
    }
    if (want_planars)
    {
        pcl::transformPointCloud(*features_.planar, debug_scan_cloud_, transform);
        toCloudMsg(debug_scan_cloud_, map_frame_, ros_stamp, msg);
        debug_scan_planars_pub_->publish(msg);
    }
}

void FusionNode::publishFiducialDebug(double stamp)
{
    if (!debug_fiducials_)
        return;
    constexpr double   RECENT = 3.0;  // anchor highlight duration (s)
    const rclcpp::Time ros_stamp(static_cast<int64_t>(stamp * 1e9));

    visualization_msgs::msg::MarkerArray array;
    for (const auto& [id, position] : known_markers_)
    {
        const auto event  = anchor_event_times_.find(id);
        const bool seen   = event != anchor_event_times_.end();
        const bool recent = seen && (stamp - event->second) < RECENT;

        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id    = map_frame_;
        sphere.header.stamp       = ros_stamp;
        sphere.ns                 = "fiducials";
        sphere.id                 = id;
        sphere.type               = visualization_msgs::msg::Marker::SPHERE;
        sphere.action             = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position.x    = position.x();
        sphere.pose.position.y    = position.y();
        sphere.pose.position.z    = position.z();
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.3;
        // gray = never anchored, green = anchoring now, blue = anchored before
        sphere.color.r = recent ? 0.1F : (seen ? 0.1F : 0.5F);
        sphere.color.g = recent ? 0.9F : (seen ? 0.4F : 0.5F);
        sphere.color.b = recent ? 0.1F : (seen ? 0.9F : 0.5F);
        sphere.color.a = 0.9F;
        array.markers.push_back(sphere);

        visualization_msgs::msg::Marker label = sphere;
        label.ns                              = "fiducial_labels";
        label.type                            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.text                            = "id " + std::to_string(id);
        label.pose.position.z += 0.35;
        label.scale.z = 0.25;
        label.color.r = label.color.g = label.color.b = 1.0F;
        array.markers.push_back(label);

        if (recent)
        {
            // Ray from the robot to the marker that is anchoring it right now.
            visualization_msgs::msg::Marker ray = sphere;
            ray.ns                              = "anchor_rays";
            ray.type                            = visualization_msgs::msg::Marker::LINE_LIST;
            ray.scale.x                         = 0.03;
            ray.pose                            = geometry_msgs::msg::Pose();
            ray.pose.orientation.w              = 1.0;
            geometry_msgs::msg::Point robot;
            robot.x = last_scan_pose_.translation().x();
            robot.y = last_scan_pose_.translation().y();
            robot.z = last_scan_pose_.translation().z();
            geometry_msgs::msg::Point marker_point;
            marker_point.x = position.x();
            marker_point.y = position.y();
            marker_point.z = position.z();
            ray.points     = { robot, marker_point };
            ray.color.r    = 0.1F;
            ray.color.g    = 0.9F;
            ray.color.b    = 0.1F;
            ray.color.a    = 0.9F;
            array.markers.push_back(ray);
        }
        else
        {
            visualization_msgs::msg::Marker clear;
            clear.header.frame_id = map_frame_;
            clear.ns              = "anchor_rays";
            clear.id              = id;
            clear.action          = visualization_msgs::msg::Marker::DELETE;
            array.markers.push_back(clear);
        }
    }

    // Landmark estimates (landmark mode): orange spheres at the optimized
    // positions; surveyed ids get an error segment to their survey, free
    // landmarks a "(free)" label. Convergence is visible live.
    if (marker_landmark_mode_ && pose_graph_)
    {
        for (const auto& [id, estimate] : pose_graph_->landmarks())
        {
            visualization_msgs::msg::Marker sphere;
            sphere.header.frame_id    = map_frame_;
            sphere.header.stamp       = ros_stamp;
            sphere.ns                 = "landmark_estimates";
            sphere.id                 = static_cast<int>(id);
            sphere.type               = visualization_msgs::msg::Marker::SPHERE;
            sphere.action             = visualization_msgs::msg::Marker::ADD;
            sphere.pose.position.x    = estimate.x();
            sphere.pose.position.y    = estimate.y();
            sphere.pose.position.z    = estimate.z();
            sphere.pose.orientation.w = 1.0;
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.2;
            sphere.color.r                                   = 1.0F;
            sphere.color.g                                   = 0.6F;
            sphere.color.b                                   = 0.1F;
            sphere.color.a                                   = 0.9F;
            array.markers.push_back(sphere);

            const bool undecoded = id >= UNDECODED_LANDMARK_BASE;
            const auto survey =
                undecoded ? known_markers_.end() : known_markers_.find(static_cast<int>(id));
            if (survey != known_markers_.end())
            {
                visualization_msgs::msg::Marker error_line = sphere;
                error_line.ns                              = "landmark_error";
                error_line.type               = visualization_msgs::msg::Marker::LINE_LIST;
                error_line.scale.x            = 0.02;
                error_line.pose               = geometry_msgs::msg::Pose();
                error_line.pose.orientation.w = 1.0;
                geometry_msgs::msg::Point a;
                a.x = estimate.x();
                a.y = estimate.y();
                a.z = estimate.z();
                geometry_msgs::msg::Point b;
                b.x               = survey->second.x();
                b.y               = survey->second.y();
                b.z               = survey->second.z();
                error_line.points = { a, b };
                array.markers.push_back(error_line);
            }
            else
            {
                visualization_msgs::msg::Marker label = sphere;
                label.ns                              = "landmark_labels";
                label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                label.text = undecoded ? "track " + std::to_string(id - UNDECODED_LANDMARK_BASE) +
                                             " (free)" :
                                         "id " + std::to_string(id) + " (free)";
                label.pose.position.z += 0.3;
                label.scale.z = 0.2;
                label.color.r = label.color.g = label.color.b = 1.0F;
                array.markers.push_back(label);
            }
        }
    }
    debug_fiducials_pub_->publish(array);
}

void FusionNode::publishOdometry(const gtsam::Pose3& pose, double stamp)
{
    if (!odom_pub_->is_activated())
        return;

    odom_msg_.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9));
    odom_msg_.pose.pose    = gtsam_conversions::toRosPose(pose);
    odom_pub_->publish(odom_msg_);

    // When this node owns odom->base, both TF edges are sent together in
    // publishSmoothOdometry so consumers always see a consistent snapshot.
    if (!publish_map_tf_ || publish_odom_tf_)
        return;

    // REP-105 split (wheel-owned odom frame): broadcast only the map->odom
    // correction. The continuous odom->base transform belongs to the wheel
    // odometry source, so a global update here can never teleport the robot
    // in the odom frame.
    const auto wheel_pose = wheel_buffer_.poseAt(stamp);
    if (!wheel_pose)
        return;

    const gtsam::Pose3 map_from_odom = pose * wheel_pose->inverse();

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp            = odom_msg_.header.stamp;
    tf.header.frame_id         = map_frame_;
    tf.child_frame_id          = odom_frame_;
    const auto pose_msg        = gtsam_conversions::toRosPose(map_from_odom);
    tf.transform.translation.x = pose_msg.position.x;
    tf.transform.translation.y = pose_msg.position.y;
    tf.transform.translation.z = pose_msg.position.z;
    tf.transform.rotation      = pose_msg.orientation;
    tf_broadcaster_->sendTransform(tf);
}

void FusionNode::updateMapOdomCorrection()
{
    if (!publish_odom_tf_)
        return;
    // map->odom = (map->base) o (odom->base)^-1 at the scan stamp, where
    // odom->base = smooth (odom->footprint) o (footprint->base). Anchor and
    // loop jumps land here and never in the smooth stream.
    map_from_odom_  = last_scan_pose_ * (smooth_pose_ * child_from_base_).inverse();
    map_odom_valid_ = true;
}

void FusionNode::publishSmoothOdometry(const gtsam::Pose3& odom_from_child, double stamp)
{
    if (!smooth_odom_pub_ || !smooth_odom_pub_->is_activated())
        return;

    smooth_odom_msg_.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9));
    smooth_odom_msg_.pose.pose    = gtsam_conversions::toRosPose(odom_from_child);
    smooth_odom_msg_.twist.twist  = last_wheel_twist_;
    if (imu_buffer_.hasData())
        smooth_odom_msg_.twist.twist.angular.z = imu_buffer_.rateNear(stamp).z();
    smooth_odom_pub_->publish(smooth_odom_msg_);

    if (!publish_odom_tf_)
        return;

    // Both TF edges in one broadcast so consumers always see a consistent
    // snapshot: odom->base_footprint (smooth, jump-free) and map->odom (the
    // correction — anchors and loop closures jump HERE, REP-105).
    tf_batch_.clear();

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp            = smooth_odom_msg_.header.stamp;
    tf.header.frame_id         = odom_frame_;
    tf.child_frame_id          = odom_child_frame_;
    auto pose_msg              = gtsam_conversions::toRosPose(odom_from_child);
    tf.transform.translation.x = pose_msg.position.x;
    tf.transform.translation.y = pose_msg.position.y;
    tf.transform.translation.z = pose_msg.position.z;
    tf.transform.rotation      = pose_msg.orientation;
    tf_batch_.push_back(tf);

    if (publish_map_tf_ && map_odom_valid_)
    {
        tf.header.frame_id         = map_frame_;
        tf.child_frame_id          = odom_frame_;
        pose_msg                   = gtsam_conversions::toRosPose(map_from_odom_);
        tf.transform.translation.x = pose_msg.position.x;
        tf.transform.translation.y = pose_msg.position.y;
        tf.transform.translation.z = pose_msg.position.z;
        tf.transform.rotation      = pose_msg.orientation;
        tf_batch_.push_back(tf);
    }
    tf_broadcaster_->sendTransform(tf_batch_);
}

}  // namespace olive
