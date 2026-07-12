/**
 * @file fusion_node_lifecycle.cpp
 * @brief FusionNode bring-up/teardown
 *
 * Parameter declaration + loading, lifecycle transitions, and TF-sourced
 * extrinsics. Runtime paths live in the sibling fusion_node_*.cpp translation
 * units.
 */
#include <gtsam/navigation/CombinedImuFactor.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>

#include "olive/common/gtsam_conversions.hpp"
#include "olive/fusion/frontend/feature_extractor.hpp"
#include "olive/fusion/frontend/scan_matcher.hpp"
#include "olive/fusion/frontend/scan_preprocessor.hpp"
#include "olive/fusion/fusion_node.hpp"
#include "olive/fusion/graph/keyframe_map.hpp"
#include "olive/fusion/graph/loop_detector.hpp"
#include "olive/fusion/graph/pose_graph.hpp"
#include "olive/fusion/inputs/marker_gate.hpp"

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
                {
                    debug_enabled_ = p.as_bool();
                }
                else if (p.get_name() == "debug_path")
                {
                    debug_path_ = p.as_bool();
                }
                else if (p.get_name() == "debug_keyframes")
                {
                    debug_keyframes_ = p.as_bool();
                }
                else if (p.get_name() == "debug_local_map")
                {
                    debug_local_map_ = p.as_bool();
                }
                else if (p.get_name() == "debug_scan_features")
                {
                    debug_scan_features_ = p.as_bool();
                }
                else if (p.get_name() == "debug_fiducials")
                {
                    debug_fiducials_ = p.as_bool();
                }
                else if (p.get_name() == "debug_imu_state")
                {
                    debug_imu_state_ = p.as_bool();
                }
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

// Out-of-line so unique_ptr members of forward-declared components destroy
// where their definitions are visible (compile-firewall header).
FusionNode::~FusionNode() = default;

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
    // VO publishes sparsely (only when parallax + wheel-scale gates pass), so the
    // VO buffer needs a wider interpolation slack than wheels to bracket keyframe
    // stamps; too tight and every VO between-factor is silently dropped.
    declare_parameter("vo_buffer_slack_s", 0.25);
    declare_parameter("use_markers", true);
    declare_parameter("marker_topic", "/whycode/poses");
    declare_parameter("camera_translation", std::vector<double>{ 0.2, 0.0, 0.06 });
    declare_parameter(
        "camera_rpy",
        std::vector<double>{ -std::numbers::pi / 2.0, 0.0, -std::numbers::pi / 2.0 });
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
    declare_parameter("imu_preint_max_interval_s", 5.0);
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
    vo_buffer_.setInterpolationSlack(get_parameter("vo_buffer_slack_s").as_double());

    use_markers_  = get_parameter("use_markers").as_bool();
    marker_.topic = get_parameter("marker_topic").as_string();

    const auto cam_t         = get_parameter("camera_translation").as_double_array();
    const auto cam_rpy       = get_parameter("camera_rpy").as_double_array();
    marker_.base_from_camera = gtsam::Pose3(
        gtsam::Rot3::Ypr(cam_rpy[2], cam_rpy[1], cam_rpy[0]),
        gtsam::Point3(cam_t[0], cam_t[1], cam_t[2]));

    marker_.sigma_m        = get_parameter("marker_position_sigma_m").as_double();
    marker_.stamp_window_s = get_parameter("marker_stamp_window_s").as_double();
    marker_.landmark_mode  = get_parameter("marker_mode").as_string() != "anchor";
    marker_.survey_sigma_m = get_parameter("marker_survey_sigma_m").as_double();

    MarkerGateConfig gate_config;
    gate_config.min_range = get_parameter("marker_min_range_m").as_double();
    gate_config.max_range = get_parameter("marker_max_range_m").as_double();
    gate_config.min_track_frames =
        static_cast<int>(get_parameter("marker_min_track_frames").as_int());
    // Free landmarks (unsurveyed / undecoded) only exist in landmark mode;
    // the legacy anchor path can only consume surveyed ids.
    gate_config.accept_unknown_ids =
        marker_.landmark_mode && get_parameter("marker_accept_unknown_ids").as_bool();
    gate_config.accept_undecoded_ids =
        marker_.landmark_mode && get_parameter("marker_accept_undecoded_ids").as_bool();

    marker_.known.clear();
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
        marker_.known.emplace(
            id,
            gtsam::Point3(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]));
        gate_config.known_ids.insert(id);
    }
    if (marker_.known.empty())
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

    imu_init_.duration_s   = get_parameter("imu_init_duration_s").as_double();
    imu_init_.max_wait_s   = get_parameter("imu_init_max_wait_s").as_double();
    imu_init_.gyro_thresh  = get_parameter("stationary_gyro_thresh_rad_s").as_double();
    imu_init_.wheel_thresh = get_parameter("stationary_wheel_thresh_m").as_double();
    gyro_bias_reestimate_  = get_parameter("gyro_bias_reestimate").as_bool();

    imu_preintegration_      = get_parameter("imu_preintegration").as_bool();
    imu_preint_max_interval_ = get_parameter("imu_preint_max_interval_s").as_double();
    debug_imu_state_         = get_parameter("debug_imu_state").as_bool();
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
    marker_.max_yaw_rate = get_parameter("marker_max_yaw_rate_rad_s").as_double();
    marker_.max_speed    = get_parameter("marker_max_speed_m_s").as_double();
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

FusionNode::CallbackReturn FusionNode::on_configure(const rclcpp_lifecycle::State& /*state*/)
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

    imu_init_.done         = false;
    imu_init_.window_start = -1.0;
    imu_init_.first_stamp  = -1.0;
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
    {
        vo_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            vo_topic_,
            rclcpp::QoS(10),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                const double stamp =
                    static_cast<double>(msg->header.stamp.sec) + 1e-9 * msg->header.stamp.nanosec;
                vo_buffer_.push(stamp, gtsam_conversions::toGtsamPose(msg->pose.pose));
                health_monitor_.beat("vo", stamp);
            });
    }
    if (use_markers_)
    {
        marker_sub_ = create_subscription<whycode_vision::msg::WhyCodePoseArray>(
            marker_.topic,
            rclcpp::QoS(10),
            [this](whycode_vision::msg::WhyCodePoseArray::SharedPtr msg) { markerCallback(msg); });
    }

    if (gyro_bias_reestimate_ && imu_preintegration_)
    {
        RCLCPP_WARN(
            get_logger(),
            "gyro_bias_reestimate is redundant with imu_preintegration (the graph estimates "
            "bias online) - skipping the stationary EMA timer");
    }
    else if (gyro_bias_reestimate_)
    {
        bias_reestimate_timer_ =
            create_wall_timer(std::chrono::seconds(1), [this]() { reestimateGyroBias(); });
    }

    diagnostics_timer_ = create_wall_timer(
        std::chrono::duration<double>(get_parameter("diagnostics_period_s").as_double()),
        [this]() { publishDiagnostics(); });
    // Smooth-odom extension and dropout coasting share one tick: at the
    // smooth rate when this node owns odom->base, else the legacy 5 Hz coast.
    const double tick_hz = publish_odom_tf_ ? smooth_odom_rate_hz_ : 5.0;
    if (publish_odom_tf_ || coast_on_dropout_)
    {
        odom_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / std::max(1.0, tick_hz)),
            [this]() { odomTick(); });
    }

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

FusionNode::CallbackReturn FusionNode::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
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

void FusionNode::loadExtrinsicsFromTf()
{
    const double      timeout      = get_parameter("extrinsics_tf_timeout_s").as_double();
    const std::string lidar_frame  = get_parameter("lidar_frame").as_string();
    const std::string camera_frame = get_parameter("camera_frame").as_string();

    // on_configure runs on this node's executor thread, so the listener must
    // spin its own internal node (spin_thread=true, the default) or the
    // static transforms could never be received while we wait here.
    tf2_ros::Buffer                  buffer(get_clock());
    const tf2_ros::TransformListener listener(buffer);

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
    marker_.base_from_camera = gtsam::Pose3(
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

}  // namespace olive
