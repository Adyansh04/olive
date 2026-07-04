#include "olive/fusion/fusion_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

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

    declare_parameter("lidar_translation", std::vector<double>{ 0.0, 0.0, 0.145 });
    declare_parameter("lidar_rpy", std::vector<double>{ 0.0, 0.0, 0.0 });
    declare_parameter("imu_rpy", std::vector<double>{ 0.0, 0.0, 0.0 });

    declare_parameter("imu_init_duration_s", 1.5);
    declare_parameter("imu_init_max_wait_s", 10.0);
    declare_parameter("stationary_gyro_thresh_rad_s", 0.02);
    declare_parameter("stationary_wheel_thresh_m", 0.005);
    declare_parameter("gyro_bias_reestimate", false);

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

    declare_parameter("publish_debug", false);
    declare_parameter("debug_path", true);
    declare_parameter("debug_keyframes", true);
    declare_parameter("debug_local_map", true);
    declare_parameter("debug_scan_features", true);
    declare_parameter("debug_fiducials", true);

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

    marker_sigma_m_      = get_parameter("marker_position_sigma_m").as_double();
    marker_stamp_window_ = get_parameter("marker_stamp_window_s").as_double();

    MarkerGateConfig gate_config;
    gate_config.min_range = get_parameter("marker_min_range_m").as_double();
    gate_config.max_range = get_parameter("marker_max_range_m").as_double();
    gate_config.min_track_frames =
        static_cast<int>(get_parameter("marker_min_track_frames").as_int());

    known_markers_.clear();
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
    imu_buffer_.setMountingRotation(
        Eigen::Quaterniond(Eigen::AngleAxisd(imu_rpy[2], Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(imu_rpy[1], Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(imu_rpy[0], Eigen::Vector3d::UnitX())));

    imu_init_duration_s_    = get_parameter("imu_init_duration_s").as_double();
    imu_init_max_wait_s_    = get_parameter("imu_init_max_wait_s").as_double();
    stationary_gyro_thresh_ = get_parameter("stationary_gyro_thresh_rad_s").as_double();
    stationary_wheel_thresh_ = get_parameter("stationary_wheel_thresh_m").as_double();
    gyro_bias_reestimate_   = get_parameter("gyro_bias_reestimate").as_bool();

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
    deskew_enabled_   = get_parameter("deskew_enabled").as_bool();
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

    imu_init_done_         = false;
    imu_init_window_start_ = -1.0;
    imu_init_first_stamp_  = -1.0;
    first_scan_stamp_      = -1.0;

    odom_pub_       = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
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

    debug_path_msg_.poses.clear();
    debug_path_msg_.header.frame_id = map_frame_;
    anchor_event_times_.clear();
    last_edge_map_.reset();
    last_planar_map_.reset();

    // The fused estimate lives in the map frame; the odom frame stays owned
    // by the wheel odometry source (REP-105).
    odom_msg_.header.frame_id = map_frame_;
    odom_msg_.child_frame_id  = base_frame_;

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

    if (gyro_bias_reestimate_)
        bias_reestimate_timer_ =
            create_wall_timer(std::chrono::seconds(1), [this]() { reestimateGyroBias(); });

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
    tf_broadcaster_.reset();
    debug_path_pub_.reset();
    debug_keyframes_pub_.reset();
    debug_map_edges_pub_.reset();
    debug_map_planars_pub_.reset();
    debug_scan_edges_pub_.reset();
    debug_scan_planars_pub_.reset();
    debug_fiducials_pub_.reset();
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
    sample.timestamp =
        static_cast<double>(msg->header.stamp.sec) + 1e-9 * msg->header.stamp.nanosec +
        imu_time_offset_;
    logSensorLatency("imu", sample.timestamp);
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
            get_logger(), *get_clock(), 5000,
            "Marker detections rejected: yaw rate %.2f rad/s above limit", yaw_rate);
        return false;
    }
    if (wheel_buffer_.hasData())
    {
        const auto delta = wheel_buffer_.relativePose(stamp - 0.2, stamp);
        if (delta && delta->translation().norm() / 0.2 > marker_max_speed_)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 5000,
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

    const Eigen::Isometry3d base_from_lidar = tf2::transformToEigen(
        buffer.lookupTransform(base_frame_, lidar_frame, tf2::TimePointZero));
    preprocessor_config_.base_from_lidar = base_from_lidar.cast<float>();

    const Eigen::Isometry3d base_from_cam = tf2::transformToEigen(
        buffer.lookupTransform(base_frame_, camera_frame, tf2::TimePointZero));
    base_from_camera_ =
        gtsam::Pose3(gtsam::Rot3(base_from_cam.rotation()), gtsam::Point3(base_from_cam.translation()));

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
    wheel_buffer_.push(stamp, gtsam_conversions::toGtsamPose(msg->pose.pose));
}

void FusionNode::markerCallback(const whycode_vision::msg::WhyCodePoseArray::SharedPtr msg)
{
    const double stamp = static_cast<double>(msg->header.stamp.sec) +
                         1e-9 * msg->header.stamp.nanosec + camera_time_offset_;
    logSensorLatency("camera", stamp);
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
    pose_graph_->optimize();

    Cloud::Ptr edge_copy(new Cloud(*features.edge));
    Cloud::Ptr planar_copy(new Cloud(*features.planar));
    keyframe_map_->add(origin, edge_copy, planar_copy, features.stamp);

    last_scan_pose_  = origin;
    last_scan_stamp_ = features.stamp;
}

gtsam::Pose3 FusionNode::predictPose(double scan_stamp) const
{
    // Constant-velocity translation prediction with the rotation replaced by
    // integrated gyro rates when available.
    gtsam::Pose3 increment = last_increment_;
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
            RCLCPP_WARN(get_logger(), "No IMU data after %.1f s - running without gyro",
                        imu_init_max_wait_s_);
            imu_init_done_ = true;
        }
        else
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting for IMU initialization before processing scans");
            return;
        }
    }

    if (!preprocessor_->process(*msg, scan_image_))
        return;

    logSensorLatency("lidar", scan_image_.stamp);

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
                    scan_image_.stamp + t_min, scan_image_.stamp + t_max, deskew_time_bins_),
                t_min,
                t_max);
        }
    }

    feature_extractor_->extract(scan_image_, features_);

    if (keyframe_map_->empty())
    {
        bootstrapFirstKeyframe(features_);
        publishOdometry(last_scan_pose_, features_.stamp);
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
    if (scan_matcher_->align(features_, matcher_pose))
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

    last_increment_  = last_scan_pose_.between(scan_pose);
    last_scan_pose_  = scan_pose;
    last_scan_stamp_ = features_.stamp;

    if (keyframe_map_->shouldAddKeyframe(scan_pose))
    {
        const double       previous_stamp = keyframe_map_->back().stamp;
        const gtsam::Pose3 relative       = keyframe_map_->back().pose.between(scan_pose);
        pose_graph_->addKeyframe(scan_pose, relative, lidar_between_sigmas_);

        if (use_wheel_odom_)
        {
            const auto wheel_relative = wheel_buffer_.relativePose(previous_stamp, features_.stamp);
            if (wheel_relative)
                pose_graph_->addOdometryFactor(*wheel_relative, wheel_between_sigmas_);
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

        int anchors = 0;
        if (use_markers_)
        {
            for (const MarkerObservation& obs :
                 marker_gate_->collectNear(features_.stamp, marker_stamp_window_))
            {
                pose_graph_->addMarkerAnchor(
                    obs.position_in_camera,
                    known_markers_.at(obs.marker_id),
                    base_from_camera_,
                    marker_sigma_m_);
                anchor_event_times_[obs.marker_id] = features_.stamp;
                ++anchors;
            }
        }

        const bool corrected = pose_graph_->optimize();

        const gtsam::Pose3 optimized = pose_graph_->latestPose();
        Cloud::Ptr         edge_copy(new Cloud(*features_.edge));
        Cloud::Ptr         planar_copy(new Cloud(*features_.planar));
        keyframe_map_->add(optimized, edge_copy, planar_copy, features_.stamp);

        if (corrected)
        {
            // A marker anchor bent the past trajectory: refresh every stored
            // keyframe pose and drop the transformed-cloud cache.
            const auto poses = pose_graph_->allPoses();
            for (size_t i = 0; i < poses.size(); ++i)
                keyframe_map_->updatePose(i, poses[i]);
            keyframe_map_->invalidateCache();
            RCLCPP_INFO(
                get_logger(),
                "Marker anchor applied (%d observation%s) - trajectory corrected",
                anchors,
                anchors == 1 ? "" : "s");
        }

        last_scan_pose_ = optimized;
        publishKeyframeDebug(corrected, features_.stamp);
    }

    publishOdometry(last_scan_pose_, features_.stamp);
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
        geometry_msgs::msg::PoseArray array;
        array.header.frame_id = map_frame_;
        array.header.stamp    = ros_stamp;
        for (const gtsam::Pose3& pose : pose_graph_->allPoses())
            array.poses.push_back(gtsam_conversions::toRosPose(pose));
        debug_keyframes_pub_->publish(array);
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
    debug_fiducials_pub_->publish(array);
}

void FusionNode::publishOdometry(const gtsam::Pose3& pose, double stamp)
{
    if (!odom_pub_->is_activated())
        return;

    odom_msg_.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9));
    odom_msg_.pose.pose    = gtsam_conversions::toRosPose(pose);
    odom_pub_->publish(odom_msg_);

    if (!publish_map_tf_)
        return;

    // REP-105 split: broadcast only the map->odom correction. The continuous
    // odom->base transform belongs to the wheel odometry source, so a global
    // update here can never teleport the robot in the odom frame.
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

}  // namespace olive
