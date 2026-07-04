#include "olive/fusion/fusion_node.hpp"

#include <pcl/common/transforms.h>

#include "olive/common/covariance_utils.hpp"
#include "olive/common/gtsam_conversions.hpp"

namespace olive
{

FusionNode::FusionNode(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("fusion_node", options)
{
    declareParameters();

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
    declare_parameter(
        "wheel_between_sigmas", std::vector<double>{ 0.03, 0.03, 0.5, 0.5, 0.5, 0.2 });
    declare_parameter("planar_prior_sigmas", std::vector<double>{ 0.02, 0.009, 0.009 });

    declare_parameter("lidar_translation", std::vector<double>{ 0.0, 0.0, 0.145 });
    declare_parameter("lidar_rpy", std::vector<double>{ 0.0, 0.0, 0.0 });

    declare_parameter("min_range", 0.3);
    declare_parameter("max_range", 12.0);

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

    const auto translation               = get_parameter("lidar_translation").as_double_array();
    const auto rpy                       = get_parameter("lidar_rpy").as_double_array();
    preprocessor_config_.base_from_lidar = pcl::getTransformation(
        static_cast<float>(translation[0]),
        static_cast<float>(translation[1]),
        static_cast<float>(translation[2]),
        static_cast<float>(rpy[0]),
        static_cast<float>(rpy[1]),
        static_cast<float>(rpy[2]));
    preprocessor_config_.min_range = static_cast<float>(get_parameter("min_range").as_double());
    preprocessor_config_.max_range = static_cast<float>(get_parameter("max_range").as_double());

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

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

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

    RCLCPP_INFO(get_logger(), "Activated");
    return CallbackReturn::SUCCESS;
}

FusionNode::CallbackReturn FusionNode::on_deactivate(const rclcpp_lifecycle::State& state)
{
    points_sub_.reset();
    imu_sub_.reset();
    wheel_sub_.reset();
    LifecycleNode::on_deactivate(state);
    return CallbackReturn::SUCCESS;
}

FusionNode::CallbackReturn FusionNode::on_cleanup(const rclcpp_lifecycle::State&)
{
    points_sub_.reset();
    imu_sub_.reset();
    wheel_sub_.reset();
    odom_pub_.reset();
    tf_broadcaster_.reset();
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
        static_cast<double>(msg->header.stamp.sec) + 1e-9 * msg->header.stamp.nanosec;
    sample.angular_velocity =
        Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    sample.linear_acceleration = Eigen::Vector3d(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z);
    imu_buffer_.push(sample);
}

void FusionNode::wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    const double stamp =
        static_cast<double>(msg->header.stamp.sec) + 1e-9 * msg->header.stamp.nanosec;
    wheel_buffer_.push(stamp, gtsam_conversions::toGtsamPose(msg->pose.pose));
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

    if (!preprocessor_->process(*msg, scan_image_))
        return;

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
            const auto wheel_relative =
                wheel_buffer_.relativePose(previous_stamp, features_.stamp);
            if (wheel_relative)
                pose_graph_->addOdometryFactor(*wheel_relative, wheel_between_sigmas_);
        }
        if (use_planar_prior_)
            pose_graph_->addPlanarPrior(planar_prior_sigmas_);

        pose_graph_->optimize();

        const gtsam::Pose3 optimized = pose_graph_->latestPose();
        Cloud::Ptr         edge_copy(new Cloud(*features_.edge));
        Cloud::Ptr         planar_copy(new Cloud(*features_.planar));
        keyframe_map_->add(optimized, edge_copy, planar_copy, features_.stamp);

        last_scan_pose_ = optimized;
    }

    publishOdometry(last_scan_pose_, features_.stamp);

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
