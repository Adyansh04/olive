// FusionNode — param-gated RViz debug visualization (keyframe path/map, scan
// features, fiducial markers). Off the critical output path.
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "olive/common/gtsam_conversions.hpp"
#include "olive/fusion/fusion_node.hpp"
#include "olive/fusion/graph/pose_graph.hpp"
#include "olive/fusion/inputs/marker_gate.hpp"

namespace olive
{

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

}  // namespace olive
