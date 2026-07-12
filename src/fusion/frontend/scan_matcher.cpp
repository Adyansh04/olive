#include "olive/fusion/frontend/scan_matcher.hpp"

#include <pcl/common/transforms.h>

#include <cmath>
#include <numbers>
#include <vector>

namespace olive
{

Eigen::Affine3f MatcherPose::affine() const
{
    return pcl::getTransformation(x, y, z, roll, pitch, yaw);
}

MatcherPose MatcherPose::fromAffine(const Eigen::Affine3f& t)
{
    MatcherPose pose;
    pcl::getTranslationAndEulerAngles(t, pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
    return pose;
}

ScanMatcher::ScanMatcher(const MatcherConfig& config)
  : config_(config)
{}

void ScanMatcher::setTarget(const Cloud::Ptr& edge_map, const Cloud::Ptr& planar_map)
{
    edge_map_   = edge_map;
    planar_map_ = planar_map;
    if (!edge_map_->empty())
        edge_tree_.setInputCloud(edge_map_);
    if (!planar_map_->empty())
        planar_tree_.setInputCloud(planar_map_);
}

bool ScanMatcher::align(const FeatureClouds& features, MatcherPose& pose)
{
    if (!edge_map_ || !planar_map_ || planar_map_->empty())
        return false;

    const bool use_edges =
        static_cast<int>(features.edge->size()) >= config_.min_edge_features && !edge_map_->empty();
    if (static_cast<int>(features.planar->size()) < config_.min_planar_features)
        return false;

    is_degenerate_ = false;

    for (int iteration = 0; iteration < config_.max_iterations; ++iteration)
    {
        residual_points_.clear();
        residual_coeffs_.clear();

        const Eigen::Affine3f transform = pose.affine();
        if (use_edges)
            addEdgeResiduals(*features.edge, transform);
        addPlanarResiduals(*features.planar, transform);

        if (static_cast<int>(residual_points_.size()) < config_.min_correspondences)
            return false;

        if (gaussNewtonStep(pose, iteration))
            break;  // converged
    }
    return true;
}

void ScanMatcher::addEdgeResiduals(const Cloud& edge_scan, const Eigen::Affine3f& transform)
{
    std::vector<int>   indices(5);
    std::vector<float> sq_distances(5);

    for (const CloudPoint& point : edge_scan.points)
    {
        CloudPoint query;
        query.getVector3fMap() = transform * point.getVector3fMap();

        if (edge_tree_.nearestKSearch(query, 5, indices, sq_distances) < 5)
            continue;
        if (sq_distances[4] >= config_.max_correspondence_sq_dist)
            continue;

        // Fit a line: mean + principal eigenvector of the neighbor scatter.
        Eigen::Vector3f mean = Eigen::Vector3f::Zero();
        for (int j = 0; j < 5; ++j)
            mean += edge_map_->points[indices[j]].getVector3fMap();
        mean /= 5.0F;

        Eigen::Matrix3f scatter = Eigen::Matrix3f::Zero();
        for (int j = 0; j < 5; ++j)
        {
            const Eigen::Vector3f d = edge_map_->points[indices[j]].getVector3fMap() - mean;
            scatter += d * d.transpose();
        }
        scatter /= 5.0F;

        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(scatter);
        // Eigenvalues are ascending: [2] largest. Only a clearly linear
        // neighborhood defines a stable line.
        if (eig.eigenvalues()[2] <= 3.0F * eig.eigenvalues()[1])
            continue;

        const Eigen::Vector3f direction = eig.eigenvectors().col(2);
        const Eigen::Vector3f p0        = query.getVector3fMap();
        const Eigen::Vector3f p1        = mean + 0.1F * direction;
        const Eigen::Vector3f p2        = mean - 0.1F * direction;

        // Point-to-line distance and its gradient w.r.t. the point.
        const Eigen::Vector3f cross = (p0 - p1).cross(p0 - p2);
        const float           area  = cross.norm();
        const float           base  = (p1 - p2).norm();
        if (area < 1e-9F || base < 1e-9F)
            continue;

        const float           distance = area / base;
        const Eigen::Vector3f gradient = (p1 - p2).cross(cross).normalized();

        const float weight = 1.0F - 0.9F * std::fabs(distance);
        if (weight <= 0.1F)
            continue;

        CloudPoint coeff;
        coeff.x         = weight * gradient.x();
        coeff.y         = weight * gradient.y();
        coeff.z         = weight * gradient.z();
        coeff.intensity = weight * distance;
        residual_points_.push_back(point);
        residual_coeffs_.push_back(coeff);
    }
}

void ScanMatcher::addPlanarResiduals(const Cloud& planar_scan, const Eigen::Affine3f& transform)
{
    std::vector<int>   indices(5);
    std::vector<float> sq_distances(5);

    for (const CloudPoint& point : planar_scan.points)
    {
        CloudPoint query;
        query.getVector3fMap() = transform * point.getVector3fMap();

        if (planar_tree_.nearestKSearch(query, 5, indices, sq_distances) < 5)
            continue;
        if (sq_distances[4] >= config_.max_correspondence_sq_dist)
            continue;

        // Fit a plane through the five neighbors from their scatter: normal =
        // eigenvector of the smallest eigenvalue. The mid eigenvalue guards
        // against collinear neighbor sets (e.g. five points along one ring
        // arc on the ground), whose "plane" is arbitrary — a residual check
        // alone cannot reject those.
        Eigen::Matrix<float, 5, 3> neighbors;
        Eigen::Vector3f            mean = Eigen::Vector3f::Zero();
        for (int j = 0; j < 5; ++j)
        {
            neighbors.row(j) = planar_map_->points[indices[j]].getVector3fMap().transpose();
            mean += neighbors.row(j).transpose();
        }
        mean /= 5.0F;

        Eigen::Matrix3f scatter = Eigen::Matrix3f::Zero();
        for (int j = 0; j < 5; ++j)
        {
            const Eigen::Vector3f d = neighbors.row(j).transpose() - mean;
            scatter += d * d.transpose();
        }

        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(scatter);
        if (eig.eigenvalues()[1] < 0.05F * eig.eigenvalues()[2])
            continue;  // collinear

        const Eigen::Vector3f normal = eig.eigenvectors().col(0);
        const float           offset = -normal.dot(mean);

        // Reject fits the neighbors themselves do not honour.
        bool plane_valid = true;
        for (int j = 0; j < 5; ++j)
        {
            if (std::fabs(normal.dot(neighbors.row(j).transpose()) + offset) > 0.2F)
            {
                plane_valid = false;
                break;
            }
        }
        if (!plane_valid)
            continue;

        const float distance = normal.dot(query.getVector3fMap()) + offset;
        // The range-scaled weight trusts distant planar points less.
        const float weight =
            1.0F - 0.9F * std::fabs(distance) / std::sqrt(std::sqrt(point.getVector3fMap().norm()));
        if (weight <= 0.1F)
            continue;

        CloudPoint coeff;
        coeff.x         = weight * normal.x();
        coeff.y         = weight * normal.y();
        coeff.z         = weight * normal.z();
        coeff.intensity = weight * distance;
        residual_points_.push_back(point);
        residual_coeffs_.push_back(coeff);
    }
}

bool ScanMatcher::gaussNewtonStep(MatcherPose& pose, int iteration)
{
    // For the residual e = w . (R p + t) + d with R = Rz(yaw) Ry(pitch) Rx(roll):
    //   de/d(angle) = w . (dR/d(angle) p),  de/dt = w
    // The three dR/d(angle) matrices depend only on the pose, so they are
    // computed once per iteration.
    const Eigen::AngleAxisf roll_rot(pose.roll, Eigen::Vector3f::UnitX());
    const Eigen::AngleAxisf pitch_rot(pose.pitch, Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf yaw_rot(pose.yaw, Eigen::Vector3f::UnitZ());
    const Eigen::Matrix3f   rx = roll_rot.toRotationMatrix();
    const Eigen::Matrix3f   ry = pitch_rot.toRotationMatrix();
    const Eigen::Matrix3f   rz = yaw_rot.toRotationMatrix();

    const float sr = std::sin(pose.roll), cr = std::cos(pose.roll);
    const float sp = std::sin(pose.pitch), cp = std::cos(pose.pitch);
    const float sy = std::sin(pose.yaw), cy = std::cos(pose.yaw);

    Eigen::Matrix3f drx, dry, drz;  // element-wise derivatives of Rx, Ry, Rz
    drx << 0, 0, 0, 0, -sr, -cr, 0, cr, -sr;
    dry << -sp, 0, cp, 0, 0, 0, -cp, 0, -sp;
    drz << -sy, -cy, 0, cy, -sy, 0, 0, 0, 0;

    const Eigen::Matrix3f d_roll  = rz * ry * drx;
    const Eigen::Matrix3f d_pitch = rz * dry * rx;
    const Eigen::Matrix3f d_yaw   = drz * ry * rx;

    const int n = static_cast<int>(residual_points_.size());

    Eigen::Matrix<float, Eigen::Dynamic, 6> jacobian(n, 6);
    Eigen::VectorXf                         residual(n);

    for (int i = 0; i < n; ++i)
    {
        const Eigen::Vector3f p = residual_points_.points[i].getVector3fMap();
        const Eigen::Vector3f w = residual_coeffs_.points[i].getVector3fMap();

        jacobian(i, 0) = w.dot(d_roll * p);
        jacobian(i, 1) = w.dot(d_pitch * p);
        jacobian(i, 2) = w.dot(d_yaw * p);
        jacobian(i, 3) = w.x();
        jacobian(i, 4) = w.y();
        jacobian(i, 5) = w.z();
        residual(i)    = -residual_coeffs_.points[i].intensity;
    }

    const Eigen::Matrix<float, 6, 6> jtj = jacobian.transpose() * jacobian;
    const Eigen::Matrix<float, 6, 1> jtr = jacobian.transpose() * residual;

    Eigen::Matrix<float, 6, 1> update = jtj.colPivHouseholderQr().solve(jtr);

    if (iteration == 0)
    {
        // Eigen-check the normal matrix once per alignment: directions with
        // insufficient constraint are frozen for all iterations.
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>> eig(jtj);
        eigenvalues_ = eig.eigenvalues();

        Eigen::Matrix<float, 6, 6> eigenvectors = eig.eigenvectors().transpose();
        Eigen::Matrix<float, 6, 6> constrained  = eigenvectors;
        is_degenerate_                          = false;
        for (int i = 0; i < 6; ++i)
        {
            if (eig.eigenvalues()[i] < config_.degeneracy_eigen_threshold)
            {
                constrained.row(i).setZero();
                is_degenerate_ = true;
            }
            else
            {
                break;  // eigenvalues ascend; the rest are constrained
            }
        }
        degeneracy_projector_ = eigenvectors.inverse() * constrained;
    }

    if (is_degenerate_)
        update = degeneracy_projector_ * update;

    pose.roll += update(0);
    pose.pitch += update(1);
    pose.yaw += update(2);
    pose.x += update(3);
    pose.y += update(4);
    pose.z += update(5);

    const float delta_rotation = std::sqrt(
        std::pow(update(0) * 180.0F / static_cast<float>(std::numbers::pi), 2) +
        std::pow(update(1) * 180.0F / static_cast<float>(std::numbers::pi), 2) +
        std::pow(update(2) * 180.0F / static_cast<float>(std::numbers::pi), 2));
    const float delta_translation = std::sqrt(
        std::pow(update(3) * 100.0F, 2) + std::pow(update(4) * 100.0F, 2) +
        std::pow(update(5) * 100.0F, 2));

    return delta_rotation < 0.05F && delta_translation < 0.05F;
}

}  // namespace olive
