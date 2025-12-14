/**
 * @file feature_registration.cpp
 * @brief Implementation of feature-based registration
 *
 * Uses LOAM-style point-to-edge and point-to-plane residuals with
 * Gauss-Newton optimization.
 */

#include "olive/lidar_odom/feature_registration.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <cmath>

namespace olive
{

FeatureRegistration::FeatureRegistration(const FeatureRegistrationConfig& config)
    : config_(config)
    , edge_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>())
    , planar_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>())
{}

void FeatureRegistration::setTargetEdges(const PointCloudConstPtr& edges)
{
    target_edges_ = edges;
    if (edges && !edges->empty())
    {
        edge_kdtree_->setInputCloud(edges);
    }
}

void FeatureRegistration::setTargetPlanars(const PointCloudConstPtr& planars)
{
    target_planars_ = planars;
    if (planars && !planars->empty())
    {
        planar_kdtree_->setInputCloud(planars);
    }
}

FeatureRegistrationResult FeatureRegistration::align(
    const PointCloudConstPtr& source_edges,
    const PointCloudConstPtr& source_planars,
    const Eigen::Matrix4f& initial_guess)
{
    FeatureRegistrationResult result;
    result.transformation = initial_guess;

    // Check if we have targets
    bool has_edge_target = target_edges_ && !target_edges_->empty();
    bool has_planar_target = target_planars_ && !target_planars_->empty();

    if (!has_edge_target && !has_planar_target)
    {
        result.converged = false;
        return result;
    }

    // Check if we have sources
    bool has_edge_source = source_edges && !source_edges->empty();
    bool has_planar_source = source_planars && !source_planars->empty();

    if (!has_edge_source && !has_planar_source)
    {
        result.converged = false;
        return result;
    }

    Eigen::Matrix4f current_transform = initial_guess;
    double prev_cost = std::numeric_limits<double>::max();

    for (int iter = 0; iter < config_.max_iterations; ++iter)
    {
        Eigen::Matrix<double, 6, 1> delta;
        double edge_residual = 0.0;
        double planar_residual = 0.0;

        int num_correspondences = optimizationStep(
            current_transform,
            has_edge_source ? source_edges : nullptr,
            has_planar_source ? source_planars : nullptr,
            delta,
            edge_residual,
            planar_residual);

        if (num_correspondences < config_.min_edge_correspondences +
            config_.min_planar_correspondences)
        {
            // Not enough correspondences
            break;
        }

        // Update transformation
        current_transform = updateTransform(current_transform, delta);

        // Check convergence
        double delta_norm = delta.norm();
        double current_cost = config_.edge_weight * edge_residual +
                             config_.planar_weight * planar_residual;

        result.iterations = iter + 1;
        result.edge_fitness = edge_residual;
        result.planar_fitness = planar_residual;
        result.overall_fitness = current_cost;

        if (delta_norm < config_.convergence_threshold)
        {
            result.converged = true;
            break;
        }

        // Check for divergence
        if (current_cost > prev_cost * 1.5 && iter > 2)
        {
            // Diverging, stop
            break;
        }

        prev_cost = current_cost;
    }

    result.transformation = current_transform;

    return result;
}

int FeatureRegistration::optimizationStep(
    const Eigen::Matrix4f& transform,
    const PointCloudConstPtr& source_edges,
    const PointCloudConstPtr& source_planars,
    Eigen::Matrix<double, 6, 1>& delta,
    double& edge_residual,
    double& planar_residual)
{
    // Accumulate Hessian and gradient
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();

    int edge_count = 0;
    int planar_count = 0;
    double edge_cost = 0.0;
    double planar_cost = 0.0;

    // Process edge features
    if (source_edges && target_edges_ && !target_edges_->empty())
    {
        for (const auto& pt : *source_edges)
        {
            Eigen::Vector2d residual;
            Eigen::Matrix<double, 2, 6> jacobian;

            if (computeEdgeResidual(pt, transform, residual, jacobian))
            {
                // Gauss-Newton update: H += J^T * J, b += J^T * r
                H += config_.edge_weight * jacobian.transpose() * jacobian;
                b += config_.edge_weight * jacobian.transpose() * residual;
                edge_cost += residual.squaredNorm();
                edge_count++;
            }
        }
    }

    // Process planar features
    if (source_planars && target_planars_ && !target_planars_->empty())
    {
        for (const auto& pt : *source_planars)
        {
            double residual;
            Eigen::Matrix<double, 1, 6> jacobian;

            if (computePlanarResidual(pt, transform, residual, jacobian))
            {
                // Gauss-Newton update
                H += config_.planar_weight * jacobian.transpose() * jacobian;
                b += config_.planar_weight * jacobian.transpose() * residual;
                planar_cost += residual * residual;
                planar_count++;
            }
        }
    }

    // Compute residuals
    edge_residual = (edge_count > 0) ? std::sqrt(edge_cost / edge_count) : 0.0;
    planar_residual = (planar_count > 0) ? std::sqrt(planar_cost / planar_count) : 0.0;

    // Solve for delta with Levenberg-Marquardt damping
    for (int i = 0; i < 6; ++i)
    {
        H(i, i) += config_.lambda;
    }

    // Solve H * delta = -b
    Eigen::Matrix<double, 6, 1> neg_b = -b;
    delta = H.ldlt().solve(neg_b);

    return edge_count + planar_count;
}

bool FeatureRegistration::computeEdgeResidual(
    const pcl::PointXYZ& point,
    const Eigen::Matrix4f& transform,
    Eigen::Vector2d& residual,
    Eigen::Matrix<double, 2, 6>& jacobian)
{
    // Transform point to target frame
    pcl::PointXYZ pt_transformed = transformPoint(point, transform);
    Eigen::Vector3d pt_t(pt_transformed.x, pt_transformed.y, pt_transformed.z);

    // Find nearest neighbors in target edge cloud
    std::vector<int> indices(config_.edge_neighbors);
    std::vector<float> distances(config_.edge_neighbors);

    int found = edge_kdtree_->nearestKSearch(
        pt_transformed, config_.edge_neighbors, indices, distances);

    if (found < 2 || distances[0] > config_.edge_search_radius * config_.edge_search_radius)
    {
        return false;
    }

    // Fit line to neighbors using PCA
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (int i = 0; i < found; ++i)
    {
        const auto& p = (*target_edges_)[indices[i]];
        centroid += Eigen::Vector3d(p.x, p.y, p.z);
    }
    centroid /= found;

    // Compute covariance
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (int i = 0; i < found; ++i)
    {
        const auto& p = (*target_edges_)[indices[i]];
        Eigen::Vector3d diff = Eigen::Vector3d(p.x, p.y, p.z) - centroid;
        cov += diff * diff.transpose();
    }
    cov /= found;

    // Eigenvalue decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d eigenvalues = solver.eigenvalues();
    Eigen::Matrix3d eigenvectors = solver.eigenvectors();

    // Check if points form a line (one large eigenvalue)
    if (eigenvalues(2) < config_.min_eigenvalue_ratio * eigenvalues(1))
    {
        return false;  // Not a good line
    }

    // Line direction is eigenvector with largest eigenvalue
    Eigen::Vector3d line_dir = eigenvectors.col(2).normalized();

    // Point-to-line distance
    // d = ||(pt - centroid) x line_dir||
    Eigen::Vector3d pt_to_centroid = pt_t - centroid;
    Eigen::Vector3d cross = pt_to_centroid.cross(line_dir);

    // Residual is the cross product (represents perpendicular distance)
    residual = cross.head<2>();  // Take x, y components

    // Check residual magnitude
    if (cross.norm() > config_.max_edge_residual)
    {
        return false;  // Outlier
    }

    // Compute Jacobian
    // The residual is f(T) = (R*p + t - centroid) x line_dir
    // Jacobian w.r.t. [rx, ry, rz, tx, ty, tz]

    Eigen::Vector3d pt_s(point.x, point.y, point.z);
    Eigen::Matrix3d R = transform.block<3, 3>(0, 0).cast<double>();
    Eigen::Vector3d Rp = R * pt_s;

    // Skew-symmetric matrix for cross product
    auto skew = [](const Eigen::Vector3d& v) -> Eigen::Matrix3d {
        Eigen::Matrix3d S;
        S << 0, -v.z(), v.y(),
             v.z(), 0, -v.x(),
             -v.y(), v.x(), 0;
        return S;
    };

    // d(cross)/d(rotation) = -line_dir x (R * [p]_x)
    // d(cross)/d(translation) = -[line_dir]_x
    Eigen::Matrix3d skew_line = skew(line_dir);
    Eigen::Matrix3d skew_Rp = skew(Rp);

    Eigen::Matrix<double, 3, 6> J_full;
    J_full.block<3, 3>(0, 0) = -skew_line * skew_Rp;  // rotation part
    J_full.block<3, 3>(0, 3) = -skew_line;           // translation part

    // Take only first 2 rows (x, y residuals)
    jacobian = J_full.topRows<2>();

    return true;
}

bool FeatureRegistration::computePlanarResidual(
    const pcl::PointXYZ& point,
    const Eigen::Matrix4f& transform,
    double& residual,
    Eigen::Matrix<double, 1, 6>& jacobian)
{
    // Transform point to target frame
    pcl::PointXYZ pt_transformed = transformPoint(point, transform);
    Eigen::Vector3d pt_t(pt_transformed.x, pt_transformed.y, pt_transformed.z);

    // Find nearest neighbors in target planar cloud
    std::vector<int> indices(config_.planar_neighbors);
    std::vector<float> distances(config_.planar_neighbors);

    int found = planar_kdtree_->nearestKSearch(
        pt_transformed, config_.planar_neighbors, indices, distances);

    if (found < 3 || distances[0] > config_.planar_search_radius * config_.planar_search_radius)
    {
        return false;
    }

    // Fit plane to neighbors using PCA
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (int i = 0; i < found; ++i)
    {
        const auto& p = (*target_planars_)[indices[i]];
        centroid += Eigen::Vector3d(p.x, p.y, p.z);
    }
    centroid /= found;

    // Compute covariance
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (int i = 0; i < found; ++i)
    {
        const auto& p = (*target_planars_)[indices[i]];
        Eigen::Vector3d diff = Eigen::Vector3d(p.x, p.y, p.z) - centroid;
        cov += diff * diff.transpose();
    }
    cov /= found;

    // Eigenvalue decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d eigenvalues = solver.eigenvalues();
    Eigen::Matrix3d eigenvectors = solver.eigenvectors();

    // Check if points form a plane (two large eigenvalues, one small)
    if (eigenvalues(1) < config_.min_eigenvalue_ratio * eigenvalues(0))
    {
        return false;  // Not a good plane
    }

    // Plane normal is eigenvector with smallest eigenvalue
    Eigen::Vector3d normal = eigenvectors.col(0).normalized();

    // Point-to-plane distance
    // d = (pt - centroid) · normal
    double dist = (pt_t - centroid).dot(normal);
    residual = dist;

    // Check residual magnitude
    if (std::abs(dist) > config_.max_planar_residual)
    {
        return false;  // Outlier
    }

    // Compute Jacobian
    // The residual is f(T) = (R*p + t - centroid) · normal
    // Jacobian w.r.t. [rx, ry, rz, tx, ty, tz]

    Eigen::Vector3d pt_s(point.x, point.y, point.z);
    Eigen::Matrix3d R = transform.block<3, 3>(0, 0).cast<double>();
    Eigen::Vector3d Rp = R * pt_s;

    // d(dist)/d(rotation) = normal^T * [-Rp]_x = -normal^T * [Rp]_x
    // d(dist)/d(translation) = normal^T
    auto skew = [](const Eigen::Vector3d& v) -> Eigen::Matrix3d {
        Eigen::Matrix3d S;
        S << 0, -v.z(), v.y(),
             v.z(), 0, -v.x(),
             -v.y(), v.x(), 0;
        return S;
    };

    Eigen::Matrix3d skew_Rp = skew(Rp);

    jacobian.block<1, 3>(0, 0) = -normal.transpose() * skew_Rp;
    jacobian.block<1, 3>(0, 3) = normal.transpose();

    return true;
}

pcl::PointXYZ FeatureRegistration::transformPoint(
    const pcl::PointXYZ& point,
    const Eigen::Matrix4f& transform) const
{
    Eigen::Vector4f pt(point.x, point.y, point.z, 1.0f);
    Eigen::Vector4f pt_transformed = transform * pt;

    pcl::PointXYZ result;
    result.x = pt_transformed.x();
    result.y = pt_transformed.y();
    result.z = pt_transformed.z();

    return result;
}

Eigen::Matrix4f FeatureRegistration::updateTransform(
    const Eigen::Matrix4f& transform,
    const Eigen::Matrix<double, 6, 1>& delta) const
{
    // Delta is [rx, ry, rz, tx, ty, tz]
    Eigen::Vector3d rotation_delta = delta.head<3>();
    Eigen::Vector3d translation_delta = delta.tail<3>();

    // For 2D mode (ground robot): constrain to x, y, yaw only
    if (config_.is_2d_mode)
    {
        // Zero out roll (rx), pitch (ry), and z translation
        rotation_delta.x() = 0.0;  // No roll
        rotation_delta.y() = 0.0;  // No pitch
        translation_delta.z() = 0.0;  // No z movement
    }

    // Convert rotation delta to rotation matrix using Rodrigues' formula
    double angle = rotation_delta.norm();
    Eigen::Matrix3d dR;

    if (angle < 1e-10)
    {
        dR = Eigen::Matrix3d::Identity();
    }
    else
    {
        Eigen::Vector3d axis = rotation_delta / angle;
        dR = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    }

    // Apply delta: T_new = dT * T_old
    // where dT is the incremental transformation
    Eigen::Matrix4f delta_transform = Eigen::Matrix4f::Identity();
    delta_transform.block<3, 3>(0, 0) = dR.cast<float>();
    delta_transform.block<3, 1>(0, 3) = translation_delta.cast<float>();

    return delta_transform * transform;
}

bool FeatureRegistration::checkDegeneracy(
    const Eigen::Matrix<double, 6, 6>& H,
    int edge_count,
    int planar_count) const
{
    // Check minimum correspondences
    if (edge_count < config_.min_edge_correspondences &&
        planar_count < config_.min_planar_correspondences)
    {
        return true;
    }

    // Check Hessian condition number
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(H);
    Eigen::Matrix<double, 6, 1> eigenvalues = solver.eigenvalues();

    double min_eigen = eigenvalues.minCoeff();
    double max_eigen = eigenvalues.maxCoeff();

    if (min_eigen < 1e-6 || max_eigen / min_eigen > 1e6)
    {
        return true;  // Poorly conditioned
    }

    return false;
}

}  // namespace olive
