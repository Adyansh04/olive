/**
 * @file feature_registration.cpp
 * @brief Implementation of feature-based registration
 *
 * Uses point-to-edge and point-to-plane residuals with
 * Gauss-Newton optimization and Levenberg-Marquardt damping.
 *
 * For ground robots: Constrained to (x, y, yaw) only.
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
  , H_buffer_(Eigen::Matrix<double, 6, 6>::Zero())
  , b_buffer_(Eigen::Matrix<double, 6, 1>::Zero())
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

FeatureRegistrationResult FeatureRegistration::alignToLocalMap(
    const PointCloudConstPtr& source_edges,
    const PointCloudConstPtr& source_planars,
    const Eigen::Matrix4f&    initial_guess)
{
    FeatureRegistrationResult result;
    result.transformation = initial_guess;

    // Check targets
    const bool has_edge_target   = target_edges_ && !target_edges_->empty();
    const bool has_planar_target = target_planars_ && !target_planars_->empty();

    if (!has_edge_target && !has_planar_target)
    {
        result.degenerate = true;
        return result;
    }

    // Check sources
    const bool has_edge_source   = source_edges && !source_edges->empty();
    const bool has_planar_source = source_planars && !source_planars->empty();

    if (!has_edge_source && !has_planar_source)
    {
        result.degenerate = true;
        return result;
    }

    Eigen::Matrix4f current_transform = initial_guess;
    double          prev_cost         = std::numeric_limits<double>::max();

    for (int iter = 0; iter < config_.max_iterations; ++iter)
    {
        Eigen::Matrix<double, 6, 1> delta;
        double                      edge_residual   = 0.0;
        double                      planar_residual = 0.0;

        int correspondences = optimizationStep(
            current_transform,
            source_edges,
            source_planars,
            delta,
            edge_residual,
            planar_residual);

        if (correspondences < config_.min_edge_correspondences + config_.min_planar_correspondences)
        {
            result.degenerate = true;
            break;
        }

        // Update transform
        current_transform = updateTransform(current_transform, delta);

        // Check convergence
        double delta_norm   = delta.norm();
        double current_cost = edge_residual + planar_residual;

        result.edge_rmse  = edge_residual;
        result.plane_rmse = planar_residual;
        result.iterations = iter + 1;

        if (delta_norm < config_.convergence_threshold)
        {
            result.converged = true;
            break;
        }

        if (std::abs(current_cost - prev_cost) < config_.convergence_threshold * 0.1)
        {
            result.converged = true;
            break;
        }

        prev_cost = current_cost;
    }

    result.transformation  = current_transform;
    result.overall_fitness = result.edge_rmse + result.plane_rmse;

    return result;
}

int FeatureRegistration::optimizationStep(
    const Eigen::Matrix4f&       transform,
    const PointCloudConstPtr&    source_edges,
    const PointCloudConstPtr&    source_planars,
    Eigen::Matrix<double, 6, 1>& delta,
    double&                      edge_residual,
    double&                      planar_residual)
{
    // PERF: Reuse preallocated buffers
    H_buffer_.setZero();
    b_buffer_.setZero();

    int    edge_count   = 0;
    int    planar_count = 0;
    double edge_cost    = 0.0;
    double planar_cost  = 0.0;

    // Process edge features
    if (source_edges && target_edges_ && !target_edges_->empty())
    {
        for (const auto& pt : *source_edges)
        {
            Eigen::Vector2d             residual;
            Eigen::Matrix<double, 2, 6> jacobian;

            if (computeEdgeResidual(pt, transform, residual, jacobian))
            {
                // Gauss-Newton update: H += J^T * J, b += J^T * r
                H_buffer_ += config_.edge_weight * jacobian.transpose() * jacobian;
                b_buffer_ += config_.edge_weight * jacobian.transpose() * residual;
                edge_cost += residual.squaredNorm();
                ++edge_count;
            }
        }
    }

    // Process planar features
    if (source_planars && target_planars_ && !target_planars_->empty())
    {
        for (const auto& pt : *source_planars)
        {
            double                      residual;
            Eigen::Matrix<double, 1, 6> jacobian;

            if (computePlanarResidual(pt, transform, residual, jacobian))
            {
                H_buffer_ += config_.planar_weight * jacobian.transpose() * jacobian;
                b_buffer_ += config_.planar_weight * jacobian.transpose() *
                             Eigen::Matrix<double, 1, 1>(residual);
                planar_cost += residual * residual;
                ++planar_count;
            }
        }
    }

    // Compute RMS residuals
    edge_residual   = (edge_count > 0) ? std::sqrt(edge_cost / edge_count) : 0.0;
    planar_residual = (planar_count > 0) ? std::sqrt(planar_cost / planar_count) : 0.0;

    // Add Levenberg-Marquardt damping
    for (int i = 0; i < 6; ++i)
    {
        H_buffer_(i, i) += config_.lambda * H_buffer_(i, i) + 1e-6;
    }

    // Check for degeneracy
    if (isFrameDegenerate(H_buffer_, edge_count, planar_count))
    {
        delta.setZero();
        return 0;
    }

    // Solve H * delta = -b
    delta = H_buffer_.ldlt().solve(-b_buffer_);

    // Store correspondence counts
    // (These would go in result but we return them via count)

    return edge_count + planar_count;
}

bool FeatureRegistration::computeEdgeResidual(
    const pcl::PointXYZ&         point,
    const Eigen::Matrix4f&       transform,
    Eigen::Vector2d&             residual,
    Eigen::Matrix<double, 2, 6>& jacobian)
{
    // Transform point to target frame
    pcl::PointXYZ   pt_transformed = transformPoint(point, transform);
    Eigen::Vector3d pt_t(pt_transformed.x, pt_transformed.y, pt_transformed.z);

    // Find nearest neighbors in target edge map
    std::vector<int>   indices(config_.edge_neighbors);
    std::vector<float> distances(config_.edge_neighbors);

    int found =
        edge_kdtree_->nearestKSearch(pt_transformed, config_.edge_neighbors, indices, distances);

    if (found < 2)
    {
        return false;
    }

    // Check max distance
    if (distances[0] > config_.edge_search_radius * config_.edge_search_radius)
    {
        return false;
    }

    // Fit line to neighbors using PCA
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (int i = 0; i < found; ++i)
    {
        const auto& neighbor = (*target_edges_)[indices[i]];
        centroid += Eigen::Vector3d(neighbor.x, neighbor.y, neighbor.z);
    }
    centroid /= found;

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (int i = 0; i < found; ++i)
    {
        const auto&     neighbor = (*target_edges_)[indices[i]];
        Eigen::Vector3d diff(
            neighbor.x - centroid.x(),
            neighbor.y - centroid.y(),
            neighbor.z - centroid.z());
        cov += diff * diff.transpose();
    }
    cov /= found;

    // Eigenvalue decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d                                eigenvalues  = solver.eigenvalues();
    Eigen::Matrix3d                                eigenvectors = solver.eigenvectors();

    // Check if points form a line (one dominant eigenvalue)
    if (eigenvalues(2) < config_.min_eigenvalue_ratio * eigenvalues(1))
    {
        return false;
    }

    // Line direction is eigenvector with largest eigenvalue
    Eigen::Vector3d line_dir = eigenvectors.col(2).normalized();

    // Point-to-line distance: d = ||(p - c) x line_dir||
    Eigen::Vector3d pc    = pt_t - centroid;
    Eigen::Vector3d cross = pc.cross(line_dir);

    // 2D residual (perpendicular components to line)
    residual(0) = cross(0);
    residual(1) = cross(1);

    // Check max residual
    if (residual.norm() > config_.max_edge_residual)
    {
        return false;
    }

    // Compute Jacobian
    // For point-to-line, J = d(cross)/d(transform)
    Eigen::Vector3d pt_orig(point.x, point.y, point.z);
    Eigen::Matrix3d rotation_matrix = transform.block<3, 3>(0, 0).cast<double>();

    // Jacobian of cross product w.r.t. rotation and translation
    // d(residual)/d(rx, ry, rz, tx, ty, tz)
    Eigen::Matrix<double, 3, 6> j_full;

    // Translation part: d(cross)/dt = skew(line_dir)
    j_full.block<3, 3>(0, 3) = skewSymmetric(line_dir);

    // Rotation part: d(cross)/dR = -skew(line_dir) * skew(R * p)
    Eigen::Vector3d r_p      = rotation_matrix * pt_orig;
    j_full.block<3, 3>(0, 0) = -skewSymmetric(line_dir) * skewSymmetric(r_p);

    // Take first two rows for 2D residual
    jacobian = j_full.topRows<2>();

    // For 2D mode: zero out roll, pitch, z contributions
    if (config_.is_2d_mode)
    {
        jacobian.col(0).setZero();  // rx
        jacobian.col(1).setZero();  // ry
        jacobian.col(5).setZero();  // tz
    }

    return true;
}

bool FeatureRegistration::computePlanarResidual(
    const pcl::PointXYZ&         point,
    const Eigen::Matrix4f&       transform,
    double&                      residual,
    Eigen::Matrix<double, 1, 6>& jacobian)
{
    // Transform point
    pcl::PointXYZ   pt_transformed = transformPoint(point, transform);
    Eigen::Vector3d pt_t(pt_transformed.x, pt_transformed.y, pt_transformed.z);

    // Find nearest neighbors
    std::vector<int>   indices(config_.planar_neighbors);
    std::vector<float> distances(config_.planar_neighbors);

    int found =
        planar_kdtree_->nearestKSearch(pt_transformed, config_.planar_neighbors, indices, distances);

    if (found < 3)
    {
        return false;
    }

    if (distances[0] > config_.planar_search_radius * config_.planar_search_radius)
    {
        return false;
    }

    // Fit plane using PCA
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (int i = 0; i < found; ++i)
    {
        const auto& neighbor = (*target_planars_)[indices[i]];
        centroid += Eigen::Vector3d(neighbor.x, neighbor.y, neighbor.z);
    }
    centroid /= found;

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (int i = 0; i < found; ++i)
    {
        const auto&     neighbor = (*target_planars_)[indices[i]];
        Eigen::Vector3d diff(
            neighbor.x - centroid.x(),
            neighbor.y - centroid.y(),
            neighbor.z - centroid.z());
        cov += diff * diff.transpose();
    }
    cov /= found;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d                                eigenvalues  = solver.eigenvalues();
    Eigen::Matrix3d                                eigenvectors = solver.eigenvectors();

    // Check if points form a plane (one small eigenvalue)
    if (eigenvalues(0) > eigenvalues(2) / config_.min_eigenvalue_ratio)
    {
        return false;
    }

    // Plane normal is eigenvector with smallest eigenvalue
    Eigen::Vector3d normal = eigenvectors.col(0).normalized();

    // Point-to-plane distance: d = (p - c) . normal
    residual = (pt_t - centroid).dot(normal);

    if (std::abs(residual) > config_.max_planar_residual)
    {
        return false;
    }

    // Jacobian: d(residual)/d(params)
    Eigen::Vector3d pt_orig(point.x, point.y, point.z);
    Eigen::Matrix3d rotation_matrix = transform.block<3, 3>(0, 0).cast<double>();

    // Translation: d(residual)/dt = normal^T
    jacobian.block<1, 3>(0, 3) = normal.transpose();

    // Rotation: d(residual)/dR = normal^T * (-skew(R*p))
    Eigen::Vector3d r_p        = rotation_matrix * pt_orig;
    jacobian.block<1, 3>(0, 0) = -normal.transpose() * skewSymmetric(r_p);

    // 2D mode constraints
    if (config_.is_2d_mode)
    {
        jacobian(0, 0) = 0.0;  // rx
        jacobian(0, 1) = 0.0;  // ry
        jacobian(0, 5) = 0.0;  // tz
    }

    return true;
}

pcl::PointXYZ FeatureRegistration::transformPoint(
    const pcl::PointXYZ&   point,
    const Eigen::Matrix4f& transform) const
{
    Eigen::Vector4f pt(point.x, point.y, point.z, 1.0f);
    Eigen::Vector4f pt_t = transform * pt;
    return pcl::PointXYZ(pt_t.x(), pt_t.y(), pt_t.z());
}

Eigen::Matrix4f FeatureRegistration::updateTransform(
    const Eigen::Matrix4f&             transform,
    const Eigen::Matrix<double, 6, 1>& delta) const
{
    // Delta: [rx, ry, rz, tx, ty, tz]
    Eigen::Vector3d rotation_delta    = delta.head<3>();
    Eigen::Vector3d translation_delta = delta.tail<3>();

    // 2D mode: only yaw and x,y
    if (config_.is_2d_mode)
    {
        rotation_delta.x()    = 0.0;
        rotation_delta.y()    = 0.0;
        translation_delta.z() = 0.0;
    }

    // Convert rotation to matrix
    double          angle = rotation_delta.norm();
    Eigen::Matrix3d d_r;

    if (angle < 1e-10)
    {
        d_r = Eigen::Matrix3d::Identity();
    }
    else
    {
        Eigen::Vector3d axis = rotation_delta / angle;
        d_r                  = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    }

    // Apply: T_new = dT * T
    Eigen::Matrix4f delta_transform   = Eigen::Matrix4f::Identity();
    delta_transform.block<3, 3>(0, 0) = d_r.cast<float>();
    delta_transform.block<3, 1>(0, 3) = translation_delta.cast<float>();

    return delta_transform * transform;
}

bool FeatureRegistration::isFrameDegenerate(
    const Eigen::Matrix<double, 6, 6>& H,
    int                                edge_count,
    int                                planar_count) const
{
    // Check minimum correspondences
    if (edge_count < config_.min_edge_correspondences &&
        planar_count < config_.min_planar_correspondences)
    {
        return true;
    }

    // Check Hessian condition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(H);
    const Eigen::Matrix<double, 6, 1>&                         eigenvalues = solver.eigenvalues();

    double min_ev = eigenvalues.minCoeff();
    double max_ev = eigenvalues.maxCoeff();

    return min_ev < 1e-6 || max_ev / min_ev > 1e6;
}

Eigen::Matrix3d FeatureRegistration::skewSymmetric(const Eigen::Vector3d& v) const
{
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return m;
}

}  // namespace olive
