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
#include <iostream>

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
    
    // DEBUG: Log input sizes
    static int align_call_count = 0;
    if (++align_call_count % 50 == 1)  // Log every 50th call
    {
        std::cerr << "[FeatureReg DEBUG] alignToLocalMap called: "
                  << "src_edges=" << (source_edges ? source_edges->size() : 0)
                  << ", src_planars=" << (source_planars ? source_planars->size() : 0)
                  << ", tgt_edges=" << (target_edges_ ? target_edges_->size() : 0)
                  << ", tgt_planars=" << (target_planars_ ? target_planars_->size() : 0)
                  << ", has_edge_tgt=" << has_edge_target
                  << ", has_planar_tgt=" << has_planar_target
                  << std::endl;
    }

    if (!has_edge_target && !has_planar_target)
    {
        std::cerr << "[FeatureReg DEBUG] DEGENERATE: No targets available!" << std::endl;
        result.degenerate = true;
        return result;
    }

    // Check sources
    const bool has_edge_source   = source_edges && !source_edges->empty();
    const bool has_planar_source = source_planars && !source_planars->empty();

    if (!has_edge_source && !has_planar_source)
    {
        std::cerr << "[FeatureReg DEBUG] DEGENERATE: No sources available!" << std::endl;
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

        // DEBUG: Log first iteration details
        if (iter == 0 && align_call_count % 50 == 1)
        {
            std::cerr << "[FeatureReg DEBUG] iter=0: correspondences=" << correspondences
                      << ", min_required=" << (config_.min_edge_correspondences + config_.min_planar_correspondences)
                      << ", edge_res=" << edge_residual
                      << ", planar_res=" << planar_residual
                      << std::endl;
        }

        if (correspondences < config_.min_edge_correspondences + config_.min_planar_correspondences)
        {
            if (align_call_count % 50 == 1)
            {
                std::cerr << "[FeatureReg DEBUG] DEGENERATE: Too few correspondences! "
                          << correspondences << " < " 
                          << (config_.min_edge_correspondences + config_.min_planar_correspondences)
                          << std::endl;
            }
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
    
    // DEBUG: Log final result
    if (align_call_count % 50 == 1)
    {
        std::cerr << "[FeatureReg DEBUG] Final: converged=" << result.converged
                  << ", degenerate=" << result.degenerate
                  << ", fitness=" << result.overall_fitness
                  << ", iterations=" << result.iterations
                  << std::endl;
    }

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

    // DEBUG: Track failure reasons
    static int debug_counter = 0;
    bool should_debug = (debug_counter++ % 50 == 0);  // Debug every 50th call
    int edge_fail_neighbors = 0, edge_fail_dist = 0, edge_fail_eigen = 0, edge_fail_residual = 0;
    int planar_fail_neighbors = 0, planar_fail_dist = 0, planar_fail_eigen = 0, planar_fail_residual = 0;

    // Process edge features
    if (source_edges && target_edges_ && !target_edges_->empty())
    {
        for (const auto& pt : *source_edges)
        {
            Eigen::Vector2d             residual;
            Eigen::Matrix<double, 2, 6> jacobian;
            int fail_reason = 0;

            if (computeEdgeResidualDebug(pt, transform, residual, jacobian, fail_reason))
            {
                // Gauss-Newton update: H += J^T * J, b += J^T * r
                H_buffer_ += config_.edge_weight * jacobian.transpose() * jacobian;
                b_buffer_ += config_.edge_weight * jacobian.transpose() * residual;
                edge_cost += residual.squaredNorm();
                ++edge_count;
            }
            else
            {
                if (fail_reason == 1) ++edge_fail_neighbors;
                else if (fail_reason == 2) ++edge_fail_dist;
                else if (fail_reason == 3) ++edge_fail_eigen;
                else if (fail_reason == 4) ++edge_fail_residual;
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
            int fail_reason = 0;

            if (computePlanarResidualDebug(pt, transform, residual, jacobian, fail_reason))
            {
                H_buffer_ += config_.planar_weight * jacobian.transpose() * jacobian;
                b_buffer_ += config_.planar_weight * jacobian.transpose() *
                             Eigen::Matrix<double, 1, 1>(residual);
                planar_cost += residual * residual;
                ++planar_count;
            }
            else
            {
                if (fail_reason == 1) ++planar_fail_neighbors;
                else if (fail_reason == 2) ++planar_fail_dist;
                else if (fail_reason == 3) ++planar_fail_eigen;
                else if (fail_reason == 4) ++planar_fail_residual;
            }
        }
    }

    if (should_debug)
    {
        std::cerr << "[FeatureReg DEBUG] Edge failures: neighbors=" << edge_fail_neighbors
                  << ", dist=" << edge_fail_dist << ", eigen=" << edge_fail_eigen
                  << ", residual=" << edge_fail_residual << ", success=" << edge_count << std::endl;
        std::cerr << "[FeatureReg DEBUG] Planar failures: neighbors=" << planar_fail_neighbors
                  << ", dist=" << planar_fail_dist << ", eigen=" << planar_fail_eigen
                  << ", residual=" << planar_fail_residual << ", success=" << planar_count << std::endl;
        std::cerr << "[FeatureReg DEBUG] Config: edge_search_r=" << config_.edge_search_radius
                  << ", planar_search_r=" << config_.planar_search_radius
                  << ", edge_neighbors=" << config_.edge_neighbors
                  << ", planar_neighbors=" << config_.planar_neighbors
                  << ", eigenvalue_ratio=" << config_.min_eigenvalue_ratio << std::endl;
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

    // For 2D mode, only check the 3x3 sub-block for (yaw, tx, ty)
    // Indices: 2=rz(yaw), 3=tx, 4=ty
    // The other DOFs (rx, ry, tz) are constrained to zero
    if (config_.is_2d_mode)
    {
        Eigen::Matrix3d H_2d;
        H_2d(0, 0) = H(2, 2);  // yaw-yaw
        H_2d(0, 1) = H(2, 3);  // yaw-tx
        H_2d(0, 2) = H(2, 4);  // yaw-ty
        H_2d(1, 0) = H(3, 2);  // tx-yaw
        H_2d(1, 1) = H(3, 3);  // tx-tx
        H_2d(1, 2) = H(3, 4);  // tx-ty
        H_2d(2, 0) = H(4, 2);  // ty-yaw
        H_2d(2, 1) = H(4, 3);  // ty-tx
        H_2d(2, 2) = H(4, 4);  // ty-ty

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(H_2d);
        const Eigen::Vector3d& eigenvalues = solver.eigenvalues();

        double min_ev = eigenvalues.minCoeff();
        double max_ev = eigenvalues.maxCoeff();

        return min_ev < 1e-6 || max_ev / min_ev > 1e6;
    }

    // Full 6-DOF check
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

bool FeatureRegistration::computeEdgeResidualDebug(
    const pcl::PointXYZ&         point,
    const Eigen::Matrix4f&       transform,
    Eigen::Vector2d&             residual,
    Eigen::Matrix<double, 2, 6>& jacobian,
    int&                         fail_reason)
{
    fail_reason = 0;
    
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
        fail_reason = 1;  // Not enough neighbors
        return false;
    }

    // Check max distance (distances are squared from KNN)
    if (distances[0] > config_.edge_search_radius * config_.edge_search_radius)
    {
        fail_reason = 2;  // Too far
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
        fail_reason = 3;  // Bad eigenvalue ratio
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
        fail_reason = 4;  // Residual too large
        return false;
    }

    // Compute Jacobian
    Eigen::Vector3d pt_orig(point.x, point.y, point.z);
    Eigen::Matrix3d rotation_matrix = transform.block<3, 3>(0, 0).cast<double>();

    Eigen::Matrix<double, 3, 6> J_full;
    Eigen::Vector3d             r_p = rotation_matrix * pt_orig;
    J_full.block<3, 3>(0, 0) = -skewSymmetric(r_p);
    J_full.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d cross_matrix;
    cross_matrix << 0, -line_dir.z(), line_dir.y(), line_dir.z(), 0, -line_dir.x(), -line_dir.y(),
        line_dir.x(), 0;

    Eigen::Matrix<double, 2, 3> proj;
    proj << 1, 0, 0, 0, 1, 0;

    jacobian = proj * cross_matrix * J_full;

    if (config_.is_2d_mode)
    {
        jacobian.col(0).setZero();
        jacobian.col(1).setZero();
        jacobian.col(5).setZero();
    }

    return true;
}

bool FeatureRegistration::computePlanarResidualDebug(
    const pcl::PointXYZ&         point,
    const Eigen::Matrix4f&       transform,
    double&                      residual,
    Eigen::Matrix<double, 1, 6>& jacobian,
    int&                         fail_reason)
{
    fail_reason = 0;

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
        fail_reason = 1;  // Not enough neighbors
        return false;
    }

    if (distances[0] > config_.planar_search_radius * config_.planar_search_radius)
    {
        fail_reason = 2;  // Too far
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
        fail_reason = 3;  // Bad eigenvalue ratio
        return false;
    }

    // Plane normal is eigenvector with smallest eigenvalue
    Eigen::Vector3d normal = eigenvectors.col(0).normalized();

    // Point-to-plane distance: d = (p - c) . normal
    residual = (pt_t - centroid).dot(normal);

    if (std::abs(residual) > config_.max_planar_residual)
    {
        fail_reason = 4;  // Residual too large
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

}  // namespace olive
