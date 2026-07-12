/**
 * @file knn_tree.hpp
 * @brief kNN tree backend selection for the scan matcher / keyframe map
 *
 * Default backend is PCL's FLANN kd-tree (available on every PCL). With
 * -DOLIVE_USE_NANOFLANN=ON (requires PCL >= 1.15 built with nanoflann, see
 * BUILDING.md) the faster nanoflann implementation is used instead. Both
 * return k-NN results in ascending distance order; radius results are only
 * ever consumed order-independently here.
 */

#ifndef OLIVE_FUSION_KNN_TREE_HPP_
#define OLIVE_FUSION_KNN_TREE_HPP_

#ifdef OLIVE_USE_NANOFLANN
#include <pcl/search/kdtree_nanoflann.h>
#else
#include <pcl/kdtree/kdtree_flann.h>
#endif

namespace olive
{

#ifdef OLIVE_USE_NANOFLANN
template <typename PointT>
using KnnTree = pcl::search::KdTreeNanoflann<PointT>;
#else
template <typename PointT>
using KnnTree = pcl::KdTreeFLANN<PointT>;
#endif

}  // namespace olive

#endif  // OLIVE_FUSION_KNN_TREE_HPP_
