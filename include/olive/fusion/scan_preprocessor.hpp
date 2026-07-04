/**
 * @file scan_preprocessor.hpp
 * @brief Converts incoming PointCloud2 scans into the ring-major ScanImage layout
 */

#ifndef OLIVE_FUSION_SCAN_PREPROCESSOR_HPP_
#define OLIVE_FUSION_SCAN_PREPROCESSOR_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

#include "olive/fusion/fusion_types.hpp"

namespace olive
{

/**
 * @brief Unrolls a PointCloud2 into a ScanImage.
 *
 * Two input layouts are supported:
 *  - organized (rings x columns, e.g. simulated or Ouster-style clouds):
 *    rows are rings, columns come from the grid;
 *  - unorganized (height == 1, e.g. Velodyne driver output): points are
 *    grouped by their `ring` field, sorted by azimuth, and the column index
 *    is derived from the azimuth at the ring's native resolution.
 *
 * Points are range-gated, checked for validity, and transformed into the
 * base frame with the fixed lidar extrinsic. Per-point times are extracted
 * from a `time` (float32, seconds relative to the header stamp), `t`
 * (uint32, nanoseconds from frame start) or `timestamp` (float64, absolute
 * seconds) field into ScanImage::rel_time; without a time field rel_time is
 * all zeros and deskew becomes a no-op.
 */
class ScanPreprocessor
{
public:
    explicit ScanPreprocessor(const PreprocessorConfig& config);

    /**
     * @brief Process one scan into @p out (reused between calls)
     * @return false when the cloud has no usable layout or too few points
     */
    bool process(const sensor_msgs::msg::PointCloud2& msg, ScanImage& out) const;

private:
    bool processOrganized(const sensor_msgs::msg::PointCloud2& msg, ScanImage& out) const;
    bool processUnorganized(const sensor_msgs::msg::PointCloud2& msg, ScanImage& out) const;

    PreprocessorConfig config_;
};

/**
 * @brief Rotate every point to the scan's reference (header-stamp) time.
 *
 * @p rotations[k] is the body rotation from the reference time to the k-th
 * of n evenly spaced bin edges across [t_min, t_max] of rel_time (as
 * produced by ImuBuffer::sampleRotations over that interval). Points are
 * rotated in the base frame — translation during the scan is ignored, which
 * costs millimeters at ground-robot speeds. No-op when rel_time is empty or
 * all zeros.
 */
void deskewScan(
    ScanImage&                             scan,
    const std::vector<Eigen::Quaterniond>& rotations,
    double                                 t_min,
    double                                 t_max);

}  // namespace olive

#endif  // OLIVE_FUSION_SCAN_PREPROCESSOR_HPP_
