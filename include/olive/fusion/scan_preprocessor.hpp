/**
 * @file scan_preprocessor.hpp
 * @brief Converts incoming PointCloud2 scans into the ring-major ScanImage layout
 */

#ifndef OLIVE_FUSION_SCAN_PREPROCESSOR_HPP_
#define OLIVE_FUSION_SCAN_PREPROCESSOR_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "olive/fusion/fusion_types.hpp"

namespace olive
{

/**
 * @brief Unrolls an organized (rings x columns) PointCloud2 into a ScanImage.
 *
 * Points are range-gated, checked for validity, and transformed into the base
 * frame with the fixed lidar extrinsic. The output preserves ring-major,
 * increasing-azimuth order, which the feature extractor relies on.
 */
class ScanPreprocessor
{
public:
    explicit ScanPreprocessor(const PreprocessorConfig& config);

    /**
     * @brief Process one scan into @p out (reused between calls)
     * @return false when the cloud is not organized or contains too few points
     */
    bool process(const sensor_msgs::msg::PointCloud2& msg, ScanImage& out) const;

private:
    PreprocessorConfig config_;
};

}  // namespace olive

#endif  // OLIVE_FUSION_SCAN_PREPROCESSOR_HPP_
