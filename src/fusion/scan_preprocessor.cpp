#include "olive/fusion/scan_preprocessor.hpp"

#include <cmath>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace olive
{

ScanPreprocessor::ScanPreprocessor(const PreprocessorConfig& config)
  : config_(config)
{}

bool ScanPreprocessor::process(const sensor_msgs::msg::PointCloud2& msg, ScanImage& out) const
{
    out.clear();
    out.stamp = static_cast<double>(msg.header.stamp.sec) + 1e-9 * msg.header.stamp.nanosec;

    if (msg.height < 2 || msg.width < 8)
        return false;  // needs an organized rings x cols cloud

    const int num_rings   = static_cast<int>(msg.height);
    const int num_columns = static_cast<int>(msg.width);

    const size_t max_points = static_cast<size_t>(num_rings) * num_columns;
    out.points->reserve(max_points);
    out.range.reserve(max_points);
    out.column.reserve(max_points);
    out.ring_start.resize(num_rings);
    out.ring_end.resize(num_rings);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");

    const Eigen::Affine3f& tf = config_.base_from_lidar;

    for (int ring = 0; ring < num_rings; ++ring)
    {
        out.ring_start[ring] = static_cast<int>(out.points->size());

        for (int col = 0; col < num_columns; ++col, ++it_x, ++it_y, ++it_z)
        {
            const float x = *it_x;
            const float y = *it_y;
            const float z = *it_z;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;

            const float range = std::sqrt(x * x + y * y + z * z);
            if (range < config_.min_range || range > config_.max_range)
                continue;

            const Eigen::Vector3f p = tf * Eigen::Vector3f(x, y, z);

            CloudPoint point;
            point.x         = p.x();
            point.y         = p.y();
            point.z         = p.z();
            point.intensity = static_cast<float>(ring);
            out.points->push_back(point);
            out.range.push_back(range);
            out.column.push_back(col);
        }

        out.ring_end[ring] = static_cast<int>(out.points->size());
    }

    return out.points->size() > 64;
}

}  // namespace olive
