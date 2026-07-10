#include "olive/fusion/frontend/scan_preprocessor.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <optional>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace olive
{

namespace
{

/// Reads a per-point scalar field as float, handling the field's datatype
class ScalarReader
{
public:
    ScalarReader(const sensor_msgs::msg::PointCloud2& msg, const std::string& name)
    {
        for (const auto& field : msg.fields)
        {
            if (field.name == name)
            {
                offset_   = field.offset;
                datatype_ = field.datatype;
                found_    = true;
                break;
            }
        }
        data_ = msg.data.data();
        step_ = msg.point_step;
    }

    bool found() const { return found_; }

    double read(size_t index) const
    {
        const uint8_t* ptr = data_ + index * step_ + offset_;
        switch (datatype_)
        {
            case sensor_msgs::msg::PointField::FLOAT32:
            {
                float v;
                std::memcpy(&v, ptr, sizeof(v));
                return v;
            }
            case sensor_msgs::msg::PointField::FLOAT64:
            {
                double v;
                std::memcpy(&v, ptr, sizeof(v));
                return v;
            }
            case sensor_msgs::msg::PointField::UINT32:
            {
                uint32_t v;
                std::memcpy(&v, ptr, sizeof(v));
                return v;
            }
            case sensor_msgs::msg::PointField::UINT16:
            {
                uint16_t v;
                std::memcpy(&v, ptr, sizeof(v));
                return v;
            }
            case sensor_msgs::msg::PointField::UINT8:
                return *ptr;
            case sensor_msgs::msg::PointField::INT32:
            {
                int32_t v;
                std::memcpy(&v, ptr, sizeof(v));
                return v;
            }
            default:
                return 0.0;
        }
    }

private:
    const uint8_t* data_     = nullptr;
    size_t         step_     = 0;
    size_t         offset_   = 0;
    uint8_t        datatype_ = 0;
    bool           found_    = false;
};

/// Per-point time extractor: converts any supported encoding to seconds
/// relative to the header stamp.
class TimeReader
{
public:
    TimeReader(const sensor_msgs::msg::PointCloud2& msg, const std::string& mode, double header_stamp)
      : header_stamp_(header_stamp)
    {
        if (mode == "none")
            return;

        const std::vector<std::string> candidates =
            (mode == "auto") ? std::vector<std::string>{ "time", "t", "timestamp" } :
                               std::vector<std::string>{ mode };
        for (const auto& name : candidates)
        {
            ScalarReader reader(msg, name);
            if (!reader.found())
                continue;
            reader_.emplace(reader);
            name_ = name;
            for (const auto& field : msg.fields)
            {
                if (field.name == name)
                {
                    is_nanoseconds_ = field.datatype == sensor_msgs::msg::PointField::UINT32;
                    is_absolute_    = field.datatype == sensor_msgs::msg::PointField::FLOAT64;
                    break;
                }
            }
            found_ = true;
            break;
        }
    }

    bool found() const { return found_; }

    float relTime(size_t index) const
    {
        const double raw = reader_->read(index);
        if (is_nanoseconds_)
            return static_cast<float>(raw * 1e-9);
        if (is_absolute_)
            return static_cast<float>(raw - header_stamp_);
        return static_cast<float>(raw);
    }

private:
    std::optional<ScalarReader> reader_;
    std::string                 name_;
    double       header_stamp_   = 0.0;
    bool         found_          = false;
    bool         is_nanoseconds_ = false;
    bool         is_absolute_    = false;
};

}  // namespace

ScanPreprocessor::ScanPreprocessor(const PreprocessorConfig& config)
  : config_(config)
{}

bool ScanPreprocessor::process(const sensor_msgs::msg::PointCloud2& msg, ScanImage& out) const
{
    out.clear();
    out.stamp = static_cast<double>(msg.header.stamp.sec) + 1e-9 * msg.header.stamp.nanosec;

    if (msg.height >= 2 && msg.width >= 8)
        return processOrganized(msg, out);
    return processUnorganized(msg, out);
}

bool ScanPreprocessor::processOrganized(const sensor_msgs::msg::PointCloud2& msg, ScanImage& out) const
{
    const int num_rings   = static_cast<int>(msg.height);
    const int num_columns = static_cast<int>(msg.width);

    const size_t max_points = static_cast<size_t>(num_rings) * num_columns;
    out.points->reserve(max_points);
    out.range.reserve(max_points);
    out.column.reserve(max_points);
    out.rel_time.reserve(max_points);
    out.ring_start.resize(num_rings);
    out.ring_end.resize(num_rings);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");
    const TimeReader time_reader(msg, config_.point_time_field, out.stamp);

    const Eigen::Affine3f& tf = config_.base_from_lidar;

    size_t flat_index = 0;
    for (int ring = 0; ring < num_rings; ++ring)
    {
        out.ring_start[ring] = static_cast<int>(out.points->size());

        for (int col = 0; col < num_columns; ++col, ++it_x, ++it_y, ++it_z, ++flat_index)
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
            out.rel_time.push_back(time_reader.found() ? time_reader.relTime(flat_index) : 0.0F);
        }

        out.ring_end[ring] = static_cast<int>(out.points->size());
    }

    return out.points->size() > 64;
}

bool ScanPreprocessor::processUnorganized(const sensor_msgs::msg::PointCloud2& msg, ScanImage& out)
    const
{
    // Velodyne-style flat clouds: recover the ring structure from a ring
    // field and the column index from azimuth at the ring's native
    // resolution (the column feeds occlusion/gap logic downstream).
    if (msg.width < 64 || msg.height != 1)
        return false;

    const std::string ring_name = config_.ring_field == "auto" ? "ring" : config_.ring_field;
    ScalarReader      ring_reader(msg, ring_name);
    if (!ring_reader.found())
        return false;  // cannot recover scan lines without a ring field

    sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");
    const TimeReader time_reader(msg, config_.point_time_field, out.stamp);

    struct RawPoint
    {
        float azimuth;
        float x, y, z;
        float range;
        float rel_time;
    };
    std::vector<std::vector<RawPoint>> rings;

    for (size_t index = 0; index < msg.width; ++index, ++it_x, ++it_y, ++it_z)
    {
        const float x = *it_x;
        const float y = *it_y;
        const float z = *it_z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            continue;

        const float range = std::sqrt(x * x + y * y + z * z);
        if (range < config_.min_range || range > config_.max_range)
            continue;

        const int ring = static_cast<int>(ring_reader.read(index));
        if (ring < 0 || ring > 255)
            continue;
        if (static_cast<size_t>(ring) >= rings.size())
            rings.resize(ring + 1);

        // Azimuth in the SENSOR frame (before the extrinsic), wrapped to [0, 2pi)
        float azimuth = std::atan2(y, x);
        if (azimuth < 0.0F)
            azimuth += 2.0F * static_cast<float>(M_PI);

        rings[ring].push_back(
            { azimuth, x, y, z, range, time_reader.found() ? time_reader.relTime(index) : 0.0F });
    }

    size_t columns_per_ring = 0;
    for (const auto& ring : rings)
        columns_per_ring = std::max(columns_per_ring, ring.size());
    if (columns_per_ring < 8)
        return false;

    const float column_scale =
        static_cast<float>(columns_per_ring) / (2.0F * static_cast<float>(M_PI));
    const Eigen::Affine3f& tf = config_.base_from_lidar;

    out.ring_start.resize(rings.size());
    out.ring_end.resize(rings.size());
    size_t total = 0;
    for (const auto& ring : rings)
        total += ring.size();
    out.points->reserve(total);
    out.range.reserve(total);
    out.column.reserve(total);
    out.rel_time.reserve(total);

    for (size_t ring = 0; ring < rings.size(); ++ring)
    {
        auto& points = rings[ring];
        std::sort(points.begin(), points.end(), [](const RawPoint& a, const RawPoint& b) {
            return a.azimuth < b.azimuth;
        });

        out.ring_start[ring] = static_cast<int>(out.points->size());
        for (const RawPoint& raw : points)
        {
            const Eigen::Vector3f p = tf * Eigen::Vector3f(raw.x, raw.y, raw.z);
            CloudPoint            point;
            point.x         = p.x();
            point.y         = p.y();
            point.z         = p.z();
            point.intensity = static_cast<float>(ring);
            out.points->push_back(point);
            out.range.push_back(raw.range);
            out.column.push_back(static_cast<int>(raw.azimuth * column_scale));
            out.rel_time.push_back(raw.rel_time);
        }
        out.ring_end[ring] = static_cast<int>(out.points->size());
    }

    return out.points->size() > 64;
}

void deskewScan(
    ScanImage&                             scan,
    const std::vector<Eigen::Quaterniond>& rotations,
    double                                 t_min,
    double                                 t_max)
{
    const int bins = static_cast<int>(rotations.size()) - 1;
    if (bins < 1 || t_max - t_min < 1e-6 || scan.rel_time.size() != scan.points->size())
        return;

    // rotations[k] is R(t_min -> bin k). Reference is the header stamp
    // (rel_time == 0): R(ref -> pt) = R(t_min -> ref)^-1 * R(t_min -> pt),
    // and a point measured at t_pt maps to the reference body frame as
    // p_ref = R(ref -> pt) * p_pt.
    const double bin_width = (t_max - t_min) / bins;
    const int    ref_bin =
        std::clamp(static_cast<int>(std::lround((0.0 - t_min) / bin_width)), 0, bins);
    const Eigen::Quaterniond ref_inverse = rotations[ref_bin].conjugate();

    std::vector<Eigen::Matrix3f> corrections(rotations.size());
    for (size_t k = 0; k < rotations.size(); ++k)
        corrections[k] = (ref_inverse * rotations[k]).toRotationMatrix().cast<float>();

    for (size_t i = 0; i < scan.points->size(); ++i)
    {
        const int bin = std::clamp(
            static_cast<int>(std::lround((scan.rel_time[i] - t_min) / bin_width)),
            0,
            bins);
        if (bin == ref_bin)
            continue;
        auto& point            = scan.points->points[i];
        point.getVector3fMap() = corrections[bin] * point.getVector3fMap();
    }
}

}  // namespace olive
