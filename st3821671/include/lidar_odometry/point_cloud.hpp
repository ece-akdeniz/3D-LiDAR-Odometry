#pragma once

#include "lidar_odometry/core.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace lidar_odometry {
struct Filter {
  double min_range = 0.5, max_range = 80.0, intensity_threshold = 0.0;
  std::size_t max_input_points = 200000;
  void validate() const {
    if (!std::isfinite(min_range) || !std::isfinite(max_range) ||
        !std::isfinite(intensity_threshold) || min_range < 0 ||
        max_range <= min_range || max_input_points == 0)
      throw std::invalid_argument("invalid range/intensity/input limit");
  }
};

namespace detail {
inline std::size_t fieldSize(std::uint8_t type) {
  switch (type) {
    case 1: case 2: return 1;
    case 3: case 4: return 2;
    case 5: case 6: case 7: return 4;
    case 8: return 8;
    default: throw std::invalid_argument("unsupported PointCloud2 field datatype");
  }
}
template<typename T>
T load(const std::uint8_t *data, bool bigendian) {
  std::array<std::uint8_t, sizeof(T)> bytes;
  std::copy_n(data, sizeof(T), bytes.begin());
  const std::uint16_t one = 1;
  const bool host_bigendian = *reinterpret_cast<const std::uint8_t *>(&one) == 0;
  if (bigendian != host_bigendian) std::reverse(bytes.begin(), bytes.end());
  T value;
  std::memcpy(&value, bytes.data(), sizeof(T));
  return value;
}
inline double number(const std::uint8_t *p, std::uint8_t type, bool bigendian) {
  switch (type) {
    case 1: return load<std::int8_t>(p, bigendian);
    case 2: return load<std::uint8_t>(p, bigendian);
    case 3: return load<std::int16_t>(p, bigendian);
    case 4: return load<std::uint16_t>(p, bigendian);
    case 5: return load<std::int32_t>(p, bigendian);
    case 6: return load<std::uint32_t>(p, bigendian);
    case 7: return load<float>(p, bigendian);
    case 8: return load<double>(p, bigendian);
    default: throw std::invalid_argument("unsupported PointCloud2 field datatype");
  }
}
}  // namespace detail

// Duck typed so binary layout handling can be tested without a ROS installation.
// Honors row padding, field offsets/types and endianness; never casts unaligned data.
template<typename Message>
Cloud decodeCloud(const Message &msg, const Filter &filter) {
  filter.validate();
  if (msg.width == 0 || msg.height == 0) return {};
  if (msg.point_step == 0 ||
      static_cast<std::uint64_t>(msg.width) * msg.point_step > msg.row_step ||
      static_cast<std::uint64_t>(msg.height) * msg.row_step > msg.data.size())
    throw std::invalid_argument("truncated or inconsistent PointCloud2 layout");
  struct Field { std::uint32_t offset = 0; std::uint8_t type = 0; bool found = false; };
  Field x, y, z, intensity;
  for (const auto &field : msg.fields) {
    Field *out = nullptr;
    if (field.name == "x") out = &x;
    else if (field.name == "y") out = &y;
    else if (field.name == "z") out = &z;
    else if (field.name == "intensity") out = &intensity;
    if (!out) continue;
    if (out->found || field.count != 1 ||
        static_cast<std::uint64_t>(field.offset) + detail::fieldSize(field.datatype) > msg.point_step)
      throw std::invalid_argument("invalid or duplicate PointCloud2 scalar field");
    *out = {field.offset, field.datatype, true};
  }
  if (!x.found || !y.found || !z.found)
    throw std::invalid_argument("PointCloud2 requires x, y and z fields");
  if (!intensity.found && filter.intensity_threshold > 0)
    throw std::invalid_argument("positive intensity threshold requires an intensity field");
  const std::uint64_t count = static_cast<std::uint64_t>(msg.width) * msg.height;
  const std::uint64_t stride = std::max<std::uint64_t>(1,
      (count + filter.max_input_points - 1) / filter.max_input_points);
  Cloud result;
  result.reserve(static_cast<std::size_t>(std::min<std::uint64_t>(count, filter.max_input_points)));
  for (std::uint64_t i = 0; i < count; i += stride) {
    const auto *base = msg.data.data() + (i / msg.width) * msg.row_step +
                       (i % msg.width) * msg.point_step;
    const Eigen::Vector3d p(detail::number(base + x.offset, x.type, msg.is_bigendian),
                            detail::number(base + y.offset, y.type, msg.is_bigendian),
                            detail::number(base + z.offset, z.type, msg.is_bigendian));
    if (!p.allFinite()) continue;
    const double range = p.norm();
    if (range < filter.min_range || range > filter.max_range) continue;
    if (intensity.found) {
      const double value = detail::number(base + intensity.offset, intensity.type, msg.is_bigendian);
      if (!std::isfinite(value) || value < filter.intensity_threshold) continue;
    }
    result.push_back(p);
  }
  return result;
}
}  // namespace lidar_odometry
