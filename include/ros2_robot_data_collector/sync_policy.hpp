#pragma once

#include <algorithm>
#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <vector>

namespace ros2_robot_data_collector
{

inline std::int64_t timestamp_span_ns(const std::initializer_list<std::int64_t> timestamps)
{
  if (timestamps.size() == 0U) {
    throw std::invalid_argument("timestamp_span_ns requires at least one timestamp.");
  }

  const auto bounds = std::minmax_element(timestamps.begin(), timestamps.end());
  return *bounds.second - *bounds.first;
}

inline std::int64_t timestamp_span_ns(const std::vector<std::int64_t> & timestamps)
{
  if (timestamps.empty()) {
    throw std::invalid_argument("timestamp_span_ns requires at least one timestamp.");
  }

  const auto bounds = std::minmax_element(timestamps.begin(), timestamps.end());
  return *bounds.second - *bounds.first;
}

inline bool timestamps_within_window(
  const std::initializer_list<std::int64_t> timestamps,
  const std::int64_t max_span_ns)
{
  if (max_span_ns < 0) {
    throw std::invalid_argument("max_span_ns must be greater than or equal to zero.");
  }

  return timestamp_span_ns(timestamps) <= max_span_ns;
}

inline bool timestamps_within_window(
  const std::vector<std::int64_t> & timestamps,
  const std::int64_t max_span_ns)
{
  if (max_span_ns < 0) {
    throw std::invalid_argument("max_span_ns must be greater than or equal to zero.");
  }

  return timestamp_span_ns(timestamps) <= max_span_ns;
}

}  // namespace ros2_robot_data_collector