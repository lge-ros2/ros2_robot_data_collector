#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace ros2_robot_data_collector
{

inline std::int64_t to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return (static_cast<std::int64_t>(stamp.sec) * 1000000000LL) +
         static_cast<std::int64_t>(stamp.nanosec);
}

struct ImageSample
{
  std::int64_t stamp_ns;
  std::uint32_t height;
  std::uint32_t width;
  std::uint32_t step;
  std::uint8_t is_bigendian;
  std::string encoding;
  std::vector<std::uint8_t> data;
};

struct JointStateSample
{
  std::int64_t stamp_ns;
  std::vector<std::string> names;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
};

struct ActionSample
{
  std::int64_t stamp_ns;
  std::vector<double> values;
};

struct Frame
{
  std::int64_t stamp_ns;
  std::shared_ptr<const ImageSample> image;
  std::shared_ptr<const JointStateSample> joint_state;
  ActionSample action;
};

inline std::size_t infer_joint_count(const sensor_msgs::msg::JointState & message)
{
  return std::max(
    {message.name.size(), message.position.size(), message.velocity.size(), message.effort.size()});
}

inline std::vector<double> normalize_numeric_field(
  const std::vector<double> & values,
  const std::size_t target_size)
{
  std::vector<double> normalized = values;
  normalized.resize(target_size, std::numeric_limits<double>::quiet_NaN());
  return normalized;
}

inline std::shared_ptr<ImageSample> make_image_sample(const sensor_msgs::msg::Image & message)
{
  auto sample = std::make_shared<ImageSample>();
  sample->stamp_ns = to_nanoseconds(message.header.stamp);
  sample->height = message.height;
  sample->width = message.width;
  sample->step = message.step;
  sample->is_bigendian = message.is_bigendian;
  sample->encoding = message.encoding;
  sample->data = message.data;
  return sample;
}

inline std::shared_ptr<JointStateSample> make_joint_state_sample(
  const sensor_msgs::msg::JointState & message)
{
  const std::size_t joint_count = infer_joint_count(message);

  auto sample = std::make_shared<JointStateSample>();
  sample->stamp_ns = to_nanoseconds(message.header.stamp);
  sample->names = message.name;
  sample->names.resize(joint_count);
  sample->positions = normalize_numeric_field(message.position, joint_count);
  sample->velocities = normalize_numeric_field(message.velocity, joint_count);
  sample->efforts = normalize_numeric_field(message.effort, joint_count);
  return sample;
}

inline ActionSample make_action_sample(
  const std_msgs::msg::Float64MultiArray & message,
  const std::int64_t stamp_ns)
{
  ActionSample sample;
  sample.stamp_ns = stamp_ns;
  sample.values = message.data;
  return sample;
}

}  // namespace ros2_robot_data_collector