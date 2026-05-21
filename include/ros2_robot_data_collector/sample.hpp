#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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

struct LidarScanSample
{
  std::int64_t stamp_ns;
  double angle_min;
  double angle_max;
  double angle_increment;
  double time_increment;
  double scan_time;
  double range_min;
  double range_max;
  std::vector<double> ranges;
  std::vector<double> intensities;
};

struct ActionSample
{
  std::int64_t stamp_ns;
  std::vector<double> values;
  std::vector<std::string> labels;
  std::string layout;
};

struct Frame
{
  std::uint64_t episode_index;
  std::int64_t stamp_ns;
  std::shared_ptr<const ImageSample> image;
  std::shared_ptr<const JointStateSample> joint_state;
  std::shared_ptr<const LidarScanSample> lidar;
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

inline std::vector<double> normalize_numeric_field(
  const std::vector<float> & values,
  const std::size_t target_size)
{
  std::vector<double> normalized;
  normalized.reserve(target_size);

  for (const float value : values) {
    normalized.push_back(static_cast<double>(value));
  }

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
  sample.layout = "float64_multi_array";
  return sample;
}

inline ActionSample make_action_sample(
  const geometry_msgs::msg::Twist & message,
  const std::int64_t stamp_ns)
{
  ActionSample sample;
  sample.stamp_ns = stamp_ns;
  sample.layout = "twist_linear_then_angular";
  sample.values = {
    message.linear.x,
    message.linear.y,
    message.linear.z,
    message.angular.x,
    message.angular.y,
    message.angular.z};
  sample.labels = {
    "linear/x",
    "linear/y",
    "linear/z",
    "angular/x",
    "angular/y",
    "angular/z"};
  return sample;
}

inline std::vector<std::string> make_joint_jog_action_labels(
  const std::vector<std::string> & joint_names,
  const std::size_t target_size)
{
  std::vector<std::string> normalized_names = joint_names;
  normalized_names.resize(target_size);

  std::vector<std::string> labels;
  labels.reserve(target_size * 2U);

  for (std::size_t index = 0; index < target_size; ++index) {
    const std::string base_name = normalized_names[index].empty() ?
      ("joint_" + std::to_string(index + 1U)) : normalized_names[index];
    labels.push_back(base_name + "/displacement");
  }

  for (std::size_t index = 0; index < target_size; ++index) {
    const std::string base_name = normalized_names[index].empty() ?
      ("joint_" + std::to_string(index + 1U)) : normalized_names[index];
    labels.push_back(base_name + "/velocity");
  }

  return labels;
}

inline ActionSample make_action_sample(
  const control_msgs::msg::JointJog & message,
  const std::int64_t stamp_ns)
{
  const std::size_t joint_count = std::max(
    {message.joint_names.size(), message.displacements.size(), message.velocities.size()});

  ActionSample sample;
  sample.stamp_ns = stamp_ns;
  sample.layout = "joint_jog_displacements_then_velocities";

  if (joint_count == 0U) {
    return sample;
  }

  const std::vector<double> displacements = normalize_numeric_field(message.displacements, joint_count);
  const std::vector<double> velocities = normalize_numeric_field(message.velocities, joint_count);

  sample.values.reserve(joint_count * 2U);
  sample.values.insert(sample.values.end(), displacements.begin(), displacements.end());
  sample.values.insert(sample.values.end(), velocities.begin(), velocities.end());
  sample.labels = make_joint_jog_action_labels(message.joint_names, joint_count);
  return sample;
}

inline std::shared_ptr<LidarScanSample> make_lidar_scan_sample(
  const sensor_msgs::msg::LaserScan & message)
{
  auto sample = std::make_shared<LidarScanSample>();
  sample->stamp_ns = to_nanoseconds(message.header.stamp);
  sample->angle_min = static_cast<double>(message.angle_min);
  sample->angle_max = static_cast<double>(message.angle_max);
  sample->angle_increment = static_cast<double>(message.angle_increment);
  sample->time_increment = static_cast<double>(message.time_increment);
  sample->scan_time = static_cast<double>(message.scan_time);
  sample->range_min = static_cast<double>(message.range_min);
  sample->range_max = static_cast<double>(message.range_max);
  sample->ranges = normalize_numeric_field(message.ranges, message.ranges.size());
  sample->intensities = normalize_numeric_field(message.intensities, message.ranges.size());
  return sample;
}

}  // namespace ros2_robot_data_collector