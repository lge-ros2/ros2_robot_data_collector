#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace ros2_robot_data_collector
{

struct CollectorConfig
{
  std::string namespace_prefix;
  std::string image_topic;
  std::string joint_state_topic;
  std::string action_topic;
  std::string output_path;
  std::size_t topic_queue_depth;
  std::size_t writer_queue_depth;
  std::size_t batch_size;
  int sync_slop_ms;
  int flush_interval_ms;
  bool approximate_sync;
  bool enable_compression;
};

inline CollectorConfig load_collector_config(rclcpp::Node & node)
{
  CollectorConfig config;
  config.namespace_prefix = node.declare_parameter<std::string>("namespace_prefix", "");
  config.image_topic = node.declare_parameter<std::string>("image_topic", "camera/front/image_raw");
  config.joint_state_topic = node.declare_parameter<std::string>("joint_state_topic", "joint_states");
  config.action_topic = node.declare_parameter<std::string>("action_topic", "actions");
  config.output_path = node.declare_parameter<std::string>("output_path", "/tmp/ros2_robot_data.h5");
  config.topic_queue_depth = static_cast<std::size_t>(
    node.declare_parameter<int>("topic_queue_depth", 64));
  config.writer_queue_depth = static_cast<std::size_t>(
    node.declare_parameter<int>("writer_queue_depth", 512));
  config.batch_size = static_cast<std::size_t>(node.declare_parameter<int>("batch_size", 32));
  config.sync_slop_ms = node.declare_parameter<int>("sync_slop_ms", 50);
  config.flush_interval_ms = node.declare_parameter<int>("flush_interval_ms", 1000);
  config.approximate_sync = node.declare_parameter<bool>("approximate_sync", true);
  config.enable_compression = node.declare_parameter<bool>("enable_compression", false);

  if (config.topic_queue_depth == 0U) {
    throw std::invalid_argument("topic_queue_depth must be greater than zero.");
  }
  if (config.writer_queue_depth == 0U) {
    throw std::invalid_argument("writer_queue_depth must be greater than zero.");
  }
  if (config.batch_size == 0U) {
    throw std::invalid_argument("batch_size must be greater than zero.");
  }
  if (config.sync_slop_ms < 0) {
    throw std::invalid_argument("sync_slop_ms must be greater than or equal to zero.");
  }
  if (config.flush_interval_ms <= 0) {
    throw std::invalid_argument("flush_interval_ms must be greater than zero.");
  }

  return config;
}

}  // namespace ros2_robot_data_collector