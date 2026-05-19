#pragma once

#include <stdexcept>
#include <string>

namespace ros2_robot_data_collector
{

inline std::string trim_slashes(const std::string & value)
{
  std::size_t begin = 0;
  while (begin < value.size() && value[begin] == '/') {
    ++begin;
  }

  std::size_t end = value.size();
  while (end > begin && value[end - 1] == '/') {
    --end;
  }

  return value.substr(begin, end - begin);
}

inline std::string canonicalize_namespace_prefix(const std::string & namespace_prefix)
{
  const std::string trimmed = trim_slashes(namespace_prefix);
  if (trimmed.empty()) {
    return "";
  }

  return "/" + trimmed;
}

inline std::string resolve_topic_name(
  const std::string & namespace_prefix,
  const std::string & topic_name)
{
  if (topic_name.empty()) {
    throw std::invalid_argument("Topic name must not be empty.");
  }

  if (topic_name.front() == '/') {
    return topic_name;
  }

  const std::string sanitized_topic = trim_slashes(topic_name);
  if (sanitized_topic.empty()) {
    throw std::invalid_argument("Topic name must contain a non-slash character.");
  }

  const std::string canonical_prefix = canonicalize_namespace_prefix(namespace_prefix);
  if (canonical_prefix.empty()) {
    return "/" + sanitized_topic;
  }

  return canonical_prefix + "/" + sanitized_topic;
}

}  // namespace ros2_robot_data_collector