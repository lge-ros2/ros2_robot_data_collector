#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "ros2_robot_data_collector/topic_resolver.hpp"

namespace ros2_robot_data_collector
{

class SampleDataPublisher : public rclcpp::Node
{
public:
  SampleDataPublisher()
  : rclcpp::Node("sample_data_publisher")
  {
    const std::string namespace_prefix = declare_parameter<std::string>("namespace_prefix", "");
    const std::string image_topic = declare_parameter<std::string>(
      "image_topic", "camera/front/image_raw");
    const std::string joint_state_topic = declare_parameter<std::string>(
      "joint_state_topic", "joint_states");
    const std::string action_topic = declare_parameter<std::string>("action_topic", "actions");
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 10.0);
    image_height_ = static_cast<std::size_t>(declare_parameter<int>("image_height", 8));
    image_width_ = static_cast<std::size_t>(declare_parameter<int>("image_width", 8));
    joint_count_ = static_cast<std::size_t>(declare_parameter<int>("joint_count", 6));
    action_count_ = static_cast<std::size_t>(declare_parameter<int>("action_count", 6));
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_front");

    if (publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be greater than zero.");
    }
    if (image_height_ == 0U || image_width_ == 0U) {
      throw std::invalid_argument("image_height and image_width must be greater than zero.");
    }
    if (joint_count_ == 0U || action_count_ == 0U) {
      throw std::invalid_argument("joint_count and action_count must be greater than zero.");
    }

    const std::string resolved_image_topic = resolve_topic_name(namespace_prefix, image_topic);
    const std::string resolved_joint_state_topic = resolve_topic_name(namespace_prefix, joint_state_topic);
    const std::string resolved_action_topic = resolve_topic_name(namespace_prefix, action_topic);

    auto sensor_qos = rclcpp::SensorDataQoS();
    sensor_qos.keep_last(10);

    auto action_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    action_qos.reliable();

    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(resolved_image_topic, sensor_qos);
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      resolved_joint_state_topic, sensor_qos);
    action_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      resolved_action_topic, action_qos);

    joint_names_.reserve(joint_count_);
    for (std::size_t index = 0; index < joint_count_; ++index) {
      joint_names_.push_back("joint_" + std::to_string(index + 1U));
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz));
    publish_timer_ = create_wall_timer(period, std::bind(&SampleDataPublisher::publish_samples, this));

    RCLCPP_INFO(
      get_logger(),
      "Sample publisher configured with namespace_prefix='%s', image='%s', joint_state='%s', action='%s', rate=%.2f Hz",
      namespace_prefix.c_str(),
      resolved_image_topic.c_str(),
      resolved_joint_state_topic.c_str(),
      resolved_action_topic.c_str(),
      publish_rate_hz);
  }

private:
  void publish_samples()
  {
    const rclcpp::Time stamp = now();
    image_publisher_->publish(make_image_message(stamp));
    joint_state_publisher_->publish(make_joint_state_message(stamp));
    action_publisher_->publish(make_action_message());
    ++sequence_number_;
  }

  sensor_msgs::msg::Image make_image_message(const rclcpp::Time & stamp) const
  {
    sensor_msgs::msg::Image message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    message.height = static_cast<std::uint32_t>(image_height_);
    message.width = static_cast<std::uint32_t>(image_width_);
    message.encoding = "mono8";
    message.is_bigendian = 0;
    message.step = static_cast<std::uint32_t>(image_width_);
    message.data.resize(image_height_ * image_width_);

    for (std::size_t row = 0; row < image_height_; ++row) {
      for (std::size_t column = 0; column < image_width_; ++column) {
        const std::size_t offset = (row * image_width_) + column;
        message.data[offset] = static_cast<std::uint8_t>((offset + sequence_number_) % 255U);
      }
    }

    return message;
  }

  sensor_msgs::msg::JointState make_joint_state_message(const rclcpp::Time & stamp) const
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = stamp;
    message.name = joint_names_;
    message.position.reserve(joint_count_);
    message.velocity.reserve(joint_count_);
    message.effort.reserve(joint_count_);

    const double time_seconds = static_cast<double>(sequence_number_) * 0.1;
    for (std::size_t index = 0; index < joint_count_; ++index) {
      const double phase = time_seconds + static_cast<double>(index) * 0.15;
      message.position.push_back(std::sin(phase));
      message.velocity.push_back(std::cos(phase));
      message.effort.push_back(0.25 * static_cast<double>(index + 1U));
    }

    return message;
  }

  std_msgs::msg::Float64MultiArray make_action_message() const
  {
    std_msgs::msg::Float64MultiArray message;
    message.data.reserve(action_count_);

    const double base = static_cast<double>(sequence_number_) * 0.05;
    for (std::size_t index = 0; index < action_count_; ++index) {
      message.data.push_back(base + (0.1 * static_cast<double>(index)));
    }

    return message;
  }

  std::size_t image_height_ = 0;
  std::size_t image_width_ = 0;
  std::size_t joint_count_ = 0;
  std::size_t action_count_ = 0;
  std::string frame_id_;
  std::vector<std::string> joint_names_;
  std::uint64_t sequence_number_ = 0;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr action_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace ros2_robot_data_collector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_robot_data_collector::SampleDataPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}