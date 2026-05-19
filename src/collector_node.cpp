#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "ros2_robot_data_collector/collector_config.hpp"
#include "ros2_robot_data_collector/hdf5_writer.hpp"
#include "ros2_robot_data_collector/sample.hpp"
#include "ros2_robot_data_collector/sync_policy.hpp"
#include "ros2_robot_data_collector/topic_resolver.hpp"

namespace ros2_robot_data_collector
{

namespace
{

std::int64_t absolute_difference(const std::int64_t left, const std::int64_t right)
{
  return (left >= right) ? (left - right) : (right - left);
}

}  // namespace

class CollectorNode : public rclcpp::Node
{
public:
  CollectorNode()
  : rclcpp::Node("collector_node"),
    config_(load_collector_config(*this)),
    sync_slop_ns_(static_cast<std::int64_t>(config_.sync_slop_ms) * 1000000LL),
    stale_window_ns_(std::max<std::int64_t>(sync_slop_ns_ * 4LL, 500000000LL)),
    writer_(config_, get_logger())
  {
    subscription_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = subscription_group_;

    auto sensor_qos = rclcpp::SensorDataQoS();
    sensor_qos.keep_last(config_.topic_queue_depth);

    auto action_qos = rclcpp::QoS(rclcpp::KeepLast(config_.topic_queue_depth));
    action_qos.reliable();

    resolved_image_topic_ = resolve_topic_name(config_.namespace_prefix, config_.image_topic);
    resolved_joint_state_topic_ = resolve_topic_name(config_.namespace_prefix, config_.joint_state_topic);
    resolved_action_topic_ = resolve_topic_name(config_.namespace_prefix, config_.action_topic);

    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      resolved_image_topic_,
      sensor_qos,
      std::bind(&CollectorNode::on_image, this, std::placeholders::_1),
      subscription_options);

    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      resolved_joint_state_topic_,
      sensor_qos,
      std::bind(&CollectorNode::on_joint_state, this, std::placeholders::_1),
      subscription_options);

    action_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      resolved_action_topic_,
      action_qos,
      std::bind(&CollectorNode::on_action, this, std::placeholders::_1),
      subscription_options);

    stats_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&CollectorNode::log_stats, this));

    RCLCPP_INFO(
      get_logger(),
      "Collector configured with namespace_prefix='%s', image='%s', joint_state='%s', action='%s', output='%s'",
      config_.namespace_prefix.c_str(),
      resolved_image_topic_.c_str(),
      resolved_joint_state_topic_.c_str(),
      resolved_action_topic_.c_str(),
      config_.output_path.c_str());

    RCLCPP_WARN(
      get_logger(),
      "Action messages currently use receive time as their timestamp. For strict temporal alignment across replay or transport jitter, use a stamped action message type.");
  }

private:
  template<typename SampleT>
  void push_bounded_locked(
    std::deque<std::shared_ptr<const SampleT>> & queue,
    std::shared_ptr<const SampleT> sample,
    std::atomic<std::uint64_t> & dropped_counter)
  {
    if (queue.size() >= config_.topic_queue_depth) {
      queue.pop_front();
      ++dropped_counter;
    }

    queue.push_back(std::move(sample));
  }

  template<typename SampleT>
  std::shared_ptr<const SampleT> find_nearest_locked(
    const std::deque<std::shared_ptr<const SampleT>> & queue,
    const std::int64_t anchor_stamp_ns) const
  {
    std::shared_ptr<const SampleT> best_sample;
    std::int64_t best_delta = std::numeric_limits<std::int64_t>::max();

    for (const auto & sample : queue) {
      if (!sample) {
        continue;
      }

      const std::int64_t delta = absolute_difference(sample->stamp_ns, anchor_stamp_ns);
      const bool matches = config_.approximate_sync ? (delta <= sync_slop_ns_) : (delta == 0);
      if (!matches) {
        continue;
      }

      if (delta < best_delta) {
        best_delta = delta;
        best_sample = sample;
        if (delta == 0) {
          break;
        }
      }
    }

    return best_sample;
  }

  template<typename SampleT>
  void prune_stale_queue_locked(
    std::deque<std::shared_ptr<const SampleT>> & queue,
    const std::int64_t anchor_stamp_ns)
  {
    const std::int64_t cutoff = anchor_stamp_ns - stale_window_ns_;
    while (!queue.empty() && queue.front() && queue.front()->stamp_ns < cutoff) {
      queue.pop_front();
    }
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    std::shared_ptr<const ImageSample> sample = make_image_sample(*message);

    std::lock_guard<std::mutex> lock(samples_mutex_);
    push_bounded_locked(image_samples_, std::move(sample), image_queue_drop_count_);
  }

  void on_joint_state(const sensor_msgs::msg::JointState::ConstSharedPtr message)
  {
    std::shared_ptr<const JointStateSample> sample = make_joint_state_sample(*message);

    std::lock_guard<std::mutex> lock(samples_mutex_);
    push_bounded_locked(joint_state_samples_, std::move(sample), joint_queue_drop_count_);
  }

  void on_action(const std_msgs::msg::Float64MultiArray::ConstSharedPtr message)
  {
    const std::int64_t action_stamp_ns = get_clock()->now().nanoseconds();
    ActionSample action = make_action_sample(*message, action_stamp_ns);

    std::shared_ptr<const ImageSample> image;
    std::shared_ptr<const JointStateSample> joint_state;
    {
      std::lock_guard<std::mutex> lock(samples_mutex_);
      prune_stale_queue_locked(image_samples_, action_stamp_ns);
      prune_stale_queue_locked(joint_state_samples_, action_stamp_ns);
      image = find_nearest_locked(image_samples_, action_stamp_ns);
      joint_state = find_nearest_locked(joint_state_samples_, action_stamp_ns);
    }

    if (!image || !joint_state) {
      ++sync_miss_count_;
      return;
    }

    if (!timestamps_within_window({action_stamp_ns, image->stamp_ns, joint_state->stamp_ns}, sync_slop_ns_)) {
      ++sync_window_reject_count_;
      return;
    }

    Frame frame;
    frame.stamp_ns = action_stamp_ns;
    frame.image = std::move(image);
    frame.joint_state = std::move(joint_state);
    frame.action = std::move(action);

    if (!writer_.enqueue(std::move(frame))) {
      ++writer_reject_count_;
      return;
    }

    ++frames_enqueued_count_;
  }

  void log_stats()
  {
    std::size_t image_queue_size = 0;
    std::size_t joint_queue_size = 0;
    {
      std::lock_guard<std::mutex> lock(samples_mutex_);
      image_queue_size = image_samples_.size();
      joint_queue_size = joint_state_samples_.size();
    }

    RCLCPP_INFO(
      get_logger(),
      "frames=%lu sync_miss=%lu image_queue=%zu joint_queue=%zu image_drop=%lu joint_drop=%lu writer_reject=%lu",
      static_cast<unsigned long>(frames_enqueued_count_.load()),
      static_cast<unsigned long>(sync_miss_count_.load()),
      image_queue_size,
      joint_queue_size,
      static_cast<unsigned long>(image_queue_drop_count_.load()),
      static_cast<unsigned long>(joint_queue_drop_count_.load()),
      static_cast<unsigned long>(writer_reject_count_.load()));
    RCLCPP_INFO(
      get_logger(),
      "sync_window_reject=%lu sync_slop_ms=%d",
      static_cast<unsigned long>(sync_window_reject_count_.load()),
      config_.sync_slop_ms);
  }

  CollectorConfig config_;
  const std::int64_t sync_slop_ns_;
  const std::int64_t stale_window_ns_;
  Hdf5Writer writer_;

  std::mutex samples_mutex_;
  std::deque<std::shared_ptr<const ImageSample>> image_samples_;
  std::deque<std::shared_ptr<const JointStateSample>> joint_state_samples_;

  std::string resolved_image_topic_;
  std::string resolved_joint_state_topic_;
  std::string resolved_action_topic_;

  rclcpp::CallbackGroup::SharedPtr subscription_group_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr action_subscription_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  std::atomic<std::uint64_t> frames_enqueued_count_{0};
  std::atomic<std::uint64_t> sync_miss_count_{0};
  std::atomic<std::uint64_t> image_queue_drop_count_{0};
  std::atomic<std::uint64_t> joint_queue_drop_count_{0};
  std::atomic<std::uint64_t> writer_reject_count_{0};
  std::atomic<std::uint64_t> sync_window_reject_count_{0};
};

}  // namespace ros2_robot_data_collector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ros2_robot_data_collector::CollectorNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}