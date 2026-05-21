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

#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

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

const char * display_topic(const std::string & topic_name)
{
  return topic_name.empty() ? "<disabled>" : topic_name.c_str();
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
    resolved_lidar_topic_ = resolve_topic_name(config_.namespace_prefix, config_.lidar_topic);

    if (!config_.action_topic.empty()) {
      resolved_action_topic_ = resolve_topic_name(config_.namespace_prefix, config_.action_topic);
      action_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        resolved_action_topic_,
        action_qos,
        std::bind(&CollectorNode::on_numeric_action, this, std::placeholders::_1),
        subscription_options);
    }

    if (!config_.joint_action_topic.empty()) {
      resolved_joint_action_topic_ = resolve_topic_name(config_.namespace_prefix, config_.joint_action_topic);
      joint_action_subscription_ = create_subscription<control_msgs::msg::JointJog>(
        resolved_joint_action_topic_,
        action_qos,
        std::bind(&CollectorNode::on_joint_action, this, std::placeholders::_1),
        subscription_options);
    }

    if (!config_.cmd_vel_action_topic.empty()) {
      resolved_cmd_vel_action_topic_ = resolve_topic_name(
        config_.namespace_prefix, config_.cmd_vel_action_topic);
      cmd_vel_action_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        resolved_cmd_vel_action_topic_,
        action_qos,
        std::bind(&CollectorNode::on_cmd_vel_action, this, std::placeholders::_1),
        subscription_options);
    }

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

    if (!config_.lidar_topic.empty()) {
      lidar_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        resolved_lidar_topic_,
        sensor_qos,
        std::bind(&CollectorNode::on_lidar, this, std::placeholders::_1),
        subscription_options);
    }

    stats_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&CollectorNode::log_stats, this));

    start_episode_service_ = create_service<std_srvs::srv::Trigger>(
      "collector_node/start_episode",
      std::bind(
        &CollectorNode::on_start_episode,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));

    end_episode_service_ = create_service<std_srvs::srv::Trigger>(
      "collector_node/end_episode",
      std::bind(
        &CollectorNode::on_end_episode,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));

    RCLCPP_INFO(
      get_logger(),
      "Collector configured with namespace_prefix='%s', image='%s', joint_state='%s', lidar='%s', action='%s', joint_action='%s', cmd_vel_action='%s', output='%s', start_episode='/collector_node/start_episode', end_episode='/collector_node/end_episode'",
      config_.namespace_prefix.c_str(),
      resolved_image_topic_.c_str(),
      resolved_joint_state_topic_.c_str(),
      display_topic(resolved_lidar_topic_),
      display_topic(resolved_action_topic_),
      display_topic(resolved_joint_action_topic_),
      display_topic(resolved_cmd_vel_action_topic_),
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

  void on_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
  {
    std::shared_ptr<const LidarScanSample> sample = make_lidar_scan_sample(*message);

    std::lock_guard<std::mutex> lock(samples_mutex_);
    push_bounded_locked(lidar_samples_, std::move(sample), lidar_queue_drop_count_);
  }

  void on_numeric_action(const std_msgs::msg::Float64MultiArray::ConstSharedPtr message)
  {
    const std::int64_t action_stamp_ns = get_clock()->now().nanoseconds();
    process_action(make_action_sample(*message, action_stamp_ns));
  }

  void on_joint_action(const control_msgs::msg::JointJog::ConstSharedPtr message)
  {
    const std::int64_t action_stamp_ns = get_clock()->now().nanoseconds();
    process_action(make_action_sample(*message, action_stamp_ns));
  }

  void on_cmd_vel_action(const geometry_msgs::msg::Twist::ConstSharedPtr message)
  {
    const std::int64_t action_stamp_ns = get_clock()->now().nanoseconds();
    process_action(make_action_sample(*message, action_stamp_ns));
  }

  void process_action(ActionSample action)
  {
    if (action.values.empty()) {
      ++writer_reject_count_;
      return;
    }

    const std::int64_t action_stamp_ns = action.stamp_ns;
    std::uint64_t episode_index = 0;
    {
      std::lock_guard<std::mutex> episode_lock(episode_mutex_);
      if (!episode_active_) {
        ++episode_skip_count_;
        return;
      }
      episode_index = current_episode_index_;
    }

    std::shared_ptr<const ImageSample> image;
    std::shared_ptr<const JointStateSample> joint_state;
    std::shared_ptr<const LidarScanSample> lidar;
    {
      std::lock_guard<std::mutex> lock(samples_mutex_);
      prune_stale_queue_locked(image_samples_, action_stamp_ns);
      prune_stale_queue_locked(joint_state_samples_, action_stamp_ns);
      if (!config_.lidar_topic.empty()) {
        prune_stale_queue_locked(lidar_samples_, action_stamp_ns);
      }
      image = find_nearest_locked(image_samples_, action_stamp_ns);
      joint_state = find_nearest_locked(joint_state_samples_, action_stamp_ns);
      if (!config_.lidar_topic.empty()) {
        lidar = find_nearest_locked(lidar_samples_, action_stamp_ns);
      }
    }

    if (!image || !joint_state || (!config_.lidar_topic.empty() && !lidar)) {
      ++sync_miss_count_;
      return;
    }

    std::vector<std::int64_t> synchronized_timestamps = {
      action_stamp_ns,
      image->stamp_ns,
      joint_state->stamp_ns};
    if (lidar) {
      synchronized_timestamps.push_back(lidar->stamp_ns);
    }

    if (!timestamps_within_window(synchronized_timestamps, sync_slop_ns_)) {
      ++sync_window_reject_count_;
      return;
    }

    Frame frame;
    frame.episode_index = episode_index;
    frame.stamp_ns = action_stamp_ns;
    frame.image = std::move(image);
    frame.joint_state = std::move(joint_state);
    frame.lidar = std::move(lidar);
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
    std::size_t lidar_queue_size = 0;
    {
      std::lock_guard<std::mutex> lock(samples_mutex_);
      image_queue_size = image_samples_.size();
      joint_queue_size = joint_state_samples_.size();
      lidar_queue_size = lidar_samples_.size();
    }

    RCLCPP_INFO(
      get_logger(),
      "frames=%lu sync_miss=%lu image_queue=%zu joint_queue=%zu lidar_queue=%zu image_drop=%lu joint_drop=%lu lidar_drop=%lu writer_reject=%lu episode_skip=%lu",
      static_cast<unsigned long>(frames_enqueued_count_.load()),
      static_cast<unsigned long>(sync_miss_count_.load()),
      image_queue_size,
      joint_queue_size,
      lidar_queue_size,
      static_cast<unsigned long>(image_queue_drop_count_.load()),
      static_cast<unsigned long>(joint_queue_drop_count_.load()),
      static_cast<unsigned long>(lidar_queue_drop_count_.load()),
      static_cast<unsigned long>(writer_reject_count_.load()),
      static_cast<unsigned long>(episode_skip_count_.load()));
    RCLCPP_INFO(
      get_logger(),
      "sync_window_reject=%lu sync_slop_ms=%d",
      static_cast<unsigned long>(sync_window_reject_count_.load()),
      config_.sync_slop_ms);
  }

  void on_start_episode(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(episode_mutex_);
    if (episode_active_) {
      response->success = false;
      response->message = "Episode already active: " + std::to_string(current_episode_index_);
      return;
    }

    ++current_episode_index_;
    episode_active_ = true;
    response->success = true;
    response->message = "Started episode " + std::to_string(current_episode_index_);
  }

  void on_end_episode(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(episode_mutex_);
    if (!episode_active_) {
      response->success = false;
      response->message = "No active episode.";
      return;
    }

    episode_active_ = false;
    response->success = true;
    response->message = "Ended episode " + std::to_string(current_episode_index_);
  }

  CollectorConfig config_;
  const std::int64_t sync_slop_ns_;
  const std::int64_t stale_window_ns_;
  Hdf5Writer writer_;

  std::mutex samples_mutex_;
  std::mutex episode_mutex_;
  std::deque<std::shared_ptr<const ImageSample>> image_samples_;
  std::deque<std::shared_ptr<const JointStateSample>> joint_state_samples_;
  std::deque<std::shared_ptr<const LidarScanSample>> lidar_samples_;
  std::uint64_t current_episode_index_ = 1;
  bool episode_active_ = true;

  std::string resolved_image_topic_;
  std::string resolved_joint_state_topic_;
  std::string resolved_lidar_topic_;
  std::string resolved_action_topic_;
  std::string resolved_joint_action_topic_;
  std::string resolved_cmd_vel_action_topic_;

  rclcpp::CallbackGroup::SharedPtr subscription_group_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr action_subscription_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr joint_action_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_action_subscription_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_episode_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr end_episode_service_;

  std::atomic<std::uint64_t> frames_enqueued_count_{0};
  std::atomic<std::uint64_t> sync_miss_count_{0};
  std::atomic<std::uint64_t> image_queue_drop_count_{0};
  std::atomic<std::uint64_t> joint_queue_drop_count_{0};
  std::atomic<std::uint64_t> lidar_queue_drop_count_{0};
  std::atomic<std::uint64_t> writer_reject_count_{0};
  std::atomic<std::uint64_t> sync_window_reject_count_{0};
  std::atomic<std::uint64_t> episode_skip_count_{0};
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