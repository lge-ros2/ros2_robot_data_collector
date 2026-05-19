#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "H5Cpp.h"
#include "rclcpp/logger.hpp"

#include "ros2_robot_data_collector/collector_config.hpp"
#include "ros2_robot_data_collector/sample.hpp"

namespace ros2_robot_data_collector
{

class Hdf5Writer
{
public:
  Hdf5Writer(const CollectorConfig & config, const rclcpp::Logger & logger);
  ~Hdf5Writer();

  Hdf5Writer(const Hdf5Writer &) = delete;
  Hdf5Writer & operator=(const Hdf5Writer &) = delete;

  bool enqueue(Frame frame);
  void stop();

private:
  struct SchemaDescription
  {
    bool initialized = false;
    std::size_t image_bytes = 0;
    std::size_t joint_count = 0;
    std::size_t action_count = 0;
    std::uint32_t image_height = 0;
    std::uint32_t image_width = 0;
    std::uint32_t image_step = 0;
    std::uint8_t image_is_bigendian = 0;
    std::string image_encoding;
    std::string joint_names_csv;
  };

  void writer_loop();
  void initialize_file(const Frame & frame);
  void flush_batch(std::vector<Frame> frames, bool force_flush);
  bool validate_frame(const Frame & frame) const;
  void flush_file(bool force_flush);
  void write_string_attribute(H5::H5Object & object, const std::string & name, const std::string & value);
  void write_uint64_attribute(H5::H5Object & object, const std::string & name, std::uint64_t value);
  void write_uint32_attribute(H5::H5Object & object, const std::string & name, std::uint32_t value);
  void write_uint8_attribute(H5::H5Object & object, const std::string & name, std::uint8_t value);
  void append_1d_i64(H5::DataSet & dataset, const std::vector<std::int64_t> & values);
  void append_2d_f64(
    H5::DataSet & dataset,
    const std::vector<double> & values,
    std::size_t rows,
    std::size_t columns);
  void append_2d_u8(
    H5::DataSet & dataset,
    const std::vector<std::uint8_t> & values,
    std::size_t rows,
    std::size_t columns);

  CollectorConfig config_;
  rclcpp::Logger logger_;
  SchemaDescription schema_;
  std::mutex mutex_;
  std::condition_variable queue_cv_;
  std::deque<Frame> pending_frames_;
  std::thread worker_thread_;
  bool stopping_ = false;
  std::size_t dropped_frame_count_ = 0;
  std::chrono::steady_clock::time_point last_flush_time_;

  std::unique_ptr<H5::H5File> file_;
  std::unique_ptr<H5::DataSet> timestamps_dataset_;
  std::unique_ptr<H5::DataSet> image_dataset_;
  std::unique_ptr<H5::DataSet> joint_positions_dataset_;
  std::unique_ptr<H5::DataSet> joint_velocities_dataset_;
  std::unique_ptr<H5::DataSet> joint_efforts_dataset_;
  std::unique_ptr<H5::DataSet> action_dataset_;
};

}  // namespace ros2_robot_data_collector