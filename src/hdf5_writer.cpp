#include "ros2_robot_data_collector/hdf5_writer.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <sstream>
#include <utility>

#include "rclcpp/rclcpp.hpp"

namespace ros2_robot_data_collector
{

namespace
{

std::string join_strings(const std::vector<std::string> & values)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0U) {
      stream << ',';
    }
    stream << values[index];
  }
  return stream.str();
}

H5::DSetCreatPropList make_creation_properties(
  const int rank,
  const hsize_t * chunk_dimensions,
  const bool enable_compression)
{
  H5::DSetCreatPropList properties;
  properties.setChunk(rank, chunk_dimensions);
  if (enable_compression) {
    properties.setDeflate(1);
  }
  return properties;
}

}  // namespace

Hdf5Writer::Hdf5Writer(const CollectorConfig & config, const rclcpp::Logger & logger)
: config_(config), logger_(logger), last_flush_time_(std::chrono::steady_clock::now())
{
  H5::Exception::dontPrint();
  worker_thread_ = std::thread(&Hdf5Writer::writer_loop, this);
}

Hdf5Writer::~Hdf5Writer()
{
  stop();
}

bool Hdf5Writer::enqueue(Frame frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopping_) {
    return false;
  }

  if (pending_frames_.size() >= config_.writer_queue_depth) {
    pending_frames_.pop_front();
    ++dropped_frame_count_;
  }

  pending_frames_.push_back(std::move(frame));
  queue_cv_.notify_one();
  return true;
}

void Hdf5Writer::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) {
      return;
    }
    stopping_ = true;
  }

  queue_cv_.notify_one();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void Hdf5Writer::writer_loop()
{
  const auto flush_interval = std::chrono::milliseconds(config_.flush_interval_ms);

  while (true) {
    std::vector<Frame> batch;
    bool force_flush = false;

    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (pending_frames_.empty() && !stopping_) {
        const auto wait_status = queue_cv_.wait_for(lock, flush_interval);
        force_flush = (wait_status == std::cv_status::timeout);
      }

      while (!pending_frames_.empty() && batch.size() < config_.batch_size) {
        batch.push_back(std::move(pending_frames_.front()));
        pending_frames_.pop_front();
      }

      if (stopping_ && pending_frames_.empty() && batch.empty()) {
        break;
      }
    }

    if (!batch.empty()) {
      flush_batch(std::move(batch), force_flush);
      continue;
    }

    flush_file(force_flush);
  }

  flush_file(true);

  if (file_) {
    auto meta_group = file_->openGroup("/meta");
    write_uint64_attribute(meta_group, "dropped_frames", static_cast<std::uint64_t>(dropped_frame_count_));
    file_->flush(H5F_SCOPE_LOCAL);
  }
}

void Hdf5Writer::initialize_file(const Frame & frame)
{
  if (schema_.initialized) {
    return;
  }

  if (!frame.image || !frame.joint_state) {
    throw std::runtime_error("Cannot initialize HDF5 schema without a complete frame.");
  }

  schema_.image_bytes = frame.image->data.size();
  schema_.joint_count = frame.joint_state->positions.size();
  schema_.action_count = frame.action.values.size();
  schema_.image_height = frame.image->height;
  schema_.image_width = frame.image->width;
  schema_.image_step = frame.image->step;
  schema_.image_is_bigendian = frame.image->is_bigendian;
  schema_.image_encoding = frame.image->encoding;
  schema_.joint_names_csv = join_strings(frame.joint_state->names);
  schema_.action_layout = frame.action.layout;
  schema_.action_labels_csv = join_strings(frame.action.labels);

  if (schema_.image_bytes == 0U) {
    throw std::runtime_error("Image frames must contain pixel data.");
  }
  if (schema_.joint_count == 0U) {
    throw std::runtime_error("Joint state frames must contain at least one value.");
  }
  if (schema_.action_count == 0U) {
    throw std::runtime_error("Action frames must contain at least one value.");
  }

  file_ = std::make_unique<H5::H5File>(config_.output_path, H5F_ACC_TRUNC);
  file_->createGroup("/meta");
  file_->createGroup("/episodes");
  file_->createGroup("/observations");
  file_->createGroup("/observations/images");
  file_->createGroup("/observations/joint_state");
  file_->createGroup("/actions");

  auto meta_group = file_->openGroup("/meta");
  write_string_attribute(meta_group, "schema_version", "0.1.0");
  write_string_attribute(meta_group, "namespace_prefix", config_.namespace_prefix);
  write_string_attribute(meta_group, "image_topic", config_.image_topic);
  write_string_attribute(meta_group, "joint_state_topic", config_.joint_state_topic);
  write_string_attribute(meta_group, "action_topic", config_.action_topic);
  write_string_attribute(meta_group, "joint_action_topic", config_.joint_action_topic);
  write_uint64_attribute(meta_group, "batch_size", static_cast<std::uint64_t>(config_.batch_size));
  write_uint64_attribute(meta_group, "sync_slop_ms", static_cast<std::uint64_t>(config_.sync_slop_ms));

  auto image_group = file_->openGroup("/observations/images");
  write_string_attribute(image_group, "encoding", schema_.image_encoding);
  write_uint32_attribute(image_group, "height", schema_.image_height);
  write_uint32_attribute(image_group, "width", schema_.image_width);
  write_uint32_attribute(image_group, "step", schema_.image_step);
  write_uint8_attribute(image_group, "is_bigendian", schema_.image_is_bigendian);

  auto joint_group = file_->openGroup("/observations/joint_state");
  write_string_attribute(joint_group, "joint_names_csv", schema_.joint_names_csv);

  auto action_group = file_->openGroup("/actions");
  write_string_attribute(action_group, "layout", schema_.action_layout);
  write_string_attribute(action_group, "labels_csv", schema_.action_labels_csv);

  const hsize_t scalar_initial_dimensions[1] = {0};
  const hsize_t scalar_max_dimensions[1] = {H5S_UNLIMITED};
  const hsize_t scalar_chunk_dimensions[1] = {
    static_cast<hsize_t>(std::max<std::size_t>(1U, config_.batch_size))};
  const H5::DataSpace scalar_dataspace(1, scalar_initial_dimensions, scalar_max_dimensions);
  const H5::DSetCreatPropList scalar_properties = make_creation_properties(
    1, scalar_chunk_dimensions, false);
  timestamps_dataset_ = std::make_unique<H5::DataSet>(file_->createDataSet(
      "/timestamps",
      H5::PredType::NATIVE_INT64,
      scalar_dataspace,
      scalar_properties));
  episode_index_dataset_ = std::make_unique<H5::DataSet>(file_->createDataSet(
      "/episodes/index",
      H5::PredType::NATIVE_UINT64,
      scalar_dataspace,
      scalar_properties));

  const hsize_t image_initial_dimensions[2] = {0, static_cast<hsize_t>(schema_.image_bytes)};
  const hsize_t image_max_dimensions[2] = {H5S_UNLIMITED, static_cast<hsize_t>(schema_.image_bytes)};
  const hsize_t image_chunk_dimensions[2] = {
    static_cast<hsize_t>(std::max<std::size_t>(1U, config_.batch_size)),
    static_cast<hsize_t>(schema_.image_bytes)};
  const H5::DataSpace image_dataspace(2, image_initial_dimensions, image_max_dimensions);
  const H5::DSetCreatPropList image_properties = make_creation_properties(
    2, image_chunk_dimensions, config_.enable_compression);
  image_dataset_ = std::make_unique<H5::DataSet>(file_->createDataSet(
      "/observations/images/data",
      H5::PredType::NATIVE_UINT8,
      image_dataspace,
      image_properties));

  const hsize_t joint_initial_dimensions[2] = {0, static_cast<hsize_t>(schema_.joint_count)};
  const hsize_t joint_max_dimensions[2] = {H5S_UNLIMITED, static_cast<hsize_t>(schema_.joint_count)};
  const hsize_t joint_chunk_dimensions[2] = {
    static_cast<hsize_t>(std::max<std::size_t>(1U, config_.batch_size)),
    static_cast<hsize_t>(schema_.joint_count)};
  const H5::DataSpace joint_dataspace(2, joint_initial_dimensions, joint_max_dimensions);
  const H5::DSetCreatPropList joint_properties = make_creation_properties(2, joint_chunk_dimensions, false);
  joint_positions_dataset_ = std::make_unique<H5::DataSet>(file_->createDataSet(
      "/observations/joint_state/positions",
      H5::PredType::NATIVE_DOUBLE,
      joint_dataspace,
      joint_properties));
  joint_velocities_dataset_ = std::make_unique<H5::DataSet>(file_->createDataSet(
      "/observations/joint_state/velocities",
      H5::PredType::NATIVE_DOUBLE,
      joint_dataspace,
      joint_properties));
  joint_efforts_dataset_ = std::make_unique<H5::DataSet>(file_->createDataSet(
      "/observations/joint_state/efforts",
      H5::PredType::NATIVE_DOUBLE,
      joint_dataspace,
      joint_properties));

  const hsize_t action_initial_dimensions[2] = {0, static_cast<hsize_t>(schema_.action_count)};
  const hsize_t action_max_dimensions[2] = {H5S_UNLIMITED, static_cast<hsize_t>(schema_.action_count)};
  const hsize_t action_chunk_dimensions[2] = {
    static_cast<hsize_t>(std::max<std::size_t>(1U, config_.batch_size)),
    static_cast<hsize_t>(schema_.action_count)};
  const H5::DataSpace action_dataspace(2, action_initial_dimensions, action_max_dimensions);
  const H5::DSetCreatPropList action_properties = make_creation_properties(2, action_chunk_dimensions, false);
  action_dataset_ = std::make_unique<H5::DataSet>(file_->createDataSet(
      "/actions/data",
      H5::PredType::NATIVE_DOUBLE,
      action_dataspace,
      action_properties));

  schema_.initialized = true;
}

void Hdf5Writer::flush_batch(std::vector<Frame> frames, const bool force_flush)
{
  if (frames.empty()) {
    flush_file(force_flush);
    return;
  }

  try {
    if (!schema_.initialized) {
      initialize_file(frames.front());
    }

    std::vector<Frame> accepted_frames;
    accepted_frames.reserve(frames.size());
    for (auto & frame : frames) {
      if (validate_frame(frame)) {
        accepted_frames.push_back(std::move(frame));
      } else {
        ++dropped_frame_count_;
      }
    }

    if (accepted_frames.empty()) {
      flush_file(force_flush);
      return;
    }

    std::vector<std::int64_t> timestamps;
    std::vector<std::uint64_t> episode_indices;
    std::vector<std::uint8_t> images;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_efforts;
    std::vector<double> actions;

    timestamps.reserve(accepted_frames.size());
    episode_indices.reserve(accepted_frames.size());
    images.reserve(accepted_frames.size() * schema_.image_bytes);
    joint_positions.reserve(accepted_frames.size() * schema_.joint_count);
    joint_velocities.reserve(accepted_frames.size() * schema_.joint_count);
    joint_efforts.reserve(accepted_frames.size() * schema_.joint_count);
    actions.reserve(accepted_frames.size() * schema_.action_count);

    for (const auto & frame : accepted_frames) {
      episode_indices.push_back(frame.episode_index);
      timestamps.push_back(frame.stamp_ns);
      images.insert(images.end(), frame.image->data.begin(), frame.image->data.end());
      joint_positions.insert(
        joint_positions.end(), frame.joint_state->positions.begin(), frame.joint_state->positions.end());
      joint_velocities.insert(
        joint_velocities.end(), frame.joint_state->velocities.begin(), frame.joint_state->velocities.end());
      joint_efforts.insert(
        joint_efforts.end(), frame.joint_state->efforts.begin(), frame.joint_state->efforts.end());
      actions.insert(actions.end(), frame.action.values.begin(), frame.action.values.end());
    }

    append_1d_u64(*episode_index_dataset_, episode_indices);
    append_1d_i64(*timestamps_dataset_, timestamps);
    append_2d_u8(*image_dataset_, images, accepted_frames.size(), schema_.image_bytes);
    append_2d_f64(*joint_positions_dataset_, joint_positions, accepted_frames.size(), schema_.joint_count);
    append_2d_f64(*joint_velocities_dataset_, joint_velocities, accepted_frames.size(), schema_.joint_count);
    append_2d_f64(*joint_efforts_dataset_, joint_efforts, accepted_frames.size(), schema_.joint_count);
    append_2d_f64(*action_dataset_, actions, accepted_frames.size(), schema_.action_count);

    flush_file(force_flush);
  } catch (const H5::Exception & exception) {
    RCLCPP_ERROR(logger_, "Failed to write HDF5 batch: %s", exception.getDetailMsg().c_str());
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(logger_, "Failed to write batch: %s", exception.what());
  }
}

bool Hdf5Writer::validate_frame(const Frame & frame) const
{
  if (!frame.image || !frame.joint_state) {
    return false;
  }

  return frame.image->data.size() == schema_.image_bytes &&
         frame.image->height == schema_.image_height &&
         frame.image->width == schema_.image_width &&
         frame.image->step == schema_.image_step &&
         frame.image->encoding == schema_.image_encoding &&
         frame.joint_state->positions.size() == schema_.joint_count &&
         frame.joint_state->velocities.size() == schema_.joint_count &&
         frame.joint_state->efforts.size() == schema_.joint_count &&
      frame.action.values.size() == schema_.action_count &&
      frame.action.layout == schema_.action_layout &&
      join_strings(frame.action.labels) == schema_.action_labels_csv;
}

void Hdf5Writer::flush_file(const bool force_flush)
{
  if (!file_) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (!force_flush && (now - last_flush_time_) < std::chrono::milliseconds(config_.flush_interval_ms)) {
    return;
  }

  file_->flush(H5F_SCOPE_LOCAL);
  last_flush_time_ = now;
}

void Hdf5Writer::write_string_attribute(
  H5::H5Object & object,
  const std::string & name,
  const std::string & value)
{
  const std::size_t string_length = std::max<std::size_t>(1U, value.size() + 1U);
  const H5::StrType string_type(H5::PredType::C_S1, string_length);
  const H5::DataSpace scalar_space(H5S_SCALAR);
  H5::Attribute attribute = object.createAttribute(name, string_type, scalar_space);
  attribute.write(string_type, value.c_str());
}

void Hdf5Writer::write_uint64_attribute(
  H5::H5Object & object,
  const std::string & name,
  const std::uint64_t value)
{
  const H5::DataSpace scalar_space(H5S_SCALAR);
  H5::Attribute attribute = object.createAttribute(name, H5::PredType::NATIVE_UINT64, scalar_space);
  attribute.write(H5::PredType::NATIVE_UINT64, &value);
}

void Hdf5Writer::write_uint32_attribute(
  H5::H5Object & object,
  const std::string & name,
  const std::uint32_t value)
{
  const H5::DataSpace scalar_space(H5S_SCALAR);
  H5::Attribute attribute = object.createAttribute(name, H5::PredType::NATIVE_UINT32, scalar_space);
  attribute.write(H5::PredType::NATIVE_UINT32, &value);
}

void Hdf5Writer::write_uint8_attribute(
  H5::H5Object & object,
  const std::string & name,
  const std::uint8_t value)
{
  const H5::DataSpace scalar_space(H5S_SCALAR);
  H5::Attribute attribute = object.createAttribute(name, H5::PredType::NATIVE_UINT8, scalar_space);
  attribute.write(H5::PredType::NATIVE_UINT8, &value);
}

void Hdf5Writer::append_1d_i64(H5::DataSet & dataset, const std::vector<std::int64_t> & values)
{
  if (values.empty()) {
    return;
  }

  hsize_t previous_dimensions[1] = {0};
  H5::DataSpace file_space = dataset.getSpace();
  file_space.getSimpleExtentDims(previous_dimensions);

  const hsize_t append_count[1] = {static_cast<hsize_t>(values.size())};
  const hsize_t new_dimensions[1] = {previous_dimensions[0] + append_count[0]};
  dataset.extend(new_dimensions);

  file_space = dataset.getSpace();
  file_space.selectHyperslab(H5S_SELECT_SET, append_count, previous_dimensions);
  const H5::DataSpace memory_space(1, append_count);
  dataset.write(values.data(), H5::PredType::NATIVE_INT64, memory_space, file_space);
}

void Hdf5Writer::append_1d_u64(H5::DataSet & dataset, const std::vector<std::uint64_t> & values)
{
  if (values.empty()) {
    return;
  }

  hsize_t previous_dimensions[1] = {0};
  H5::DataSpace file_space = dataset.getSpace();
  file_space.getSimpleExtentDims(previous_dimensions);

  const hsize_t append_count[1] = {static_cast<hsize_t>(values.size())};
  const hsize_t new_dimensions[1] = {previous_dimensions[0] + append_count[0]};
  dataset.extend(new_dimensions);

  file_space = dataset.getSpace();
  file_space.selectHyperslab(H5S_SELECT_SET, append_count, previous_dimensions);
  const H5::DataSpace memory_space(1, append_count);
  dataset.write(values.data(), H5::PredType::NATIVE_UINT64, memory_space, file_space);
}

void Hdf5Writer::append_2d_f64(
  H5::DataSet & dataset,
  const std::vector<double> & values,
  const std::size_t rows,
  const std::size_t columns)
{
  if (values.empty()) {
    return;
  }

  hsize_t previous_dimensions[2] = {0, 0};
  H5::DataSpace file_space = dataset.getSpace();
  file_space.getSimpleExtentDims(previous_dimensions);

  const hsize_t append_count[2] = {
    static_cast<hsize_t>(rows),
    static_cast<hsize_t>(columns)};
  const hsize_t offset[2] = {previous_dimensions[0], 0};
  const hsize_t new_dimensions[2] = {
    previous_dimensions[0] + append_count[0],
    previous_dimensions[1]};
  dataset.extend(new_dimensions);

  file_space = dataset.getSpace();
  file_space.selectHyperslab(H5S_SELECT_SET, append_count, offset);
  const H5::DataSpace memory_space(2, append_count);
  dataset.write(values.data(), H5::PredType::NATIVE_DOUBLE, memory_space, file_space);
}

void Hdf5Writer::append_2d_u8(
  H5::DataSet & dataset,
  const std::vector<std::uint8_t> & values,
  const std::size_t rows,
  const std::size_t columns)
{
  if (values.empty()) {
    return;
  }

  hsize_t previous_dimensions[2] = {0, 0};
  H5::DataSpace file_space = dataset.getSpace();
  file_space.getSimpleExtentDims(previous_dimensions);

  const hsize_t append_count[2] = {
    static_cast<hsize_t>(rows),
    static_cast<hsize_t>(columns)};
  const hsize_t offset[2] = {previous_dimensions[0], 0};
  const hsize_t new_dimensions[2] = {
    previous_dimensions[0] + append_count[0],
    previous_dimensions[1]};
  dataset.extend(new_dimensions);

  file_space = dataset.getSpace();
  file_space.selectHyperslab(H5S_SELECT_SET, append_count, offset);
  const H5::DataSpace memory_space(2, append_count);
  dataset.write(values.data(), H5::PredType::NATIVE_UINT8, memory_space, file_space);
}

}  // namespace ros2_robot_data_collector