#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

#include "H5Cpp.h"
#include "rclcpp/rclcpp.hpp"

#include "ros2_robot_data_collector/hdf5_writer.hpp"

namespace ros2_robot_data_collector
{

namespace
{

std::filesystem::path make_output_path(const std::string & stem)
{
  const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         (stem + "_" + std::to_string(suffix) + ".h5");
}

CollectorConfig make_config(const std::filesystem::path & output_path)
{
  CollectorConfig config;
  config.namespace_prefix = "";
  config.image_topic = "/test/image";
  config.joint_state_topic = "/test/joint_states";
  config.lidar_topic = "";
  config.action_topic = "/test/actions";
  config.joint_action_topic = "";
  config.cmd_vel_action_topic = "";
  config.output_path = output_path.string();
  config.topic_queue_depth = 8U;
  config.writer_queue_depth = 8U;
  config.batch_size = 1U;
  config.sync_slop_ms = 50;
  config.flush_interval_ms = 1;
  config.approximate_sync = true;
  config.enable_compression = false;
  return config;
}

std::shared_ptr<ImageSample> make_image_sample_for_test()
{
  auto image = std::make_shared<ImageSample>();
  image->stamp_ns = 100;
  image->height = 1U;
  image->width = 2U;
  image->step = 2U;
  image->is_bigendian = 0U;
  image->encoding = "mono8";
  image->data = {1U, 2U};
  return image;
}

std::shared_ptr<JointStateSample> make_joint_state_sample_for_test(
  const std::vector<std::string> & names,
  const std::int64_t stamp_ns)
{
  auto joint_state = std::make_shared<JointStateSample>();
  joint_state->stamp_ns = stamp_ns;
  joint_state->names = names;
  joint_state->positions = {1.0, 2.0};
  joint_state->velocities = {3.0, 4.0};
  joint_state->efforts = {5.0, 6.0};
  return joint_state;
}

ActionSample make_action_sample_for_test(const std::int64_t stamp_ns)
{
  ActionSample action;
  action.stamp_ns = stamp_ns;
  action.values = {0.1, 0.2};
  action.labels = {"joint_a/position", "joint_b/position"};
  action.layout = "float64_multi_array";
  return action;
}

Frame make_frame_for_test(
  const std::vector<std::string> & joint_names,
  const std::int64_t stamp_ns)
{
  Frame frame;
  frame.episode_index = 1U;
  frame.stamp_ns = stamp_ns;
  frame.image = make_image_sample_for_test();
  frame.joint_state = make_joint_state_sample_for_test(joint_names, stamp_ns);
  frame.action = make_action_sample_for_test(stamp_ns);
  return frame;
}

std::size_t dataset_row_count(
  const std::filesystem::path & output_path,
  const std::string & dataset_name)
{
  H5::H5File file(output_path.string(), H5F_ACC_RDONLY);
  H5::DataSet dataset = file.openDataSet(dataset_name);
  H5::DataSpace dataspace = dataset.getSpace();
  hsize_t dimensions[1] = {0};
  dataspace.getSimpleExtentDims(dimensions);
  return static_cast<std::size_t>(dimensions[0]);
}

class Hdf5WriterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

}  // namespace

TEST_F(Hdf5WriterTest, RejectsJointStateFramesWithChangedNameOrder)
{
  const auto output_path = make_output_path("joint_name_order");
  std::error_code error_code;
  std::filesystem::remove(output_path, error_code);

  {
    Hdf5Writer writer(make_config(output_path), rclcpp::get_logger("test_hdf5_writer_order"));
    ASSERT_TRUE(writer.enqueue(make_frame_for_test({"joint_a", "joint_b"}, 100)));
    ASSERT_TRUE(writer.enqueue(make_frame_for_test({"joint_b", "joint_a"}, 200)));
    writer.stop();
  }

  ASSERT_TRUE(std::filesystem::exists(output_path));
  EXPECT_EQ(dataset_row_count(output_path, "/timestamps"), 1U);

  std::filesystem::remove(output_path, error_code);
}

TEST_F(Hdf5WriterTest, RefusesToInitializeSchemaWithoutNamedJoints)
{
  const auto output_path = make_output_path("joint_name_missing");
  std::error_code error_code;
  std::filesystem::remove(output_path, error_code);

  {
    Hdf5Writer writer(make_config(output_path), rclcpp::get_logger("test_hdf5_writer_missing"));
    ASSERT_TRUE(writer.enqueue(make_frame_for_test({"joint_a", ""}, 100)));
    writer.stop();
  }

  EXPECT_FALSE(std::filesystem::exists(output_path));
}

}  // namespace ros2_robot_data_collector
