#include <cmath>

#include <gtest/gtest.h>

#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "ros2_robot_data_collector/sample.hpp"

namespace ros2_robot_data_collector
{

TEST(ActionSampleTest, ConvertsJointJogToDisplacementVelocityLayout)
{
  control_msgs::msg::JointJog message;
  message.joint_names = {"joint_a", "joint_b"};
  message.displacements = {1.0, 2.0};
  message.velocities = {3.0, 4.0};

  const ActionSample sample = make_action_sample(message, 1234);

  EXPECT_EQ(sample.stamp_ns, 1234);
  EXPECT_EQ(sample.layout, "joint_jog_displacements_then_velocities");
  ASSERT_EQ(sample.values.size(), 4U);
  EXPECT_DOUBLE_EQ(sample.values[0], 1.0);
  EXPECT_DOUBLE_EQ(sample.values[1], 2.0);
  EXPECT_DOUBLE_EQ(sample.values[2], 3.0);
  EXPECT_DOUBLE_EQ(sample.values[3], 4.0);
  ASSERT_EQ(sample.labels.size(), 4U);
  EXPECT_EQ(sample.labels[0], "joint_a/displacement");
  EXPECT_EQ(sample.labels[1], "joint_b/displacement");
  EXPECT_EQ(sample.labels[2], "joint_a/velocity");
  EXPECT_EQ(sample.labels[3], "joint_b/velocity");
}

TEST(ActionSampleTest, PadsMissingJointJogFieldsWithNan)
{
  control_msgs::msg::JointJog message;
  message.joint_names = {"joint_a", "joint_b"};
  message.displacements = {1.0};

  const ActionSample sample = make_action_sample(message, 99);

  ASSERT_EQ(sample.values.size(), 4U);
  EXPECT_DOUBLE_EQ(sample.values[0], 1.0);
  EXPECT_TRUE(std::isnan(sample.values[1]));
  EXPECT_TRUE(std::isnan(sample.values[2]));
  EXPECT_TRUE(std::isnan(sample.values[3]));
  ASSERT_EQ(sample.labels.size(), 4U);
  EXPECT_EQ(sample.labels[1], "joint_b/displacement");
  EXPECT_EQ(sample.labels[3], "joint_b/velocity");
}

TEST(ActionSampleTest, ConvertsTwistToLinearThenAngularLayout)
{
  geometry_msgs::msg::Twist message;
  message.linear.x = 0.1;
  message.linear.y = 0.2;
  message.linear.z = 0.3;
  message.angular.x = 1.1;
  message.angular.y = 1.2;
  message.angular.z = 1.3;

  const ActionSample sample = make_action_sample(message, 55);

  EXPECT_EQ(sample.stamp_ns, 55);
  EXPECT_EQ(sample.layout, "twist_linear_then_angular");
  ASSERT_EQ(sample.values.size(), 6U);
  EXPECT_DOUBLE_EQ(sample.values[0], 0.1);
  EXPECT_DOUBLE_EQ(sample.values[1], 0.2);
  EXPECT_DOUBLE_EQ(sample.values[2], 0.3);
  EXPECT_DOUBLE_EQ(sample.values[3], 1.1);
  EXPECT_DOUBLE_EQ(sample.values[4], 1.2);
  EXPECT_DOUBLE_EQ(sample.values[5], 1.3);
  ASSERT_EQ(sample.labels.size(), 6U);
  EXPECT_EQ(sample.labels[0], "linear/x");
  EXPECT_EQ(sample.labels[5], "angular/z");
}

TEST(ActionSampleTest, NormalizesLaserScanIntensitiesToRangeCount)
{
  sensor_msgs::msg::LaserScan message;
  message.header.stamp.sec = 12;
  message.header.stamp.nanosec = 34;
  message.angle_min = -1.57F;
  message.angle_max = 1.57F;
  message.angle_increment = 0.01F;
  message.time_increment = 0.001F;
  message.scan_time = 0.1F;
  message.range_min = 0.05F;
  message.range_max = 20.0F;
  message.ranges = {1.0F, 2.0F, 3.0F};
  message.intensities = {10.0F};

  const std::shared_ptr<LidarScanSample> sample = make_lidar_scan_sample(message);

  ASSERT_NE(sample, nullptr);
  EXPECT_EQ(sample->stamp_ns, 12000000034LL);
  ASSERT_EQ(sample->ranges.size(), 3U);
  ASSERT_EQ(sample->intensities.size(), 3U);
  EXPECT_DOUBLE_EQ(sample->ranges[2], 3.0);
  EXPECT_DOUBLE_EQ(sample->intensities[0], 10.0);
  EXPECT_TRUE(std::isnan(sample->intensities[1]));
  EXPECT_TRUE(std::isnan(sample->intensities[2]));
  EXPECT_DOUBLE_EQ(sample->range_max, 20.0);
}

}  // namespace ros2_robot_data_collector