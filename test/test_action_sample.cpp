#include <cmath>

#include <gtest/gtest.h>

#include "control_msgs/msg/joint_jog.hpp"

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

}  // namespace ros2_robot_data_collector