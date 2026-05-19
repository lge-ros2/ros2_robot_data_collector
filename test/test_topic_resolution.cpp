#include <gtest/gtest.h>

#include "ros2_robot_data_collector/topic_resolver.hpp"

namespace ros2_robot_data_collector
{

TEST(TopicResolverTest, LeavesAbsoluteTopicsUntouched)
{
  EXPECT_EQ(resolve_topic_name("/robot1", "/tf"), "/tf");
}

TEST(TopicResolverTest, ExpandsRelativeTopicsUnderNamespace)
{
  EXPECT_EQ(resolve_topic_name("robot1", "camera/front/image_raw"), "/robot1/camera/front/image_raw");
}

TEST(TopicResolverTest, ResolvesRelativeTopicsWithoutNamespace)
{
  EXPECT_EQ(resolve_topic_name("", "joint_states"), "/joint_states");
}

TEST(TopicResolverTest, RejectsEmptyTopicNames)
{
  EXPECT_THROW(resolve_topic_name("/robot1", ""), std::invalid_argument);
}

}  // namespace ros2_robot_data_collector