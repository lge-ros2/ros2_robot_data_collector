#include <gtest/gtest.h>

#include <stdexcept>

#include "ros2_robot_data_collector/sync_policy.hpp"

namespace ros2_robot_data_collector
{

TEST(SyncPolicyTest, ComputesTimestampSpan)
{
  EXPECT_EQ(timestamp_span_ns({100, 120, 105}), 20);
}

TEST(SyncPolicyTest, AcceptsTimestampsWithinWindow)
{
  EXPECT_TRUE(timestamps_within_window({1000, 1010, 1020}, 20));
}

TEST(SyncPolicyTest, RejectsTimestampsOutsideWindow)
{
  EXPECT_FALSE(timestamps_within_window({1000, 1010, 1035}, 20));
}

TEST(SyncPolicyTest, RejectsNegativeWindow)
{
  EXPECT_THROW(timestamps_within_window({1000, 1001}, -1), std::invalid_argument);
}

TEST(SyncPolicyTest, RejectsEmptyTimestampList)
{
  EXPECT_THROW(timestamp_span_ns({}), std::invalid_argument);
}

}  // namespace ros2_robot_data_collector