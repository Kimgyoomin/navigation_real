#include <gtest/gtest.h>
#include "rubi_heightmap_step_wavefront_planner/ros/nav2_costmap_adapter.hpp"
namespace planner = rubi_heightmap_step_wavefront_planner;

TEST(Nav2CostmapAdapter, ConvertsFullRawSnapshotAndRejectsRotation)
{
  nav2_msgs::msg::Costmap message;
  message.metadata.size_x = 2U;
  message.metadata.size_y = 2U;
  message.metadata.resolution = 0.05F;
  message.metadata.origin.position.x = -1.0;
  message.metadata.origin.position.y = 2.0;
  message.metadata.origin.orientation.w = 1.0;
  message.data = {0U, 252U, 253U, 255U};
  const auto snapshot = planner::Nav2CostmapAdapter{}.makeSnapshot(message);
  EXPECT_EQ(*snapshot.cost({1, 1}), 255U);
  message.metadata.origin.orientation.z = 0.1;
  EXPECT_THROW(planner::Nav2CostmapAdapter{}.makeSnapshot(message), std::invalid_argument);
}

TEST(Nav2CostmapAdapter, RejectsPartialOrMalformedData)
{
  nav2_msgs::msg::Costmap message;
  message.metadata.size_x = 2U;
  message.metadata.size_y = 2U;
  message.metadata.resolution = 0.05F;
  message.metadata.origin.orientation.w = 1.0;
  message.data = {0U};
  EXPECT_THROW(planner::Nav2CostmapAdapter{}.makeSnapshot(message), std::invalid_argument);
}
