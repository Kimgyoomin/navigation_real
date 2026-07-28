#include "pongbot_local_graph_insertion_planner/local_overlay.hpp"
#include "pongbot_local_graph_insertion_planner/path_post_processor.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace p = pongbot_local_graph_insertion_planner;
namespace
{

p::GridSnapshot grid(std::size_t width, std::size_t height)
{
  p::GridSnapshot result;
  result.frame_id = "map";
  result.size_x = width;
  result.size_y = height;
  result.resolution = 1.0;
  result.costs.assign(width * height, 0);
  return result;
}

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.position.x = x;
  result.pose.position.y = y;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  result.pose.orientation = tf2::toMsg(quaternion);
  return result;
}

double quaternionNorm(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::sqrt(
    quaternion.x * quaternion.x +
    quaternion.y * quaternion.y +
    quaternion.z * quaternion.z +
    quaternion.w * quaternion.w);
}

}  // namespace

TEST(LocalOverlay, UnknownAndFreeDoNotEraseGlobalBase)
{
  auto fused = grid(3, 1);
  fused.costs[0] = 253;
  fused.costs[1] = 100;

  p::LocalGrid local;
  local.size_x = 3;
  local.size_y = 1;
  local.resolution = 1.0;
  local.costs = {0, 255, 0};

  const auto result = p::applyLocalOverlay(fused, local, {});

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.overlay_cells, 0u);
  EXPECT_EQ(fused.costs[0], 253);
  EXPECT_EQ(fused.costs[1], 100);
}

TEST(LocalOverlay, SoftAndBlockedCostsUseMaxAggregation)
{
  auto fused = grid(3, 1);
  fused.costs[0] = 200;

  p::LocalGrid local;
  local.size_x = 3;
  local.size_y = 1;
  local.resolution = 1.0;
  local.costs = {100, 252, 253};

  const auto result = p::applyLocalOverlay(fused, local, {});

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.overlay_cells, 2u);
  EXPECT_EQ(fused.costs[0], 200);
  EXPECT_EQ(fused.costs[1], 252);
  EXPECT_EQ(fused.costs[2], 253);
}

TEST(LocalOverlay, ObstacleRemovalRestoresFreshGlobalSnapshot)
{
  const auto global_base = grid(2, 1);
  p::LocalGrid local{2, 1, 1.0, {0, 253}};

  auto inserted = global_base;
  ASSERT_TRUE(p::applyLocalOverlay(inserted, local, {}).success);
  EXPECT_EQ(inserted.costs[1], 253);

  local.costs[1] = 0;
  auto removed = global_base;
  ASSERT_TRUE(p::applyLocalOverlay(removed, local, {}).success);
  EXPECT_EQ(removed.costs[1], 0);
}

TEST(LocalOverlay, RotatedProjectionStaysInsideBounds)
{
  auto fused = grid(5, 5);
  fused.origin_x = -2.0;
  fused.origin_y = -2.0;
  p::LocalGrid local{1, 1, 1.0, {253}};
  const p::Transform2D transform{0.0, 0.0, 0.7853981633974483};

  const auto result = p::applyLocalOverlay(fused, local, transform);

  ASSERT_TRUE(result.success);
  EXPECT_GT(result.overlay_cells, 0u);
  EXPECT_LE(result.overlay_cells, fused.costs.size());
  EXPECT_EQ(
    std::count(fused.costs.begin(), fused.costs.end(), 253),
    static_cast<long>(result.overlay_cells));
}

TEST(PathPostProcessor, StartEqualsGoalPreservesGoalOrientation)
{
  auto snapshot = grid(1, 1);
  p::SearchResult search;
  search.status = p::SearchStatus::kSuccess;
  search.path = {0};

  auto start = pose(0.25, 0.25, 0.0);
  auto goal = pose(0.25, 0.25, 1.25);
  // Exercise normalization without changing the represented yaw.
  goal.pose.orientation.x *= 2.0;
  goal.pose.orientation.y *= 2.0;
  goal.pose.orientation.z *= 2.0;
  goal.pose.orientation.w *= 2.0;

  const auto path = p::buildMetricPath(snapshot, search, start, goal, rclcpp::Time(10, 0));

  ASSERT_EQ(path.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, start.pose.position.x);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, goal.pose.position.x);
  EXPECT_NEAR(tf2::getYaw(path.poses.back().pose.orientation), 1.25, 1e-12);
  EXPECT_NEAR(quaternionNorm(path.poses.front().pose.orientation), 1.0, 1e-12);
  EXPECT_NEAR(quaternionNorm(path.poses.back().pose.orientation), 1.0, 1e-12);
}

TEST(PathPostProcessor, ExactEndpointsAndNormalizedIntermediateYaw)
{
  auto snapshot = grid(3, 2);
  p::SearchResult search;
  search.status = p::SearchStatus::kSuccess;
  search.path = {snapshot.index(0, 0), snapshot.index(1, 0), snapshot.index(2, 1)};
  const auto start = pose(0.1, 0.2, 0.3);
  const auto goal = pose(2.8, 1.7, -0.7);

  const auto path = p::buildMetricPath(snapshot, search, start, goal, rclcpp::Time(10, 0));

  ASSERT_EQ(path.poses.size(), 3u);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, start.pose.position.x);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.y, start.pose.position.y);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, goal.pose.position.x);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.y, goal.pose.position.y);
  EXPECT_NEAR(tf2::getYaw(path.poses.back().pose.orientation), -0.7, 1e-12);
  for (const auto & path_pose : path.poses) {
    EXPECT_NEAR(quaternionNorm(path_pose.pose.orientation), 1.0, 1e-12);
  }
}
