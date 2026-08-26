#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include <gtest/gtest.h>
#include "rubi_heightmap_step_wavefront_planner/graph/spatial_index_2d.hpp"

namespace planner = rubi_heightmap_step_wavefront_planner;

TEST(SpatialIndex2D, EmptyBoundaryDuplicateAndRandomizedGroundTruth)
{
  planner::UniformGridSpatialIndex2D index(0.25);
  EXPECT_FALSE(index.nearest({0.0, 0.0}));
  EXPECT_TRUE(index.radiusSearch({0.0, 0.0}, 1.0).empty());
  std::mt19937 generator(42U);
  std::uniform_real_distribution<double> coordinate(-5.0, 5.0);
  std::vector<planner::Point2D> points(300U);
  for (std::size_t id = 0U; id < points.size(); ++id) {
    points[id] = {coordinate(generator), coordinate(generator)};
    index.insert(id, points[id]);
  }
  index.insert(7U, {0.25, 0.0});
  points[7U] = {0.25, 0.0};
  for (std::size_t query_index = 0U; query_index < 100U; ++query_index) {
    const planner::Point2D query{coordinate(generator), coordinate(generator)};
    const double radius_m = 0.75;
    std::vector<std::pair<double, planner::NodeId>> expected;
    for (std::size_t id = 0U; id < points.size(); ++id) {
      const double distance_squared = std::pow(points[id].x - query.x, 2) +
        std::pow(points[id].y - query.y, 2);
      if (distance_squared <= radius_m * radius_m + 1.0e-12) {
        expected.emplace_back(distance_squared, id);
      }
    }
    std::sort(expected.begin(), expected.end());
    std::vector<planner::NodeId> ids;
    for (const auto & value : expected) {ids.push_back(value.second);}
    EXPECT_EQ(index.radiusSearch(query, radius_m), ids);
    ASSERT_TRUE(index.nearest(query));
    std::pair<double, planner::NodeId> nearest{
      std::numeric_limits<double>::infinity(), 0U};
    for (std::size_t id = 0U; id < points.size(); ++id) {
      nearest = std::min(nearest, std::make_pair(
        std::pow(points[id].x - query.x, 2) + std::pow(points[id].y - query.y, 2), id));
    }
    EXPECT_EQ(*index.nearest(query), nearest.second);
  }
  index.clear();
  index.insert(7U, {0.25, 0.0});
  EXPECT_EQ(index.radiusSearch({0.0, 0.0}, 0.25),
    (std::vector<planner::NodeId>{7U}));
}
