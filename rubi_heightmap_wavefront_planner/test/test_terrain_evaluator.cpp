#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

#include "rubi_heightmap_wavefront_planner/terrain_evaluator.hpp"
#include "rubi_heightmap_wavefront_planner/terrain_snapshot.hpp"

namespace rubi_heightmap_wavefront_planner
{
namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;

std::vector<TerrainPoint> makePlane(
  std::size_t size_x,
  std::size_t size_y,
  double resolution,
  double x_min,
  double y_min,
  double dz_dx,
  double dz_dy)
{
  std::vector<TerrainPoint> points;
  points.reserve(size_x * size_y);
  for (std::size_t iy = 0U; iy < size_y; ++iy) {
    for (std::size_t ix = 0U; ix < size_x; ++ix) {
      const double x = x_min + static_cast<double>(ix) * resolution;
      const double y = y_min + static_cast<double>(iy) * resolution;
      points.push_back({x, y, dz_dx * x + dz_dy * y});
    }
  }
  return points;
}

TerrainEvaluatorParameters permissiveParameters()
{
  TerrainEvaluatorParameters parameters;
  parameters.pca_radius_m = 0.22;
  parameters.min_pca_points = 5U;
  parameters.footprint_radius_m = 0.0;
  parameters.min_footprint_observed_ratio = 1.0;
  parameters.max_slope_deg = 89.0;
  parameters.max_roughness_m = std::numeric_limits<double>::infinity();
  parameters.max_step_height_m = 1.0;
  parameters.edge_sample_spacing_m = 0.05;
  parameters.slope_cost_weight = 1.0;
  parameters.check_footprint_along_edge = true;
  return parameters;
}

TEST(TerrainSnapshot, ReconstructsSparseLatticeWithoutNearestNeighborFallback)
{
  std::vector<TerrainPoint> points{
    {0.0, 0.0, 0.0},
    {0.1, 0.0, 0.1},
    {0.2, 0.0, 0.2},
    {0.0, 0.1, 1.0},
    // (0.1, 0.1) is deliberately unknown.
    {0.2, 0.1, 1.2},
    {0.0, 0.2, 2.0},
    {0.1, 0.2, 2.1},
    {0.2, 0.2, 2.2},
  };
  const TerrainSnapshot snapshot = TerrainSnapshot::fromPoints(points, 0.1, 1.0e-6);

  EXPECT_EQ(snapshot.sizeX(), 3U);
  EXPECT_EQ(snapshot.sizeY(), 3U);
  EXPECT_EQ(snapshot.observedCount(), 8U);
  EXPECT_FALSE(snapshot.isObserved(1U, 1U));
  EXPECT_FALSE(snapshot.query(0.1, 0.1).has_value());
  EXPECT_FALSE(snapshot.elevationAt(0.149, 0.1).has_value());

  const auto neighboring_cell = snapshot.query(0.151, 0.1);
  ASSERT_TRUE(neighboring_cell.has_value());
  EXPECT_EQ(neighboring_cell->index, (GridIndex{2U, 1U}));
  EXPECT_DOUBLE_EQ(neighboring_cell->elevation_m, 1.2);
}

TEST(TerrainSnapshot, RejectsOffLatticeAndDuplicateCells)
{
  EXPECT_THROW(
    TerrainSnapshot::fromPoints(
      {{0.0, 0.0, 0.0}, {0.101, 0.0, 0.0}}, 0.1, 1.0e-4),
    std::invalid_argument);

  EXPECT_THROW(
    TerrainSnapshot::fromPoints(
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.1}}, 0.1, 1.0e-6),
    std::invalid_argument);

  EXPECT_THROW(
    TerrainSnapshot::fromPoints(
      {{0.0, 0.0, 0.0}, {0.2, 0.2, 0.0}}, 0.1, 1.0e-6, 8U),
    std::invalid_argument);
}

TEST(TerrainEvaluator, LocalPcaRecoversAnalyticPlaneSlopeAndNearZeroRoughness)
{
  constexpr double dz_dx = 0.20;
  constexpr double dz_dy = -0.10;
  const TerrainSnapshot snapshot = TerrainSnapshot::fromPoints(
    makePlane(11U, 11U, 0.1, -0.5, -0.5, dz_dx, dz_dy), 0.1, 1.0e-6);

  auto parameters = permissiveParameters();
  parameters.pca_radius_m = 0.31;
  const TerrainEvaluator evaluator(snapshot, parameters);
  const SurfaceMetrics surface = evaluator.localSurface({0.0, 0.0});

  ASSERT_TRUE(surface.valid);
  const double expected_slope =
    std::atan(std::hypot(dz_dx, dz_dy)) * 180.0 / kPi;
  EXPECT_NEAR(surface.slope_deg, expected_slope, 1.0e-6);
  EXPECT_NEAR(surface.roughness_m, 0.0, 1.0e-8);
  EXPECT_GT(surface.normal_z, 0.0);
}

TEST(TerrainEvaluator, NodeRejectsIncompleteFootprintSupport)
{
  auto points = makePlane(7U, 7U, 0.1, -0.3, -0.3, 0.0, 0.0);
  for (auto iterator = points.begin(); iterator != points.end(); ++iterator) {
    if (std::abs(iterator->x - 0.1) < 1.0e-9 && std::abs(iterator->y) < 1.0e-9) {
      points.erase(iterator);
      break;
    }
  }
  const TerrainSnapshot snapshot = TerrainSnapshot::fromPoints(points, 0.1, 1.0e-6);

  auto parameters = permissiveParameters();
  parameters.footprint_radius_m = 0.15;
  parameters.min_footprint_observed_ratio = 1.0;
  const TerrainEvaluator evaluator(snapshot, parameters);
  const NodeEvaluation node = evaluator.evaluateNode({0.0, 0.0});

  EXPECT_FALSE(node.valid);
  EXPECT_EQ(node.reason, TerrainInvalidReason::kInsufficientFootprintSupport);
  EXPECT_LT(node.footprint_observed_ratio, 1.0);
}

TEST(TerrainEvaluator, EdgeRejectsUnknownCellInsteadOfBridgingIt)
{
  auto points = makePlane(11U, 7U, 0.1, 0.0, -0.3, 0.0, 0.0);
  for (auto iterator = points.begin(); iterator != points.end(); ++iterator) {
    if (std::abs(iterator->x - 0.5) < 1.0e-9 && std::abs(iterator->y) < 1.0e-9) {
      points.erase(iterator);
      break;
    }
  }
  const TerrainSnapshot snapshot = TerrainSnapshot::fromPoints(points, 0.1, 1.0e-6);
  const TerrainEvaluator evaluator(snapshot, permissiveParameters());

  const EdgeEvaluation edge = evaluator.evaluateEdge({0.1, 0.0}, {0.9, 0.0});
  EXPECT_FALSE(edge.valid);
  EXPECT_EQ(edge.reason, TerrainInvalidReason::kUnknown);
}

TEST(TerrainEvaluator, EdgeRejectsHeightDiscontinuityAsStep)
{
  std::vector<TerrainPoint> points;
  for (std::size_t iy = 0U; iy < 9U; ++iy) {
    for (std::size_t ix = 0U; ix < 11U; ++ix) {
      const double x = static_cast<double>(ix) * 0.1;
      const double y = -0.4 + static_cast<double>(iy) * 0.1;
      const double z = ix <= 4U ? 0.0 : 0.20;
      points.push_back({x, y, z});
    }
  }
  const TerrainSnapshot snapshot = TerrainSnapshot::fromPoints(points, 0.1, 1.0e-6);
  auto parameters = permissiveParameters();
  parameters.max_step_height_m = 0.08;
  const TerrainEvaluator evaluator(snapshot, parameters);

  const EdgeEvaluation edge = evaluator.evaluateEdge({0.1, 0.0}, {0.9, 0.0});
  EXPECT_FALSE(edge.valid);
  EXPECT_EQ(edge.reason, TerrainInvalidReason::kStepLimit);
  EXPECT_GT(edge.max_step_m, parameters.max_step_height_m);
}

TEST(TerrainEvaluator, EdgeRejectsStepInsideFootprintBesideFlatCenterline)
{
  std::vector<TerrainPoint> points;
  for (std::size_t iy = 0U; iy < 11U; ++iy) {
    for (std::size_t ix = 0U; ix < 11U; ++ix) {
      const double x = -0.5 + static_cast<double>(ix) * 0.1;
      const double y = -0.5 + static_cast<double>(iy) * 0.1;
      const bool raised_side_row = std::abs(std::abs(y) - 0.1) < 1.0e-9;
      points.push_back({x, y, raised_side_row ? 0.20 : 0.0});
    }
  }
  const TerrainSnapshot snapshot =
    TerrainSnapshot::fromPoints(points, 0.1, 1.0e-6);
  auto parameters = permissiveParameters();
  parameters.footprint_radius_m = 0.20;
  parameters.min_footprint_observed_ratio = 1.0;
  parameters.max_step_height_m = 0.08;
  const TerrainEvaluator evaluator(snapshot, parameters);

  const EdgeEvaluation edge =
    evaluator.evaluateEdge({-0.3, 0.0}, {0.3, 0.0});
  EXPECT_FALSE(edge.valid);
  EXPECT_EQ(edge.reason, TerrainInvalidReason::kStepLimit);
  EXPECT_GT(edge.max_step_m, parameters.max_step_height_m);
}

TEST(TerrainEvaluator, EdgeRejectsPcaSlopeAboveLimit)
{
  const double slope_deg = 20.0;
  const double dz_dx = std::tan(slope_deg * kPi / 180.0);
  const TerrainSnapshot snapshot = TerrainSnapshot::fromPoints(
    makePlane(13U, 9U, 0.1, 0.0, -0.4, dz_dx, 0.0), 0.1, 1.0e-6);
  auto parameters = permissiveParameters();
  parameters.max_slope_deg = 15.0;
  parameters.max_step_height_m = 1.0;
  const TerrainEvaluator evaluator(snapshot, parameters);

  const EdgeEvaluation edge = evaluator.evaluateEdge({0.2, 0.0}, {1.0, 0.0});
  EXPECT_FALSE(edge.valid);
  EXPECT_EQ(edge.reason, TerrainInvalidReason::kSlopeLimit);
}

TEST(TerrainEvaluator, FlatObservedEdgeIsValidWithMetricCost)
{
  const TerrainSnapshot snapshot = TerrainSnapshot::fromPoints(
    makePlane(13U, 9U, 0.1, 0.0, -0.4, 0.0, 0.0), 0.1, 1.0e-6);
  const TerrainEvaluator evaluator(snapshot, permissiveParameters());

  const EdgeEvaluation edge = evaluator.evaluateEdge({0.2, 0.0}, {1.0, 0.0});
  ASSERT_TRUE(edge.valid);
  EXPECT_EQ(edge.reason, TerrainInvalidReason::kNone);
  EXPECT_NEAR(edge.length_xy_m, 0.8, 1.0e-12);
  EXPECT_NEAR(edge.length_3d_m, 0.8, 1.0e-12);
  EXPECT_NEAR(edge.max_slope_deg, 0.0, 1.0e-8);
  EXPECT_NEAR(edge.cost, 0.8, 1.0e-8);
}

TEST(TerrainEvaluator, FiveCentimeterFlatGridSupportsHalfCellEdgeSampling)
{
  constexpr double resolution = 0.05;
  constexpr double edge_spacing = 0.025;
  const TerrainSnapshot snapshot = TerrainSnapshot::fromPoints(
    makePlane(29U, 17U, resolution, -0.2, -0.4, 0.0, 0.0),
    resolution, 1.0e-6);

  auto parameters = permissiveParameters();
  parameters.pca_radius_m = 0.16;
  parameters.edge_sample_spacing_m = edge_spacing;
  const TerrainEvaluator evaluator(snapshot, parameters);

  EXPECT_DOUBLE_EQ(snapshot.resolution(), resolution);
  EXPECT_DOUBLE_EQ(evaluator.parameters().edge_sample_spacing_m, edge_spacing);
  const EdgeEvaluation edge = evaluator.evaluateEdge({0.1, 0.0}, {0.9, 0.0});
  ASSERT_TRUE(edge.valid);
  EXPECT_EQ(edge.reason, TerrainInvalidReason::kNone);
  EXPECT_EQ(edge.sample_count, 33U);
  EXPECT_NEAR(edge.length_xy_m, 0.8, 1.0e-12);
  EXPECT_NEAR(edge.length_3d_m, 0.8, 1.0e-12);
  EXPECT_NEAR(edge.cost, 0.8, 1.0e-8);
}

TEST(TerrainEvaluator, FiveCentimeterGridRejectsSpacingAboveHalfCell)
{
  constexpr double resolution = 0.05;
  const TerrainSnapshot snapshot = TerrainSnapshot::fromPoints(
    makePlane(9U, 9U, resolution, -0.2, -0.2, 0.0, 0.0),
    resolution, 1.0e-6);
  auto parameters = permissiveParameters();
  parameters.edge_sample_spacing_m = 0.025001;

  EXPECT_THROW(TerrainEvaluator(snapshot, parameters), std::invalid_argument);
}

TEST(TerrainEvaluator, FiveCentimeterEdgeSamplingPreservesUnknownHoleGate)
{
  constexpr double resolution = 0.05;
  auto points = makePlane(
    29U, 17U, resolution, -0.2, -0.4, 0.0, 0.0);
  for (auto iterator = points.begin(); iterator != points.end(); ++iterator) {
    if (
      std::abs(iterator->x - 0.5) < 1.0e-9 &&
      std::abs(iterator->y) < 1.0e-9)
    {
      points.erase(iterator);
      break;
    }
  }
  const TerrainSnapshot snapshot =
    TerrainSnapshot::fromPoints(points, resolution, 1.0e-6);
  auto parameters = permissiveParameters();
  parameters.pca_radius_m = 0.16;
  parameters.edge_sample_spacing_m = 0.025;
  const TerrainEvaluator evaluator(snapshot, parameters);

  const EdgeEvaluation edge = evaluator.evaluateEdge({0.1, 0.0}, {0.9, 0.0});
  EXPECT_FALSE(edge.valid);
  EXPECT_EQ(edge.reason, TerrainInvalidReason::kUnknown);
}

TEST(TerrainEvaluator, FiveCentimeterEdgeSamplingPreservesStepGate)
{
  constexpr double resolution = 0.05;
  std::vector<TerrainPoint> points;
  points.reserve(29U * 17U);
  for (std::size_t iy = 0U; iy < 17U; ++iy) {
    for (std::size_t ix = 0U; ix < 29U; ++ix) {
      const double x = -0.2 + static_cast<double>(ix) * resolution;
      const double y = -0.4 + static_cast<double>(iy) * resolution;
      const double z = ix < 14U ? 0.0 : 0.20;
      points.push_back({x, y, z});
    }
  }
  const TerrainSnapshot snapshot =
    TerrainSnapshot::fromPoints(points, resolution, 1.0e-6);
  auto parameters = permissiveParameters();
  parameters.pca_radius_m = 0.16;
  parameters.max_step_height_m = 0.08;
  parameters.edge_sample_spacing_m = 0.025;
  const TerrainEvaluator evaluator(snapshot, parameters);

  const EdgeEvaluation edge = evaluator.evaluateEdge({0.1, 0.0}, {0.9, 0.0});
  EXPECT_FALSE(edge.valid);
  EXPECT_EQ(edge.reason, TerrainInvalidReason::kStepLimit);
  EXPECT_GT(edge.max_step_m, parameters.max_step_height_m);
}

TEST(TerrainEvaluator, RejectsInvalidThresholdConfiguration)
{
  const TerrainSnapshot snapshot = TerrainSnapshot::fromPoints(
    makePlane(3U, 3U, 0.1, 0.0, 0.0, 0.0, 0.0), 0.1, 1.0e-6);
  auto parameters = permissiveParameters();
  parameters.edge_sample_spacing_m = 0.0;
  EXPECT_THROW(TerrainEvaluator(snapshot, parameters), std::invalid_argument);

  parameters = permissiveParameters();
  parameters.edge_sample_spacing_m = 0.051;
  EXPECT_THROW(TerrainEvaluator(snapshot, parameters), std::invalid_argument);
}

}  // namespace
}  // namespace rubi_heightmap_wavefront_planner
