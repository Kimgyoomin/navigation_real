#pragma once

#include <cstddef>
#include <limits>
#include <string_view>

#include "rubi_heightmap_wavefront_planner/terrain_snapshot.hpp"

namespace rubi_heightmap_wavefront_planner
{

enum class TerrainInvalidReason
{
  kNone,
  kOutOfBounds,
  kUnknown,
  kInsufficientFootprintSupport,
  kInsufficientPcaSupport,
  kSlopeLimit,
  kRoughnessLimit,
  kStepLimit,
  kInvalidInput,
};

std::string_view toString(TerrainInvalidReason reason) noexcept;

/**
 * @brief Terrain validity and cost parameters.
 *
 * Defaults are software bootstrap values only. They are not validated RUBI
 * kinematic or stability limits.
 */
struct TerrainEvaluatorParameters
{
  double pca_radius_m{0.30};
  std::size_t min_pca_points{4};

  double footprint_radius_m{0.30};
  double min_footprint_observed_ratio{1.00};

  double max_slope_deg{15.0};
  double max_roughness_m{std::numeric_limits<double>::infinity()};
  double max_step_height_m{0.08};

  double edge_sample_spacing_m{0.05};
  double slope_cost_weight{1.0};
  bool check_footprint_along_edge{true};
};

struct SurfaceMetrics
{
  bool valid{false};
  std::size_t sample_count{0};
  double slope_deg{std::numeric_limits<double>::quiet_NaN()};
  double roughness_m{std::numeric_limits<double>::quiet_NaN()};
  double normal_x{std::numeric_limits<double>::quiet_NaN()};
  double normal_y{std::numeric_limits<double>::quiet_NaN()};
  double normal_z{std::numeric_limits<double>::quiet_NaN()};
};

struct NodeEvaluation
{
  bool valid{false};
  TerrainInvalidReason reason{TerrainInvalidReason::kInvalidInput};
  double elevation_m{std::numeric_limits<double>::quiet_NaN()};
  double footprint_observed_ratio{0.0};
  double max_footprint_step_m{0.0};
  SurfaceMetrics surface{};
};

struct EdgeEvaluation
{
  bool valid{false};
  TerrainInvalidReason reason{TerrainInvalidReason::kInvalidInput};
  std::size_t sample_count{0};
  double length_xy_m{0.0};
  double length_3d_m{0.0};
  double max_step_m{0.0};
  double max_slope_deg{0.0};
  double mean_slope_deg{0.0};
  double max_roughness_m{0.0};
  double min_footprint_observed_ratio{1.0};
  double cost{std::numeric_limits<double>::infinity()};
};

/**
 * @brief ROS-independent terrain validity evaluator for sparse graph planners.
 */
class TerrainEvaluator
{
public:
  TerrainEvaluator(
    const TerrainSnapshot & snapshot,
    TerrainEvaluatorParameters parameters = {});

  const TerrainSnapshot & snapshot() const noexcept;
  const TerrainEvaluatorParameters & parameters() const noexcept;

  SurfaceMetrics localSurface(Point2D position) const;
  NodeEvaluation evaluateNode(Point2D position) const;
  EdgeEvaluation evaluateEdge(Point2D from, Point2D to) const;

private:
  struct FootprintMetrics
  {
    std::size_t total_cells{0};
    std::size_t observed_cells{0};
    double observed_ratio{0.0};
    double max_adjacent_step_m{0.0};
  };

  FootprintMetrics footprintSupport(Point2D position) const;
  NodeEvaluation evaluatePosition(Point2D position, bool check_footprint) const;

  const TerrainSnapshot & snapshot_;
  TerrainEvaluatorParameters parameters_;
};

}  // namespace rubi_heightmap_wavefront_planner
