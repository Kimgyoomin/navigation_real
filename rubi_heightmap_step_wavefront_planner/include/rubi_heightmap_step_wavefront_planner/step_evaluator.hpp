#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "rubi_heightmap_step_wavefront_planner/heightmap_snapshot.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

enum class StepInvalidReason
{
  kNone,
  kOutOfBounds,
  kUnknown,
  kInsufficientClearanceSupport,
  kClearanceViolation,
  kStepLimit,
  kInvalidInput,
};

std::string_view toString(StepInvalidReason reason) noexcept;

struct StepEvaluatorParameters
{
  double hard_clearance_radius_m{0.20};
  double edge_check_spacing_m{0.025};
  double max_crossable_height_jump_m{0.08};
  double height_noise_floor_m{0.01};
  double height_cost_exponent{2.0};
  double distance_weight{1.0};
  double height_cost_weight{5.0};
  double preferred_clearance_radius_m{0.20};
  double clearance_cost_weight{0.0};
  double clearance_cost_exponent{2.0};
};

struct NodeEvaluation
{
  bool valid{false};
  StepInvalidReason reason{StepInvalidReason::kInvalidInput};
  double elevation_m{0.0};
  double observed_support_ratio{0.0};
  double max_clearance_height_jump_m{0.0};
  double minimum_clearance_m{0.0};
};

struct EdgeEvaluation
{
  bool valid{false};
  StepInvalidReason reason{StepInvalidReason::kInvalidInput};
  std::size_t sample_count{0U};
  std::size_t unique_cell_count{0U};
  double length_xy_m{0.0};
  double max_height_jump_m{0.0};
  std::size_t height_jump_event_count{0U};
  double height_jump_score_m{0.0};
  double minimum_clearance_m{0.0};
  double clearance_score_m{0.0};
  double cost{0.0};
};

class StepEvaluator
{
public:
  StepEvaluator(
    const HeightmapSnapshot & snapshot,
    StepEvaluatorParameters parameters);

  const HeightmapSnapshot & snapshot() const noexcept {return snapshot_;}
  const StepEvaluatorParameters & parameters() const noexcept {return parameters_;}
  NodeEvaluation evaluateNode(Point2D point) const;
  EdgeEvaluation evaluateEdge(Point2D from, Point2D to) const;
  std::vector<GridCell> supercover(Point2D from, Point2D to) const;

private:
  NodeEvaluation evaluateClearance(GridCell center) const;
  double nearestHazardDistance(GridCell center) const;

  const HeightmapSnapshot & snapshot_;
  StepEvaluatorParameters parameters_;
  // One evaluator is request-local. This mutable memoization is therefore not
  // shared across planning threads or map generations.
  mutable std::unordered_map<std::size_t, double> clearance_cache_;
};

}  // namespace rubi_heightmap_step_wavefront_planner
