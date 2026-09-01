#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "rubi_heightmap_step_wavefront_planner/heightmap_snapshot.hpp"
#include "rubi_heightmap_step_wavefront_planner/map/costmap_snapshot.hpp"

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
  kCostmapOutOfBounds,
  kCostmapUnknown,
  kCostmapCollision,
  kInsufficientHeightEvidence,
  kHeightEvidenceGap,
  kIsolatedNode,
  kTrgCollision,
};

enum class StepEvaluationMode {kHeightOnlyStrict, kCostmapHeightHybrid};

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
  double inflation_cost_weight{5.0};
  double inflation_cost_exponent{1.0};
  double node_evidence_radius_m{0.10};
  std::size_t node_min_observed_cells{3U};
  double node_max_nearest_evidence_distance_m{0.075};
  double node_height_outlier_threshold_m{0.08};
  double node_max_height_outlier_ratio{0.30};
  double edge_height_query_radius_m{0.075};
  double edge_max_height_evidence_gap_m{0.10};
};

struct NodeEvaluation
{
  bool valid{false};
  StepInvalidReason reason{StepInvalidReason::kInvalidInput};
  double elevation_m{0.0};
  double observed_support_ratio{0.0};
  double max_clearance_height_jump_m{0.0};
  double minimum_clearance_m{0.0};
  std::uint8_t raw_cost{0U};
  bool height_evidence_available{false};
};

struct EdgeEvaluation
{
  bool valid{false};
  StepInvalidReason reason{StepInvalidReason::kInvalidInput};
  std::size_t sample_count{0U};
  std::size_t unique_cell_count{0U};
  double length_xy_m{0.0};
  double max_height_jump_m{0.0};
  double max_clearance_height_jump_m{0.0};
  double observed_support_ratio{0.0};
  std::size_t height_jump_event_count{0U};
  double height_jump_score_m{0.0};
  double minimum_clearance_m{0.0};
  double clearance_score_m{0.0};
  double inflation_score_m{0.0};
  std::uint8_t maximum_raw_cost{0U};
  std::size_t height_evidence_missing_samples{0U};
  double cost{0.0};
};

struct EvaluationInstrumentation
{
  std::size_t costmap_queries{0U};
  std::size_t height_evidence_queries{0U};
  std::size_t edge_samples_total{0U};
};

class StepEvaluator
{
public:
  StepEvaluator(
    const HeightmapSnapshot & snapshot,
    StepEvaluatorParameters parameters);
  StepEvaluator(
    const HeightmapSnapshot & heightmap,
    const CostmapSnapshot & costmap,
    StepEvaluatorParameters parameters);

  const HeightmapSnapshot & snapshot() const noexcept {return snapshot_;}
  const StepEvaluatorParameters & parameters() const noexcept {return parameters_;}
  StepEvaluationMode mode() const noexcept {return mode_;}
  const CostmapSnapshot * costmap() const noexcept {return costmap_;}
  const EvaluationInstrumentation & instrumentation() const noexcept {return instrumentation_;}
  NodeEvaluation evaluateNode(Point2D point) const;
  EdgeEvaluation evaluateEdge(Point2D from, Point2D to) const;
  std::vector<GridCell> supercover(Point2D from, Point2D to) const;

private:
  NodeEvaluation evaluateClearance(GridCell center) const;
  double nearestHazardDistance(GridCell center) const;
  NodeEvaluation evaluateHybridNode(Point2D point) const;
  EdgeEvaluation evaluateHybridEdge(Point2D from, Point2D to) const;

  const HeightmapSnapshot & snapshot_;
  const CostmapSnapshot * costmap_{nullptr};
  StepEvaluationMode mode_{StepEvaluationMode::kHeightOnlyStrict};
  StepEvaluatorParameters parameters_;
  // One evaluator is request-local. This mutable memoization is therefore not
  // shared across planning threads or map generations.
  mutable std::unordered_map<std::size_t, double> clearance_cache_;
  mutable EvaluationInstrumentation instrumentation_;
};

}  // namespace rubi_heightmap_step_wavefront_planner
