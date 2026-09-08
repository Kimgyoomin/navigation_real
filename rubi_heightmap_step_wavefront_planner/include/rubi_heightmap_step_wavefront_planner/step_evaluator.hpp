#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
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

  // Experimental FastDEM-robust edge evidence. The Sobel response is normalized
  // by 8*resolution. For an ideal straight step on a uniform grid,
  // 2*resolution*|grad Z| equals the physical step height. On smeared maps this
  // is only a local two-cell height-change heuristic, not a reconstructed height.
  // This feature branch intentionally enables the 10 cm hard threshold so the
  // original refactor branch is the baseline and this branch is the A/B variant.
  bool sobel_hard_reject_enabled{true};
  double sobel_equivalent_step_height_m{0.10};
  double sobel_cost_weight{0.0};
  double sobel_cost_exponent{2.0};
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
  double max_sobel_gradient{0.0};
  double max_sobel_equivalent_step_height_m{0.0};
  std::size_t sobel_valid_cell_count{0U};
  std::size_t sobel_missing_cell_count{0U};
  double sobel_gradient_score_m{0.0};
  bool sobel_hard_rejection{false};
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
  std::size_t sobel_queries{0U};
  std::size_t sobel_missing_neighborhoods{0U};
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
  std::optional<double> sobelGradientMagnitude(GridCell center) const;
  void accumulateSobelEvidence(GridCell cell, EdgeEvaluation & result) const;

  const HeightmapSnapshot & snapshot_;
  const CostmapSnapshot * costmap_{nullptr};
  StepEvaluationMode mode_{StepEvaluationMode::kHeightOnlyStrict};
  StepEvaluatorParameters parameters_;
  mutable std::unordered_map<std::size_t, double> clearance_cache_;
  mutable std::unordered_map<std::size_t, double> sobel_gradient_cache_;
  mutable EvaluationInstrumentation instrumentation_;
};

}  // namespace rubi_heightmap_step_wavefront_planner
