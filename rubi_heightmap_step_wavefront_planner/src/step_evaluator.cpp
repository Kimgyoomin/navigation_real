#include "rubi_heightmap_step_wavefront_planner/step_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "rubi_heightmap_step_wavefront_planner/terrain/height_evidence.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

std::string_view toString(const StepInvalidReason reason) noexcept
{
  switch (reason) {
    case StepInvalidReason::kNone: return "none";
    case StepInvalidReason::kOutOfBounds: return "out_of_bounds";
    case StepInvalidReason::kUnknown: return "unknown";
    case StepInvalidReason::kInsufficientClearanceSupport: return "insufficient_clearance_support";
    case StepInvalidReason::kClearanceViolation: return "clearance_violation";
    case StepInvalidReason::kStepLimit: return "step_limit";
    case StepInvalidReason::kInvalidInput: return "invalid_input";
    case StepInvalidReason::kCostmapOutOfBounds: return "costmap_out_of_bounds";
    case StepInvalidReason::kCostmapUnknown: return "costmap_unknown";
    case StepInvalidReason::kCostmapCollision: return "costmap_collision";
    case StepInvalidReason::kInsufficientHeightEvidence: return "insufficient_height_evidence";
    case StepInvalidReason::kHeightEvidenceGap: return "height_evidence_gap";
    case StepInvalidReason::kIsolatedNode: return "isolated_node";
    case StepInvalidReason::kTrgCollision: return "trg_collision";
  }
  return "invalid_input";
}

StepEvaluator::StepEvaluator(
  const HeightmapSnapshot & snapshot,
  const StepEvaluatorParameters parameters)
: snapshot_(snapshot), parameters_(parameters)
{
  if (!std::isfinite(parameters_.hard_clearance_radius_m) ||
    parameters_.hard_clearance_radius_m < 0.0 ||
    !std::isfinite(parameters_.edge_check_spacing_m) ||
    parameters_.edge_check_spacing_m <= 0.0 ||
    parameters_.edge_check_spacing_m > 0.5 * snapshot_.resolution() + 1.0e-12 ||
    !std::isfinite(parameters_.max_crossable_height_jump_m) ||
    parameters_.max_crossable_height_jump_m <= 0.0 ||
    !std::isfinite(parameters_.height_noise_floor_m) ||
    parameters_.height_noise_floor_m < 0.0 ||
    parameters_.height_noise_floor_m >= parameters_.max_crossable_height_jump_m ||
    !std::isfinite(parameters_.height_cost_exponent) ||
    parameters_.height_cost_exponent < 1.0 ||
    !std::isfinite(parameters_.distance_weight) || parameters_.distance_weight <= 0.0 ||
    !std::isfinite(parameters_.height_cost_weight) || parameters_.height_cost_weight < 0.0 ||
    !std::isfinite(parameters_.preferred_clearance_radius_m) ||
    parameters_.preferred_clearance_radius_m < parameters_.hard_clearance_radius_m ||
    !std::isfinite(parameters_.clearance_cost_weight) || parameters_.clearance_cost_weight < 0.0 ||
    !std::isfinite(parameters_.clearance_cost_exponent) || parameters_.clearance_cost_exponent < 1.0)
  {
    throw std::invalid_argument("invalid StepEvaluator parameters");
  }
}

StepEvaluator::StepEvaluator(
  const HeightmapSnapshot & heightmap, const CostmapSnapshot & costmap,
  const StepEvaluatorParameters parameters)
: StepEvaluator(heightmap, parameters)
{
  if (!std::isfinite(parameters_.inflation_cost_weight) ||
    parameters_.inflation_cost_weight < 0.0 ||
    !std::isfinite(parameters_.inflation_cost_exponent) ||
    parameters_.inflation_cost_exponent < 1.0 ||
    !std::isfinite(parameters_.node_evidence_radius_m) ||
    parameters_.node_evidence_radius_m < 0.0 ||
    parameters_.node_min_observed_cells == 0U ||
    !std::isfinite(parameters_.node_max_nearest_evidence_distance_m) ||
    parameters_.node_max_nearest_evidence_distance_m < 0.0 ||
    !std::isfinite(parameters_.node_height_outlier_threshold_m) ||
    parameters_.node_height_outlier_threshold_m < 0.0 ||
    !std::isfinite(parameters_.node_max_height_outlier_ratio) ||
    parameters_.node_max_height_outlier_ratio < 0.0 ||
    parameters_.node_max_height_outlier_ratio > 1.0 ||
    !std::isfinite(parameters_.edge_height_query_radius_m) ||
    parameters_.edge_height_query_radius_m < 0.0 ||
    !std::isfinite(parameters_.edge_max_height_evidence_gap_m) ||
    parameters_.edge_max_height_evidence_gap_m <= 0.0)
  {
    throw std::invalid_argument("invalid hybrid StepEvaluator parameters");
  }
  costmap_ = &costmap;
  mode_ = StepEvaluationMode::kCostmapHeightHybrid;
}

double StepEvaluator::nearestHazardDistance(const GridCell center) const
{
  const auto index = snapshot_.index(center);
  if (!index) {return 0.0;}
  const auto cached = clearance_cache_.find(*index);
  if (cached != clearance_cache_.end()) {return cached->second;}
  const double search_radius_m = parameters_.preferred_clearance_radius_m;
  const int radius_cells = static_cast<int>(std::ceil(search_radius_m / snapshot_.resolution()));
  double minimum_m = search_radius_m;
  const Point2D center_point = snapshot_.cellCenter(center);
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const GridCell cell{center.x + dx, center.y + dy};
      const double distance_m = snapshot_.resolution() * std::hypot(dx, dy);
      if (distance_m > minimum_m + 1.0e-12) {continue;}
      if (!snapshot_.inBounds(cell) || !snapshot_.observed(cell)) {
        minimum_m = distance_m; continue;
      }
      const auto elevation = snapshot_.elevation(cell);
      constexpr int kDx[4] = {1, 0, 1, 1};
      constexpr int kDy[4] = {0, 1, 1, -1};
      for (int direction = 0; direction < 4; ++direction) {
        const GridCell neighbor{cell.x + kDx[direction], cell.y + kDy[direction]};
        const auto neighbor_elevation = snapshot_.elevation(neighbor);
        if (neighbor_elevation &&
          std::abs(*neighbor_elevation - *elevation) > parameters_.max_crossable_height_jump_m)
        {
          // Match the existing hard-clearance contract, which rejects when the
          // far cell of an over-limit adjacent pair enters the clearance disk.
          const Point2D hazard_point = snapshot_.cellCenter(neighbor);
          minimum_m = std::min(minimum_m, std::hypot(
            hazard_point.x - center_point.x, hazard_point.y - center_point.y));
        }
      }
    }
  }
  clearance_cache_[*index] = minimum_m;
  return minimum_m;
}

NodeEvaluation StepEvaluator::evaluateClearance(const GridCell center) const
{
  NodeEvaluation result;
  if (!snapshot_.inBounds(center)) {
    result.reason = StepInvalidReason::kOutOfBounds;
    return result;
  }
  const auto center_z = snapshot_.elevation(center);
  if (!center_z) {
    result.reason = StepInvalidReason::kUnknown;
    return result;
  }
  result.elevation_m = *center_z;
  const int radius_cells = static_cast<int>(
    std::ceil(parameters_.hard_clearance_radius_m / snapshot_.resolution()));
  std::size_t required = 0U;
  std::size_t observed = 0U;
  std::vector<GridCell> disk;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const double distance = snapshot_.resolution() * std::hypot(dx, dy);
      if (distance > parameters_.hard_clearance_radius_m + 1.0e-12) {continue;}
      ++required;
      const GridCell cell{center.x + dx, center.y + dy};
      if (!snapshot_.inBounds(cell) || !snapshot_.observed(cell)) {continue;}
      ++observed;
      disk.push_back(cell);
    }
  }
  result.observed_support_ratio = required == 0U ? 0.0 :
    static_cast<double>(observed) / static_cast<double>(required);
  if (observed != required) {
    result.reason = StepInvalidReason::kInsufficientClearanceSupport;
    return result;
  }
  for (const auto & cell : disk) {
    const double z = *snapshot_.elevation(cell);
    constexpr int kDx[4] = {1, 0, 1, 1};
    constexpr int kDy[4] = {0, 1, 1, -1};
    for (int direction = 0; direction < 4; ++direction) {
      const GridCell neighbor{cell.x + kDx[direction], cell.y + kDy[direction]};
      const Point2D neighbor_center = snapshot_.cellCenter(neighbor);
      const Point2D center_point = snapshot_.cellCenter(center);
      if (std::hypot(
          neighbor_center.x - center_point.x,
          neighbor_center.y - center_point.y) >
        parameters_.hard_clearance_radius_m + 1.0e-12)
      {
        continue;
      }
      const auto neighbor_z = snapshot_.elevation(neighbor);
      if (!neighbor_z) {continue;}
      const double jump = std::abs(*neighbor_z - z);
      result.max_clearance_height_jump_m =
        std::max(result.max_clearance_height_jump_m, jump);
      if (jump > parameters_.max_crossable_height_jump_m) {
        result.reason = StepInvalidReason::kClearanceViolation;
        return result;
      }
    }
  }
  result.valid = true;
  result.reason = StepInvalidReason::kNone;
  result.minimum_clearance_m = nearestHazardDistance(center);
  return result;
}

NodeEvaluation StepEvaluator::evaluateNode(const Point2D point) const
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
    return NodeEvaluation{};
  }
  return mode_ == StepEvaluationMode::kCostmapHeightHybrid ?
    evaluateHybridNode(point) : evaluateClearance(snapshot_.worldToCell(point));
}

NodeEvaluation StepEvaluator::evaluateHybridNode(const Point2D point) const
{
  NodeEvaluation result;
  ++instrumentation_.costmap_queries;
  const auto cell = costmap_->worldToCell(point);
  if (!cell || !costmap_->inBounds(*cell)) {
    result.reason = StepInvalidReason::kCostmapOutOfBounds;
    return result;
  }
  const std::uint8_t raw_cost = *costmap_->cost(*cell);
  result.raw_cost = raw_cost;
  if (raw_cost == 255U) {
    result.reason = StepInvalidReason::kCostmapUnknown;
    return result;
  }
  if (raw_cost >= 253U) {
    result.reason = StepInvalidReason::kCostmapCollision;
    return result;
  }
  ++instrumentation_.height_evidence_queries;
  const HeightEvidence evidence = queryNodeHeightEvidence(
    snapshot_, point, parameters_.node_evidence_radius_m,
    parameters_.node_min_observed_cells,
    parameters_.node_max_nearest_evidence_distance_m,
    parameters_.node_height_outlier_threshold_m,
    parameters_.node_max_height_outlier_ratio);
  result.height_evidence_available = evidence.observed_cell_count > 0U;
  result.observed_support_ratio = evidence.valid ? 1.0 : 0.0;
  if (!evidence.valid) {
    result.reason = StepInvalidReason::kInsufficientHeightEvidence;
    return result;
  }
  result.valid = true;
  result.reason = StepInvalidReason::kNone;
  result.elevation_m = evidence.nearest_elevation_m;
  return result;
}

std::vector<GridCell> StepEvaluator::supercover(const Point2D from, const Point2D to) const
{
  const GridCell start = snapshot_.worldToCell(from);
  const GridCell goal = snapshot_.worldToCell(to);
  std::vector<GridCell> cells;
  cells.push_back(start);
  int x = start.x;
  int y = start.y;
  const int dx = goal.x - start.x;
  const int dy = goal.y - start.y;
  const int nx = std::abs(dx);
  const int ny = std::abs(dy);
  const int sign_x = dx >= 0 ? 1 : -1;
  const int sign_y = dy >= 0 ? 1 : -1;
  int ix = 0;
  int iy = 0;
  while (ix < nx || iy < ny) {
    const long lhs = static_cast<long>(1 + 2 * ix) * ny;
    const long rhs = static_cast<long>(1 + 2 * iy) * nx;
    if (lhs == rhs) {
      const GridCell side_x{x + sign_x, y};
      const GridCell side_y{x, y + sign_y};
      if (!(cells.back() == side_x)) {cells.push_back(side_x);}
      if (!(cells.back() == side_y)) {cells.push_back(side_y);}
      x += sign_x;
      y += sign_y;
      ++ix;
      ++iy;
    } else if (lhs < rhs) {
      x += sign_x;
      ++ix;
    } else {
      y += sign_y;
      ++iy;
    }
    const GridCell next{x, y};
    if (!(cells.back() == next)) {cells.push_back(next);}
  }
  return cells;
}

EdgeEvaluation StepEvaluator::evaluateEdge(const Point2D from, const Point2D to) const
{
  if (mode_ == StepEvaluationMode::kCostmapHeightHybrid) {
    return evaluateHybridEdge(from, to);
  }
  EdgeEvaluation result;
  if (!std::isfinite(from.x) || !std::isfinite(from.y) ||
    !std::isfinite(to.x) || !std::isfinite(to.y))
  {
    return result;
  }
  result.length_xy_m = std::hypot(to.x - from.x, to.y - from.y);
  const std::size_t intervals = std::max<std::size_t>(
    1U, static_cast<std::size_t>(
      std::ceil(result.length_xy_m / parameters_.edge_check_spacing_m)));
  result.sample_count = intervals + 1U;
  std::vector<GridCell> centerline_cells;
  std::vector<double> sample_clearances_m;
  for (std::size_t index = 0U; index <= intervals; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(intervals);
    const Point2D sample{
      from.x + ratio * (to.x - from.x),
      from.y + ratio * (to.y - from.y)};
    const NodeEvaluation node = evaluateNode(sample);
    result.max_clearance_height_jump_m = std::max(
      result.max_clearance_height_jump_m, node.max_clearance_height_jump_m);
    result.observed_support_ratio = index == 0U ? node.observed_support_ratio :
      std::min(result.observed_support_ratio, node.observed_support_ratio);
    if (!node.valid) {
      result.reason = node.reason;
      result.minimum_clearance_m = node.minimum_clearance_m;
      return result;
    }
    sample_clearances_m.push_back(node.minimum_clearance_m);
    const GridCell cell = snapshot_.worldToCell(sample);
    if (centerline_cells.empty() || !(centerline_cells.back() == cell)) {
      centerline_cells.push_back(cell);
    }
  }

  // Supercover-only cells are hard support checks. The ordered cost sequence
  // remains the sampled centerline so touching a grid corner cannot introduce
  // an artificial side-cell-to-side-cell height event.
  for (const auto & cell : supercover(from, to)) {
    if (!snapshot_.inBounds(cell)) {
      result.reason = StepInvalidReason::kOutOfBounds;
      return result;
    }
    if (!snapshot_.observed(cell)) {
      result.reason = StepInvalidReason::kUnknown;
      return result;
    }
  }
  result.unique_cell_count = centerline_cells.size();
  result.minimum_clearance_m = sample_clearances_m.empty() ? 0.0 :
    *std::min_element(sample_clearances_m.begin(), sample_clearances_m.end());
  if (parameters_.preferred_clearance_radius_m > parameters_.hard_clearance_radius_m &&
    parameters_.clearance_cost_weight > 0.0)
  {
    const double denominator = parameters_.preferred_clearance_radius_m -
      parameters_.hard_clearance_radius_m;
    const double sample_spacing_m = result.length_xy_m /
      static_cast<double>(std::max<std::size_t>(1U, intervals));
    for (const double clearance_m : sample_clearances_m) {
      const double normalized = std::clamp(
        (parameters_.preferred_clearance_radius_m - clearance_m) / denominator, 0.0, 1.0);
      result.clearance_score_m += std::pow(normalized, parameters_.clearance_cost_exponent) *
        sample_spacing_m;
    }
  }
  std::optional<double> previous_z;
  for (const auto & cell : centerline_cells) {
    const auto z = snapshot_.elevation(cell);
    if (!z) {
      result.reason = snapshot_.inBounds(cell) ?
        StepInvalidReason::kUnknown : StepInvalidReason::kOutOfBounds;
      return result;
    }
    if (previous_z) {
      const double jump = std::abs(*z - *previous_z);
      result.max_height_jump_m = std::max(result.max_height_jump_m, jump);
      if (jump > parameters_.max_crossable_height_jump_m) {
        result.reason = StepInvalidReason::kStepLimit;
        return result;
      }
      if (jump > parameters_.height_noise_floor_m) {
        ++result.height_jump_event_count;
        const double normalized = std::clamp(
          (jump - parameters_.height_noise_floor_m) /
          (parameters_.max_crossable_height_jump_m - parameters_.height_noise_floor_m),
          0.0, 1.0);
        result.height_jump_score_m += parameters_.max_crossable_height_jump_m *
          std::pow(normalized, parameters_.height_cost_exponent);
      }
    }
    previous_z = z;
  }
  result.cost = parameters_.distance_weight * result.length_xy_m +
    parameters_.height_cost_weight * result.height_jump_score_m +
    parameters_.clearance_cost_weight * result.clearance_score_m;
  result.valid = std::isfinite(result.cost);
  result.reason = result.valid ? StepInvalidReason::kNone : StepInvalidReason::kInvalidInput;
  return result;
}

EdgeEvaluation StepEvaluator::evaluateHybridEdge(const Point2D from, const Point2D to) const
{
  EdgeEvaluation result;
  if (!std::isfinite(from.x) || !std::isfinite(from.y) ||
    !std::isfinite(to.x) || !std::isfinite(to.y))
  {
    return result;
  }
  result.length_xy_m = std::hypot(to.x - from.x, to.y - from.y);
  const std::size_t intervals = std::max<std::size_t>(
    1U, static_cast<std::size_t>(
      std::ceil(result.length_xy_m / parameters_.edge_check_spacing_m)));
  result.sample_count = intervals + 1U;
  const double sample_spacing_m = result.length_xy_m / static_cast<double>(intervals);
  std::vector<HeightEvidenceSample> height_profile;
  for (std::size_t sample_index = 0U; sample_index <= intervals; ++sample_index) {
    ++instrumentation_.edge_samples_total;
    ++instrumentation_.costmap_queries;
    const double ratio = static_cast<double>(sample_index) / static_cast<double>(intervals);
    const Point2D sample{from.x + ratio * (to.x - from.x),
      from.y + ratio * (to.y - from.y)};
    const auto cost_cell = costmap_->worldToCell(sample);
    if (!cost_cell || !costmap_->inBounds(*cost_cell)) {
      result.reason = StepInvalidReason::kCostmapOutOfBounds;
      return result;
    }
    const std::uint8_t raw_cost = *costmap_->cost(*cost_cell);
    result.maximum_raw_cost = std::max(result.maximum_raw_cost, raw_cost);
    if (raw_cost == 255U) {
      result.reason = StepInvalidReason::kCostmapUnknown;
      return result;
    }
    if (raw_cost >= 253U) {
      result.reason = StepInvalidReason::kCostmapCollision;
      return result;
    }
    if (sample_index > 0U) {
      const double normalized = static_cast<double>(raw_cost) / 252.0;
      result.inflation_score_m +=
        std::pow(normalized, parameters_.inflation_cost_exponent) * sample_spacing_m;
    }
    ++instrumentation_.height_evidence_queries;
    const auto height = queryEdgeHeight(
      snapshot_, sample, parameters_.edge_height_query_radius_m);
    if (!height) {
      ++result.height_evidence_missing_samples;
      result.reason = StepInvalidReason::kInsufficientHeightEvidence;
      return result;
    }
    if (!height_profile.empty() && height_profile.back().source_cell == height->source_cell) {
      continue;
    }
    if (!height_profile.empty()) {
      const Point2D previous = snapshot_.cellCenter(height_profile.back().source_cell);
      const Point2D current = snapshot_.cellCenter(height->source_cell);
      if (std::hypot(current.x - previous.x, current.y - previous.y) >
        parameters_.edge_max_height_evidence_gap_m + 1.0e-12)
      {
        result.reason = StepInvalidReason::kHeightEvidenceGap;
        return result;
      }
    }
    height_profile.push_back(*height);
  }
  result.unique_cell_count = height_profile.size();
  for (std::size_t index = 1U; index < height_profile.size(); ++index) {
    const double jump = std::abs(
      height_profile[index].elevation_m - height_profile[index - 1U].elevation_m);
    result.max_height_jump_m = std::max(result.max_height_jump_m, jump);
    if (jump > parameters_.max_crossable_height_jump_m) {
      result.reason = StepInvalidReason::kStepLimit;
      return result;
    }
    if (jump > parameters_.height_noise_floor_m) {
      ++result.height_jump_event_count;
      const double normalized = std::clamp(
        (jump - parameters_.height_noise_floor_m) /
        (parameters_.max_crossable_height_jump_m - parameters_.height_noise_floor_m),
        0.0, 1.0);
      result.height_jump_score_m += parameters_.max_crossable_height_jump_m *
        std::pow(normalized, parameters_.height_cost_exponent);
    }
  }
  result.cost = parameters_.distance_weight * result.length_xy_m +
    parameters_.inflation_cost_weight * result.inflation_score_m +
    parameters_.height_cost_weight * result.height_jump_score_m;
  result.valid = std::isfinite(result.cost);
  result.reason = result.valid ? StepInvalidReason::kNone : StepInvalidReason::kInvalidInput;
  return result;
}

}  // namespace rubi_heightmap_step_wavefront_planner
