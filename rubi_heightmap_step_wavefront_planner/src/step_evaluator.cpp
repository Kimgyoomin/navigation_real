#include "rubi_heightmap_step_wavefront_planner/step_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "rubi_heightmap_step_wavefront_planner/terrain/height_evidence.hpp"

namespace rubi_heightmap_step_wavefront_planner
{
namespace
{
double interpolatedQuantile(const std::vector<double> & sorted, const double quantile)
{
  const double position = quantile * static_cast<double>(sorted.size() - 1U);
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
  const double fraction = position - static_cast<double>(lower);
  return sorted[lower] + fraction * (sorted[upper] - sorted[lower]);
}
}  // namespace

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
    case StepInvalidReason::kTerrainClearanceViolation: return "terrain_clearance_violation";
    case StepInvalidReason::kTerrainClearanceEvidenceMissing:
      return "terrain_clearance_evidence_missing";
    case StepInvalidReason::kTerrainClearanceContextUnavailable:
      return "terrain_clearance_context_unavailable";
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
    !std::isfinite(parameters_.clearance_cost_exponent) || parameters_.clearance_cost_exponent < 1.0 ||
    !std::isfinite(parameters_.sobel_equivalent_step_height_m) ||
    parameters_.sobel_equivalent_step_height_m <= 0.0 ||
    !std::isfinite(parameters_.sobel_cost_weight) || parameters_.sobel_cost_weight < 0.0 ||
    !std::isfinite(parameters_.sobel_cost_exponent) || parameters_.sobel_cost_exponent < 1.0 ||
    !std::isfinite(parameters_.grid_sobel_gradient_cost_weight) ||
    parameters_.grid_sobel_gradient_cost_weight < 0.0 ||
    (parameters_.grid_sobel_kernel_size != 3 && parameters_.grid_sobel_kernel_size != 5) ||
    !std::isfinite(parameters_.grid_terrain_clearance_distance_m) ||
    parameters_.grid_terrain_clearance_distance_m < 0.0 ||
    (parameters_.grid_terrain_clearance_enabled &&
    !parameters_.local_relief_hard_reject_enabled) ||
    !std::isfinite(parameters_.local_relief_threshold_m) ||
    parameters_.local_relief_threshold_m <= 0.0 ||
    !std::isfinite(parameters_.local_relief_first_window_radius_m) ||
    parameters_.local_relief_first_window_radius_m <= 0.0 ||
    !std::isfinite(parameters_.local_relief_second_window_radius_m) ||
    parameters_.local_relief_second_window_radius_m < 0.0 ||
    !std::isfinite(parameters_.local_relief_lower_quantile) ||
    parameters_.local_relief_lower_quantile < 0.0 ||
    parameters_.local_relief_lower_quantile >= 1.0 ||
    !std::isfinite(parameters_.local_relief_upper_quantile) ||
    parameters_.local_relief_upper_quantile <= 0.0 ||
    parameters_.local_relief_upper_quantile > 1.0 ||
    parameters_.local_relief_lower_quantile >= parameters_.local_relief_upper_quantile ||
    !std::isfinite(parameters_.local_relief_min_observed_ratio) ||
    parameters_.local_relief_min_observed_ratio <= 0.0 ||
    parameters_.local_relief_min_observed_ratio > 1.0 ||
    parameters_.local_relief_critical_cell_count == 0U)
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

std::optional<double> StepEvaluator::sobelGradientMagnitude(const GridCell center) const
{
  const auto center_index = snapshot_.index(center);
  if (!center_index) {return std::nullopt;}
  const auto cached = sobel_gradient_cache_.find(*center_index);
  if (cached != sobel_gradient_cache_.end()) {
    return std::isfinite(cached->second) ? std::optional<double>(cached->second) : std::nullopt;
  }

  ++instrumentation_.sobel_queries;
  double z[3][3]{};
  for (int row = -1; row <= 1; ++row) {
    for (int col = -1; col <= 1; ++col) {
      const auto value = snapshot_.elevation({center.x + col, center.y + row});
      if (!value) {
        ++instrumentation_.sobel_missing_neighborhoods;
        sobel_gradient_cache_[*center_index] = std::numeric_limits<double>::quiet_NaN();
        return std::nullopt;
      }
      z[row + 1][col + 1] = *value;
    }
  }

  const double raw_gx =
    -z[0][0] + z[0][2] - 2.0 * z[1][0] + 2.0 * z[1][2] - z[2][0] + z[2][2];
  const double raw_gy =
    -z[0][0] - 2.0 * z[0][1] - z[0][2] + z[2][0] + 2.0 * z[2][1] + z[2][2];
  const double denominator = 8.0 * snapshot_.resolution();
  const double gradient = std::hypot(raw_gx / denominator, raw_gy / denominator);
  sobel_gradient_cache_[*center_index] = gradient;
  return gradient;
}

std::optional<double> StepEvaluator::gridSobelGradientMagnitude(const GridCell center) const
{
  const auto center_index = snapshot_.index(center);
  if (!center_index) {
    ++instrumentation_.grid_sobel_queries;
    ++instrumentation_.grid_sobel_missing;
    return std::nullopt;
  }
  const auto cached = grid_sobel_gradient_cache_.find(*center_index);
  if (cached != grid_sobel_gradient_cache_.end()) {
    ++instrumentation_.grid_sobel_cache_hits;
    return std::isfinite(cached->second) ?
      std::optional<double>(cached->second) : std::nullopt;
  }

  ++instrumentation_.grid_sobel_queries;
  std::optional<double> gradient;
  if (parameters_.grid_sobel_kernel_size == 3) {
    gradient = sobelGradientMagnitude(center);
  } else {
    constexpr int kDerivative[5] = {-1, -2, 0, 2, 1};
    constexpr int kSmoothing[5] = {1, 4, 6, 4, 1};
    double raw_gx = 0.0;
    double raw_gy = 0.0;
    bool complete = true;
    for (int row = -2; row <= 2 && complete; ++row) {
      for (int col = -2; col <= 2; ++col) {
        const auto elevation = snapshot_.elevation({center.x + col, center.y + row});
        if (!elevation) {
          complete = false;
          break;
        }
        raw_gx += static_cast<double>(
          kSmoothing[row + 2] * kDerivative[col + 2]) * (*elevation);
        raw_gy += static_cast<double>(
          kDerivative[row + 2] * kSmoothing[col + 2]) * (*elevation);
      }
    }
    if (complete) {
      const double denominator = 128.0 * snapshot_.resolution();
      gradient = std::hypot(raw_gx / denominator, raw_gy / denominator);
      ++instrumentation_.grid_sobel_5x5_valid;
    } else {
      ++instrumentation_.grid_sobel_5x5_fallback_to_3x3;
      gradient = sobelGradientMagnitude(center);
    }
  }

  if (!gradient) {
    ++instrumentation_.grid_sobel_missing;
    grid_sobel_gradient_cache_[*center_index] =
      std::numeric_limits<double>::quiet_NaN();
    return std::nullopt;
  }
  grid_sobel_gradient_cache_[*center_index] = *gradient;
  return gradient;
}

void StepEvaluator::accumulateSobelEvidence(
  const GridCell cell, EdgeEvaluation & result) const
{
  const auto gradient = sobelGradientMagnitude(cell);
  if (!gradient) {
    ++result.sobel_missing_cell_count;
    return;
  }
  ++result.sobel_valid_cell_count;
  result.max_sobel_gradient = std::max(result.max_sobel_gradient, *gradient);

  // A standard 3x3 Sobel response on an ideal straight step H is H/(2r)
  // after normalization by 8r. Convert back to an intuitive local two-cell
  // equivalent height for logging and threshold configuration.
  const double equivalent_step_height = 2.0 * snapshot_.resolution() * (*gradient);
  result.max_sobel_equivalent_step_height_m = std::max(
    result.max_sobel_equivalent_step_height_m, equivalent_step_height);

  if (equivalent_step_height > parameters_.sobel_equivalent_step_height_m) {
    result.sobel_hard_rejection = true;
  }
  if (equivalent_step_height > parameters_.height_noise_floor_m &&
    parameters_.sobel_equivalent_step_height_m > parameters_.height_noise_floor_m)
  {
    const double normalized = std::clamp(
      (equivalent_step_height - parameters_.height_noise_floor_m) /
      (parameters_.sobel_equivalent_step_height_m - parameters_.height_noise_floor_m),
      0.0, 1.0);
    result.sobel_gradient_score_m += parameters_.sobel_equivalent_step_height_m *
      std::pow(normalized, parameters_.sobel_cost_exponent);
  }
}

StepEvaluator::LocalReliefResult StepEvaluator::localRelief(const GridCell center) const
{
  ++instrumentation_.local_relief_queries;
  const auto center_index = snapshot_.index(center);
  if (!center_index) {
    ++instrumentation_.local_relief_missing_neighborhoods;
    return {};
  }
  const auto cached = local_relief_cache_.find(*center_index);
  if (cached != local_relief_cache_.end()) {
    ++instrumentation_.local_relief_cache_hits;
    return cached->second;
  }

  LocalReliefResult result;
  std::vector<double> elevations;
  const double resolution = snapshot_.resolution();
  const double radius = parameters_.local_relief_first_window_radius_m;
  const int cell_radius = static_cast<int>(std::ceil(radius / resolution));
  for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
    for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
      if (resolution * std::hypot(static_cast<double>(dx), static_cast<double>(dy)) >
        radius + 1.0e-12)
      {
        continue;
      }
      ++result.required_cell_count;
      const auto elevation = snapshot_.elevation({center.x + dx, center.y + dy});
      if (elevation) {elevations.push_back(*elevation);}
    }
  }
  result.observed_cell_count = elevations.size();
  result.observed_ratio = result.required_cell_count > 0U ?
    static_cast<double>(result.observed_cell_count) /
    static_cast<double>(result.required_cell_count) : 0.0;
  if (!elevations.empty() &&
    result.observed_ratio + 1.0e-12 >= parameters_.local_relief_min_observed_ratio)
  {
    std::sort(elevations.begin(), elevations.end());
    result.raw_relief_m = interpolatedQuantile(
      elevations, parameters_.local_relief_upper_quantile) - interpolatedQuantile(
      elevations, parameters_.local_relief_lower_quantile);
    result.valid = std::isfinite(result.raw_relief_m) && result.raw_relief_m >= 0.0;
  }
  if (!result.valid) {++instrumentation_.local_relief_missing_neighborhoods;}
  local_relief_cache_[*center_index] = result;
  return result;
}

StepEvaluator::SupportedLocalReliefResult StepEvaluator::supportedLocalRelief(
  const GridCell center) const
{
  ++instrumentation_.supported_relief_queries;
  const auto center_index = snapshot_.index(center);
  if (!center_index) {return {};}
  const auto cached = supported_local_relief_cache_.find(*center_index);
  if (cached != supported_local_relief_cache_.end()) {return cached->second;}

  SupportedLocalReliefResult result;
  const double resolution = snapshot_.resolution();
  const double radius = parameters_.local_relief_second_window_radius_m;
  const int cell_radius = static_cast<int>(std::ceil(radius / resolution));
  for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
    for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
      if (resolution * std::hypot(static_cast<double>(dx), static_cast<double>(dy)) >
        radius + 1.0e-12)
      {
        continue;
      }
      const LocalReliefResult local = localRelief({center.x + dx, center.y + dy});
      if (!local.valid) {continue;}
      result.valid = true;
      result.max_raw_relief_m = std::max(result.max_raw_relief_m, local.raw_relief_m);
      if (local.raw_relief_m > parameters_.local_relief_threshold_m) {
        ++result.critical_count;
      }
    }
  }
  if (result.valid) {
    const double support_ratio = static_cast<double>(result.critical_count) /
      static_cast<double>(parameters_.local_relief_critical_cell_count);
    result.supported_relief_m = std::min(
      result.max_raw_relief_m, support_ratio * result.max_raw_relief_m);
  }
  supported_local_relief_cache_[*center_index] = result;
  return result;
}

void StepEvaluator::accumulateLocalReliefEvidence(
  const GridCell cell, EdgeEvaluation & result) const
{
  const SupportedLocalReliefResult relief = supportedLocalRelief(cell);
  if (!relief.valid) {
    ++result.local_relief_missing_cell_count;
    return;
  }
  ++result.local_relief_valid_cell_count;
  result.max_local_relief_m = std::max(
    result.max_local_relief_m, relief.max_raw_relief_m);
  result.max_supported_local_relief_m = std::max(
    result.max_supported_local_relief_m, relief.supported_relief_m);
  result.local_relief_max_critical_count = std::max(
    result.local_relief_max_critical_count, relief.critical_count);
  if (relief.supported_relief_m > parameters_.local_relief_threshold_m) {
    result.local_relief_hard_rejection = true;
  }
}

std::optional<double> StepEvaluator::supportedLocalReliefAt(const GridCell cell) const
{
  const SupportedLocalReliefResult result = supportedLocalRelief(cell);
  return result.valid ? std::optional<double>(result.supported_relief_m) : std::nullopt;
}

std::optional<double> StepEvaluator::gridSobelGradientAt(const Point2D point) const
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y)) {return std::nullopt;}
  return gridSobelGradientMagnitude(snapshot_.worldToCell(point));
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
  result.height_evidence_available = true;
  result.height_source_cell = center;
  result.height_source_cell_available = true;
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
  result.height_source_cell = evidence.nearest_cell;
  result.height_source_cell_available = true;
  return result;
}

EdgeEvaluation StepEvaluator::evaluateGridTransition(
  const GridCell from, const GridCell to,
  const NodeEvaluation & from_evaluation,
  const NodeEvaluation & to_evaluation) const
{
  ++instrumentation_.grid_transition_evaluations;
  EdgeEvaluation result;
  if (mode_ != StepEvaluationMode::kCostmapHeightHybrid || !costmap_) {
    return result;
  }
  const int dx = to.x - from.x;
  const int dy = to.y - from.y;
  if ((dx == 0 && dy == 0) || std::abs(dx) > 1 || std::abs(dy) > 1) {
    return result;
  }
  if (!costmap_->inBounds(from) || !costmap_->inBounds(to)) {
    result.reason = StepInvalidReason::kCostmapOutOfBounds;
    return result;
  }
  if (!from_evaluation.valid || !to_evaluation.valid) {
    result.reason = !from_evaluation.valid ? from_evaluation.reason : to_evaluation.reason;
    return result;
  }
  if (!from_evaluation.height_evidence_available ||
    !to_evaluation.height_evidence_available ||
    !from_evaluation.height_source_cell_available ||
    !to_evaluation.height_source_cell_available)
  {
    result.reason = StepInvalidReason::kInsufficientHeightEvidence;
    return result;
  }

  result.sample_count = 2U;
  result.unique_cell_count =
    from_evaluation.height_source_cell == to_evaluation.height_source_cell ? 1U : 2U;
  result.length_xy_m = costmap_->resolution() * std::hypot(dx, dy);
  result.maximum_raw_cost = std::max(from_evaluation.raw_cost, to_evaluation.raw_cost);

  // A Grid edge is one traversal into the neighbor cell. Unlike evaluateEdge(),
  // inflation is not numerically integrated at 2.5 cm sub-cell samples.
  const double normalized_inflation = static_cast<double>(to_evaluation.raw_cost) / 252.0;
  result.inflation_score_m = std::pow(
    normalized_inflation, parameters_.inflation_cost_exponent) * result.length_xy_m;

  accumulateSobelEvidence(from_evaluation.height_source_cell, result);
  accumulateLocalReliefEvidence(from_evaluation.height_source_cell, result);
  if (!(from_evaluation.height_source_cell == to_evaluation.height_source_cell)) {
    accumulateSobelEvidence(to_evaluation.height_source_cell, result);
    accumulateLocalReliefEvidence(to_evaluation.height_source_cell, result);
  }

  const auto from_gradient = gridSobelGradientMagnitude(from_evaluation.height_source_cell);
  const auto to_gradient = gridSobelGradientMagnitude(to_evaluation.height_source_cell);
  if (from_gradient || to_gradient) {
    const double mean_gradient = from_gradient && to_gradient ?
      0.5 * (*from_gradient + *to_gradient) :
      (from_gradient ? *from_gradient : *to_gradient);
    result.max_grid_sobel_gradient = std::max(
      from_gradient.value_or(0.0), to_gradient.value_or(0.0));
    // Treat w_s * G(c) as a scalar terrain cost density. Multiplying the
    // endpoint-average density by edge length is a trapezoidal discrete
    // line-cost approximation accumulated naturally by A*. The length factor
    // also prevents an equal per-cell penalty from favoring longer diagonals.
    result.sobel_gradient_exposure_m = result.length_xy_m * mean_gradient;
  }

  const double jump = std::abs(to_evaluation.elevation_m - from_evaluation.elevation_m);
  result.max_height_jump_m = jump;
  if (jump > parameters_.max_crossable_height_jump_m) {
    result.reason = StepInvalidReason::kStepLimit;
    return result;
  }
  if (jump > parameters_.height_noise_floor_m) {
    result.height_jump_event_count = 1U;
    const double normalized = std::clamp(
      (jump - parameters_.height_noise_floor_m) /
      (parameters_.max_crossable_height_jump_m - parameters_.height_noise_floor_m),
      0.0, 1.0);
    result.height_jump_score_m = parameters_.max_crossable_height_jump_m *
      std::pow(normalized, parameters_.height_cost_exponent);
  }
  if (parameters_.grid_sobel_hard_reject_enabled && result.sobel_hard_rejection) {
    result.reason = StepInvalidReason::kStepLimit;
    return result;
  }
  if (parameters_.local_relief_hard_reject_enabled &&
    result.local_relief_hard_rejection)
  {
    result.reason = StepInvalidReason::kStepLimit;
    return result;
  }
  result.cost = parameters_.distance_weight * result.length_xy_m +
    parameters_.inflation_cost_weight * result.inflation_score_m +
    parameters_.height_cost_weight * result.height_jump_score_m +
    parameters_.grid_sobel_gradient_cost_weight * result.sobel_gradient_exposure_m;
  result.valid = std::isfinite(result.cost);
  result.reason = result.valid ? StepInvalidReason::kNone : StepInvalidReason::kInvalidInput;
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
    accumulateSobelEvidence(cell, result);
    accumulateLocalReliefEvidence(cell, result);
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
  if (parameters_.sobel_hard_reject_enabled && result.sobel_hard_rejection) {
    result.reason = StepInvalidReason::kStepLimit;
    return result;
  }
  if (parameters_.local_relief_hard_reject_enabled &&
    result.local_relief_hard_rejection)
  {
    result.reason = StepInvalidReason::kStepLimit;
    return result;
  }
  result.cost = parameters_.distance_weight * result.length_xy_m +
    parameters_.height_cost_weight * result.height_jump_score_m +
    parameters_.clearance_cost_weight * result.clearance_score_m +
    parameters_.sobel_cost_weight * result.sobel_gradient_score_m;
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
  for (std::size_t index = 0U; index < height_profile.size(); ++index) {
    accumulateSobelEvidence(height_profile[index].source_cell, result);
    accumulateLocalReliefEvidence(height_profile[index].source_cell, result);
    if (index == 0U) {continue;}
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
  if (parameters_.sobel_hard_reject_enabled && result.sobel_hard_rejection) {
    result.reason = StepInvalidReason::kStepLimit;
    return result;
  }
  if (parameters_.local_relief_hard_reject_enabled &&
    result.local_relief_hard_rejection)
  {
    result.reason = StepInvalidReason::kStepLimit;
    return result;
  }
  result.cost = parameters_.distance_weight * result.length_xy_m +
    parameters_.inflation_cost_weight * result.inflation_score_m +
    parameters_.height_cost_weight * result.height_jump_score_m +
    parameters_.sobel_cost_weight * result.sobel_gradient_score_m;
  result.valid = std::isfinite(result.cost);
  result.reason = result.valid ? StepInvalidReason::kNone : StepInvalidReason::kInvalidInput;
  return result;
}

}  // namespace rubi_heightmap_step_wavefront_planner
