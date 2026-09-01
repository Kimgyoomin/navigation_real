#include "rubi_heightmap_step_wavefront_planner/planning/tracking_path_refiner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rubi_heightmap_step_wavefront_planner
{
namespace
{
constexpr double kTolerance = 1.0e-12;
}

TrackingPathRefiner::TrackingPathRefiner(TrackingPathRefinerParameters parameters)
: parameters_(parameters)
{
  if (!std::isfinite(parameters_.resample_spacing_m) ||
    parameters_.resample_spacing_m <= 0.0 ||
    !std::isfinite(parameters_.max_cost_increase_ratio) ||
    parameters_.max_cost_increase_ratio < 0.0)
  {
    throw std::invalid_argument("invalid tracking path refiner parameters");
  }
}

double TrackingPathRefiner::maximumHeadingChange(
  const std::vector<TerrainPoint> & path) noexcept
{
  double maximum = 0.0;
  for (std::size_t index = 1U; index + 1U < path.size(); ++index) {
    const double first = std::atan2(
      path[index].y - path[index - 1U].y,
      path[index].x - path[index - 1U].x);
    const double second = std::atan2(
      path[index + 1U].y - path[index].y,
      path[index + 1U].x - path[index].x);
    maximum = std::max(maximum, std::abs(std::remainder(second - first, 2.0 * M_PI)));
  }
  return maximum;
}

std::vector<TerrainPoint> TrackingPathRefiner::resample(
  const std::vector<TerrainPoint> & path, const StepEvaluator & evaluator,
  bool & valid) const
{
  valid = false;
  if (path.empty()) {return {};}
  std::vector<TerrainPoint> output{path.front()};
  for (std::size_t index = 1U; index < path.size(); ++index) {
    const auto & from = path[index - 1U];
    const auto & to = path[index];
    const double length = std::hypot(to.x - from.x, to.y - from.y);
    const std::size_t intervals = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(length / parameters_.resample_spacing_m)));
    for (std::size_t step = 1U; step <= intervals; ++step) {
      if (index + 1U == path.size() && step == intervals) {
        output.push_back(path.back());
        continue;
      }
      const double ratio = static_cast<double>(step) / static_cast<double>(intervals);
      const Point2D point{from.x + ratio * (to.x - from.x),
        from.y + ratio * (to.y - from.y)};
      const auto node = evaluator.evaluateNode(point);
      if (!node.valid) {return path;}
      output.push_back({point.x, point.y, node.elevation_m});
    }
  }
  valid = evaluatePolyline(output, 0U, evaluator).valid;
  return valid ? output : path;
}

TrackingPathRefinerResult TrackingPathRefiner::refine(
  const std::vector<TerrainPoint> & raw_path, const StepEvaluator & evaluator) const
{
  TrackingPathRefinerResult result;
  result.raw_point_count = raw_path.size();
  result.raw_max_heading_change_rad = maximumHeadingChange(raw_path);
  const auto raw_evaluation = evaluatePolyline(raw_path, 0U, evaluator);
  result.raw_cost = raw_evaluation.total_cost;
  result.path = raw_path;
  if (!raw_evaluation.valid) {
    result.used_raw_fallback = true;
    result.tracking_point_count = result.path.size();
    return result;
  }

  std::vector<TerrainPoint> selected = raw_path;
  if (parameters_.enabled && selected.size() >= 3U) {
    std::vector<TerrainPoint> working = selected;
    for (std::size_t pass = 0U; pass < parameters_.smoothing_passes; ++pass) {
      for (std::size_t index = 1U; index + 1U < working.size(); ++index) {
        ++result.smoothing_attempts;
        const Point2D candidate{
          (working[index - 1U].x + working[index].x + working[index + 1U].x) / 3.0,
          (working[index - 1U].y + working[index].y + working[index + 1U].y) / 3.0};
        const auto node = evaluator.evaluateNode(candidate);
        const bool accepted = node.valid &&
          evaluator.evaluateEdge(
          {working[index - 1U].x, working[index - 1U].y}, candidate).valid &&
          evaluator.evaluateEdge(
          candidate, {working[index + 1U].x, working[index + 1U].y}).valid;
        if (accepted) {
          working[index] = {candidate.x, candidate.y, node.elevation_m};
          ++result.smoothing_accepts;
        } else {
          ++result.smoothing_rejects;
        }
      }
    }
    const auto smoothed = evaluatePolyline(working, 0U, evaluator);
    if (smoothed.valid && smoothed.total_cost <=
      raw_evaluation.total_cost * (1.0 + parameters_.max_cost_increase_ratio) + kTolerance)
    {
      selected = std::move(working);
    } else {
      result.used_raw_fallback = true;
    }
  }

  bool resampled_valid = false;
  auto resampled = resample(selected, evaluator, resampled_valid);
  if (resampled_valid) {
    selected = std::move(resampled);
  }
  const auto final_evaluation = evaluatePolyline(selected, 0U, evaluator);
  if (!final_evaluation.valid) {
    selected = raw_path;
    result.used_raw_fallback = true;
  }
  result.path = std::move(selected);
  result.tracking_point_count = result.path.size();
  result.tracking_cost = evaluatePolyline(result.path, 0U, evaluator).total_cost;
  result.tracking_max_heading_change_rad = maximumHeadingChange(result.path);
  result.success = true;
  return result;
}

}  // namespace rubi_heightmap_step_wavefront_planner
