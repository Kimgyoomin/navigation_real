#pragma once

#include <cstddef>
#include <vector>

#include "rubi_heightmap_step_wavefront_planner/planning/path_cost_evaluator.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

struct TrackingPathRefinerParameters
{
  bool enabled{true};
  std::size_t smoothing_passes{1U};
  double resample_spacing_m{0.10};
  double max_cost_increase_ratio{0.05};
};

struct TrackingPathRefinerResult
{
  bool success{false};
  std::vector<TerrainPoint> path;
  std::size_t raw_point_count{0U};
  std::size_t tracking_point_count{0U};
  std::size_t smoothing_attempts{0U};
  std::size_t smoothing_accepts{0U};
  std::size_t smoothing_rejects{0U};
  double raw_cost{0.0};
  double tracking_cost{0.0};
  double raw_max_heading_change_rad{0.0};
  double tracking_max_heading_change_rad{0.0};
  bool used_raw_fallback{false};
};

class TrackingPathRefiner
{
public:
  explicit TrackingPathRefiner(TrackingPathRefinerParameters parameters);

  TrackingPathRefinerResult refine(
    const std::vector<TerrainPoint> & raw_path,
    const StepEvaluator & evaluator) const;

private:
  static double maximumHeadingChange(const std::vector<TerrainPoint> & path) noexcept;
  std::vector<TerrainPoint> resample(
    const std::vector<TerrainPoint> & path,
    const StepEvaluator & evaluator,
    bool & valid) const;

  TrackingPathRefinerParameters parameters_;
};

}  // namespace rubi_heightmap_step_wavefront_planner
