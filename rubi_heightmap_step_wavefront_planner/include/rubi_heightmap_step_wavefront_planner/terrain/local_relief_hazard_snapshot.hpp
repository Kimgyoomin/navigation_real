#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "rubi_heightmap_step_wavefront_planner/step_evaluator.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

struct TerrainClearanceQuery
{
  bool evidence_valid{false};
  bool violation{false};
  bool minimum_distance_exact{true};
  double minimum_distance_m{std::numeric_limits<double>::infinity()};
  Point2D nearest_seed_center;
};

class LocalReliefHazardSnapshot
{
public:
  static LocalReliefHazardSnapshot build(const StepEvaluator & evaluator);
  static double pointToCellAreaDistance(
    Point2D point, Point2D cell_center, double resolution_m) noexcept;
  static double segmentToCellAreaDistance(
    Point2D from, Point2D to, Point2D cell_center, double resolution_m) noexcept;

  bool matches(const StepEvaluator & evaluator) const noexcept;
  TerrainClearanceQuery queryPoint(Point2D point, double clearance_m) const;
  TerrainClearanceQuery querySegment(
    Point2D from, Point2D to, double clearance_m) const;

  std::size_t seedCount() const noexcept {return seed_cells_.size();}
  const std::vector<GridCell> & seedCells() const noexcept {return seed_cells_;}
  std::size_t evidenceInvalidCount() const noexcept {return evidence_invalid_count_;}
  std::uint64_t heightmapHash() const noexcept {return heightmap_hash_;}
  double resolution() const noexcept {return resolution_m_;}

private:
  std::optional<std::size_t> index(GridCell cell) const noexcept;
  GridCell worldToCell(Point2D point) const noexcept;
  Point2D cellCenter(GridCell cell) const noexcept;
  bool segmentEvidenceValid(Point2D from, Point2D to) const;

  std::uint64_t heightmap_hash_{0U};
  double resolution_m_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::size_t size_x_{0U};
  std::size_t size_y_{0U};
  double relief_threshold_m_{0.0};
  double first_window_radius_m_{0.0};
  double second_window_radius_m_{0.0};
  double lower_quantile_{0.0};
  double upper_quantile_{0.0};
  double minimum_observed_ratio_{0.0};
  std::size_t critical_cell_count_{0U};
  std::vector<bool> evidence_valid_;
  std::vector<bool> seed_mask_;
  std::vector<GridCell> seed_cells_;
  std::size_t evidence_invalid_count_{0U};
};

}  // namespace rubi_heightmap_step_wavefront_planner
