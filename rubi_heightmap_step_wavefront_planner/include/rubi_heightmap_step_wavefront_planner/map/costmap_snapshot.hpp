#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "rubi_heightmap_step_wavefront_planner/heightmap_snapshot.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

class CostmapSnapshot
{
public:
  static CostmapSnapshot fromData(
    std::size_t size_x, std::size_t size_y, double resolution_m,
    double origin_x, double origin_y, std::vector<std::uint8_t> costs);

  double resolution() const noexcept {return resolution_m_;}
  double originX() const noexcept {return origin_x_;}
  double originY() const noexcept {return origin_y_;}
  std::size_t sizeX() const noexcept {return size_x_;}
  std::size_t sizeY() const noexcept {return size_y_;}
  std::size_t cellCount() const noexcept {return costs_.size();}

  bool inBounds(GridCell cell) const noexcept;
  std::optional<std::size_t> index(GridCell cell) const noexcept;
  std::optional<GridCell> worldToCell(Point2D point) const noexcept;
  Point2D cellCenter(GridCell cell) const noexcept;
  std::optional<std::uint8_t> cost(GridCell cell) const noexcept;
  std::optional<std::uint8_t> costAt(Point2D point) const noexcept;

private:
  double resolution_m_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::size_t size_x_{0U};
  std::size_t size_y_{0U};
  std::vector<std::uint8_t> costs_;
};

}  // namespace rubi_heightmap_step_wavefront_planner
