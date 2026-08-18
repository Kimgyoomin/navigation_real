#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rubi_heightmap_step_wavefront_planner
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct HeightPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct GridCell
{
  int x{0};
  int y{0};

  bool operator==(const GridCell & other) const noexcept
  {
    return x == other.x && y == other.y;
  }
};

class HeightmapSnapshot
{
public:
  static HeightmapSnapshot fromPoints(
    const std::vector<HeightPoint> & points,
    double resolution_m,
    double lattice_tolerance_m,
    std::size_t max_grid_cells);

  double resolution() const noexcept {return resolution_m_;}
  double originX() const noexcept {return origin_x_;}
  double originY() const noexcept {return origin_y_;}
  std::size_t sizeX() const noexcept {return size_x_;}
  std::size_t sizeY() const noexcept {return size_y_;}
  std::size_t cellCount() const noexcept {return elevations_.size();}
  std::size_t observedCount() const noexcept {return observed_count_;}
  std::uint64_t contentHash() const noexcept {return content_hash_;}

  bool inBounds(GridCell cell) const noexcept;
  std::optional<std::size_t> index(GridCell cell) const noexcept;
  GridCell worldToCell(Point2D point) const noexcept;
  Point2D cellCenter(GridCell cell) const noexcept;
  bool observed(GridCell cell) const noexcept;
  std::optional<double> elevation(GridCell cell) const noexcept;
  std::optional<double> elevationAt(Point2D point) const noexcept;

private:
  double resolution_m_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::size_t size_x_{0U};
  std::size_t size_y_{0U};
  std::size_t observed_count_{0U};
  std::uint64_t content_hash_{0U};
  std::vector<double> elevations_;
  std::vector<bool> observed_;
};

}  // namespace rubi_heightmap_step_wavefront_planner
