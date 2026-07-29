#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace rubi_heightmap_wavefront_planner
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct GridIndex
{
  std::size_t x{0};
  std::size_t y{0};

  bool operator==(const GridIndex & other) const noexcept
  {
    return x == other.x && y == other.y;
  }
};

struct TerrainPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct TerrainCell
{
  GridIndex index{};
  Point2D center{};
  double elevation_m{0.0};
};

/**
 * @brief Immutable, cropped representation of a regular 2.5-D elevation lattice.
 *
 * Every stored point represents one lattice-cell center. An unobserved cell remains
 * explicitly unobserved; queries never substitute a nearest observed neighbor.
 */
class TerrainSnapshot
{
public:
  TerrainSnapshot(
    double resolution_m,
    double min_x_center_m,
    double min_y_center_m,
    std::size_t size_x,
    std::size_t size_y,
    std::vector<double> elevation_m,
    std::vector<std::uint8_t> observed);

  /**
   * @brief Construct the smallest dense lattice containing a sparse set of cells.
   *
   * The minimum x/y point becomes the lattice anchor. Every other point must be
   * within lattice_tolerance_m of an integer lattice offset. Duplicate cells,
   * non-finite values, and off-lattice points are rejected with std::invalid_argument.
   */
  static TerrainSnapshot fromPoints(
    const std::vector<TerrainPoint> & points,
    double resolution_m,
    double lattice_tolerance_m,
    std::size_t max_cell_count = std::numeric_limits<std::size_t>::max());

  double resolution() const noexcept;
  double minXCenter() const noexcept;
  double minYCenter() const noexcept;
  double maxXCenter() const noexcept;
  double maxYCenter() const noexcept;
  std::size_t sizeX() const noexcept;
  std::size_t sizeY() const noexcept;
  std::size_t cellCount() const noexcept;
  std::size_t observedCount() const noexcept;

  bool inBounds(std::size_t ix, std::size_t iy) const noexcept;
  bool isObserved(std::size_t ix, std::size_t iy) const noexcept;

  /**
   * @brief Return the exact regular-grid cell containing an XY position.
   *
   * "Exact" refers to cell identity: if that cell is unobserved, query() fails.
   * This method does not search for a nearby observed cell.
   */
  std::optional<GridIndex> worldToIndex(double x, double y) const noexcept;
  std::optional<Point2D> cellCenter(std::size_t ix, std::size_t iy) const noexcept;
  std::optional<double> elevationAtCell(std::size_t ix, std::size_t iy) const noexcept;
  std::optional<double> elevationAt(double x, double y) const noexcept;
  std::optional<TerrainCell> query(double x, double y) const noexcept;

  const std::vector<double> & elevations() const noexcept;
  const std::vector<std::uint8_t> & observedMask() const noexcept;

private:
  std::size_t flatIndex(std::size_t ix, std::size_t iy) const noexcept;

  double resolution_m_{0.0};
  double min_x_center_m_{0.0};
  double min_y_center_m_{0.0};
  std::size_t size_x_{0};
  std::size_t size_y_{0};
  std::size_t observed_count_{0};
  std::vector<double> elevation_m_;
  std::vector<std::uint8_t> observed_;
};

}  // namespace rubi_heightmap_wavefront_planner
