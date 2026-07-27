#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace pongbot_local_graph_insertion_planner
{

struct GridSnapshot
{
  std::string frame_id;
  std::size_t size_x{0};
  std::size_t size_y{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::uint64_t version{0};
  std::vector<unsigned char> costs;

  bool valid() const
  {
    return size_x != 0 && size_y != 0 &&
           size_x <= std::numeric_limits<std::size_t>::max() / size_y &&
           std::isfinite(resolution) && resolution > 0.0 &&
           std::isfinite(origin_x) && std::isfinite(origin_y) &&
           costs.size() == size_x * size_y;
  }
  bool inBounds(std::size_t x, std::size_t y) const {return x < size_x && y < size_y;}
  bool validIndex(std::size_t cell) const {return valid() && cell < costs.size();}
  std::size_t index(std::size_t x, std::size_t y) const {return y * size_x + x;}
  std::size_t x(std::size_t cell) const {return cell % size_x;}
  std::size_t y(std::size_t cell) const {return cell / size_x;}
  bool geometryEquals(const GridSnapshot & other) const
  {
    return size_x == other.size_x && size_y == other.size_y &&
           resolution == other.resolution && origin_x == other.origin_x &&
           origin_y == other.origin_y && frame_id == other.frame_id;
  }
};

}  // namespace pongbot_local_graph_insertion_planner
