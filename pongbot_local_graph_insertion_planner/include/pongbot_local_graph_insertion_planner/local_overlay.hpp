#pragma once

#include "pongbot_local_graph_insertion_planner/grid_snapshot.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace pongbot_local_graph_insertion_planner
{

struct LocalGrid
{
  std::size_t size_x{0};
  std::size_t size_y{0};
  double resolution{0.0};
  std::vector<unsigned char> costs;

  bool valid() const;
};

// Maps coordinates measured from the local grid origin into the global frame.
struct Transform2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};

  bool valid() const;
};

struct OverlayResult
{
  bool success{false};
  std::string failure_reason;
  std::size_t overlay_cells{0};
};

// Applies known local costs with max aggregation. Local unknown and free cells
// never erase the immutable global base snapshot.
OverlayResult applyLocalOverlay(
  GridSnapshot & fused,
  const LocalGrid & local,
  const Transform2D & global_from_local_grid);

}  // namespace pongbot_local_graph_insertion_planner
