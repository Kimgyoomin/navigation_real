#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

#include "rubi_heightmap_wavefront_planner/wavefront_planner.hpp"

namespace rubi_heightmap_wavefront_planner
{

struct SamplingBounds
{
  double min_x_m{0.0};
  double max_x_m{0.0};
  double min_y_m{0.0};
  double max_y_m{0.0};
};

/**
 * @brief Parameters for a conventional, globally sampled RRT* tree.
 *
 * RRT* draws one global sample per iteration. In particular,
 * num_expansion_samples from the wavefront planner has no RRT* equivalent.
 */
struct RrtStarParameters
{
  std::size_t max_iterations{3000U};
  double goal_bias{0.05};
  double steer_distance_m{0.50};
  double rewire_radius_min_m{0.30};
  double rewire_radius_max_m{1.00};
  double goal_connection_distance_m{0.75};
  std::size_t max_nodes{4000U};

  // Zero disables the wall-clock budget. Fixed iteration/node budgets with
  // this value set to zero give deterministic experiment termination.
  std::size_t max_planning_time_ms{2000U};
  bool stop_on_first_solution{false};
  std::uint64_t random_seed{42U};

  // This should normally equal TerrainEvaluator's hard maximum slope.
  double slope_normalization_deg{15.0};
  WavefrontRiskWeights risk_weights{};
};

/**
 * @brief ROS-independent RRT* using TerrainEvaluator node/edge contracts.
 *
 * The returned PlanResult intentionally shares the wavefront result types so
 * ROS path and debug publishers can be reused. `expansions` stores the number
 * of RRT* iterations, and `edges` contains only the final parent tree.
 */
class RrtStarPlanner
{
public:
  using NodeEvaluator = std::function<NodeEvaluation(const Point2D &)>;
  using EdgeEvaluator =
    std::function<EdgeEvaluation(const Point2D &, const Point2D &)>;

  explicit RrtStarPlanner(
    RrtStarParameters parameters = RrtStarParameters{});

  const RrtStarParameters & parameters() const noexcept;

  PlanResult plan(
    const TerrainEvaluator & terrain,
    const Point2D & start,
    const Point2D & goal) const;

  PlanResult plan(
    const Point2D & start,
    const Point2D & goal,
    const SamplingBounds & bounds,
    const NodeEvaluator & evaluate_node,
    const EdgeEvaluator & evaluate_edge) const;

private:
  RrtStarParameters parameters_;
};

}  // namespace rubi_heightmap_wavefront_planner
