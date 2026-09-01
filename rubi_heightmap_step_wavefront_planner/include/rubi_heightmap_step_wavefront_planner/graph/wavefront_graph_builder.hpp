#pragma once

#include <cstdint>

#include "rubi_heightmap_step_wavefront_planner/graph/graph_types.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

enum class SamplingPolicy {kDeterministicRing, kTrgRandomRing, kOriginalTrgRandomRing};

struct WavefrontGraphParameters
{
  double node_sampling_distance_m{0.30};
  std::size_t samples_per_expansion{20U};
  double merge_radius_m{0.20};
  double neighbor_connection_radius_m{0.45};
  double goal_connection_distance_m{0.45};
  std::size_t max_nodes{4000U};
  std::size_t max_expansions{4000U};
  std::size_t max_graph_build_time_ms{5000U};
  std::size_t post_goal_expansions{50U};
  SamplingPolicy sampling_policy{SamplingPolicy::kDeterministicRing};
  std::uint32_t random_seed{42U};
  std::size_t max_sampling_trials_per_expansion{1000U};
  double trg_expand_distance_m{0.30};
  double trg_robot_size_m{0.20};
  std::size_t trg_sample_num{20U};
  std::size_t trg_max_trial_samples{1000U};
  double trg_height_threshold_m{0.08};
  double trg_collision_threshold{0.10};
  std::uint32_t trg_random_seed{42U};
  bool trg_randomize_seed{false};
  double trg_neighbor_connection_radius_m{0.30};
};

/**
 * @brief Builds one deterministic accepted-edge graph for a single request.
 *
 * The builder owns graph topology only. Terrain validity and edge risk remain
 * exclusively owned by StepEvaluator.
 */
class WavefrontGraphBuilder
{
public:
  explicit WavefrontGraphBuilder(WavefrontGraphParameters parameters);
  GraphBuildResult build(
    const StepEvaluator & evaluator, Point2D start, Point2D goal) const;

private:
  GraphBuildResult buildOriginalTrg(
    const StepEvaluator & evaluator, Point2D start, Point2D goal) const;
  WavefrontGraphParameters parameters_;
};

}  // namespace rubi_heightmap_step_wavefront_planner
