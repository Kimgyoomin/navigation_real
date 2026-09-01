#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "rubi_heightmap_step_wavefront_planner/graph/graph_types.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

struct PlanningStatistics
{
  std::size_t node_evaluation_calls{0U};
  std::size_t edge_evaluation_calls{0U};
  std::size_t accepted_nodes{0U};
  std::size_t accepted_edges{0U};
  std::size_t rejected_costmap{0U};
  std::size_t rejected_height_evidence{0U};
  std::size_t rejected_step_limit{0U};
  std::size_t rejected_isolated_node{0U};
  std::size_t expanded_states{0U};
  std::size_t neighbor_candidates{0U};
  std::size_t astar_open_pushes{0U};
  std::size_t sampling_trials{0U};
  std::size_t candidate_generated{0U};
  std::size_t candidate_valid{0U};
  std::size_t candidate_rejected{0U};
  std::size_t merge_queries{0U};
  std::size_t neighbor_radius_queries{0U};
  std::size_t rejected_edges{0U};
  std::size_t edge_samples_total{0U};
  std::size_t height_evidence_queries{0U};
  std::size_t costmap_queries{0U};
  std::size_t trg_collision_rejects{0U};
  std::size_t costmap_rejects{0U};
  std::size_t existing_node_queries{0U};
  std::size_t existing_node_rewires{0U};
  std::size_t new_nodes_created{0U};
  std::size_t isolated_nodes{0U};
  std::size_t neighbor_queries{0U};
  std::size_t neighbor_wire_attempts{0U};
};

struct PathMetrics
{
  double length_xy_m{0.0};
  std::size_t height_event_count{0U};
  double max_height_jump_m{0.0};
  double height_score_m{0.0};
  double minimum_clearance_m{0.0};
  double clearance_score_m{0.0};
  double inflation_score_m{0.0};
  double inflation_cost{0.0};
  double height_cost{0.0};
  std::uint8_t maximum_raw_cost{0U};
  double total_cost{0.0};
};

struct PlanResult
{
  bool success{false};
  PlanTermination termination{PlanTermination::kInvalidRequest};
  std::string message;
  std::vector<GraphNode> nodes;
  std::vector<GraphEdge> edges;
  std::vector<RejectedProposal> rejected;
  std::vector<NodeId> path_node_ids;
  std::size_t expansions{0U};
  double graph_build_time_ms{0.0};
  double astar_time_ms{0.0};
  double path_finalize_time_ms{0.0};
  double graph_clean_time_ms{0.0};
  double core_total_time_ms{0.0};
  PathMetrics path_metrics;
  PlanningStatistics statistics;
};

}  // namespace rubi_heightmap_step_wavefront_planner
