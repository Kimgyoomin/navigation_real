#pragma once

#include <cstddef>
#include <optional>
#include <string_view>
#include <vector>

#include "rubi_heightmap_step_wavefront_planner/terrain/step_evaluator.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

using NodeId = std::size_t;

enum class PlanTermination
{
  kInvalidRequest,
  kPostGoalComplete,
  kFrontierExhausted,
  kMaxNodes,
  kMaxExpansions,
  kMaxGraphBuildTime,
};

std::string_view toString(PlanTermination termination) noexcept;

enum class RejectionKind {kNode, kEdge, kDuplicateEdge};

struct GraphNode
{
  NodeId id{0U};
  Point2D point;
  double elevation_m{0.0};
};

struct GraphEdge
{
  NodeId from{0U};
  NodeId to{0U};
  EdgeEvaluation evaluation;
};

struct RejectedProposal
{
  RejectionKind kind{RejectionKind::kNode};
  StepInvalidReason reason{StepInvalidReason::kInvalidInput};
  Point2D from;
  Point2D to;
};

struct TerrainGraph
{
  std::vector<GraphNode> nodes;
  std::vector<GraphEdge> edges;
};

struct GraphBuildResult
{
  TerrainGraph graph;
  std::vector<RejectedProposal> rejected;
  NodeId start_node_id{0U};
  std::optional<NodeId> goal_node_id;
  PlanTermination termination{PlanTermination::kInvalidRequest};
  std::size_t expansion_count{0U};
  double build_time_ms{0.0};
  std::string message;
};

struct SearchResult
{
  bool success{false};
  std::vector<NodeId> path_node_ids;
  double total_cost{0.0};
  std::size_t expanded_state_count{0U};
  double search_time_ms{0.0};
};

}  // namespace rubi_heightmap_step_wavefront_planner
