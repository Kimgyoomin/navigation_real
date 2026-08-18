#include "pongbot_navigation/heightmap_wavefront_nav2_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "pongbot_navigation/fastdem_snapshot_adapter.hpp"

namespace pongbot_navigation
{
namespace
{

using Clock = std::chrono::steady_clock;
using rubi_heightmap_wavefront_planner::Point2D;
using rubi_heightmap_wavefront_planner::TerrainEvaluator;
using rubi_heightmap_wavefront_planner::TerrainPoint;
using rubi_heightmap_wavefront_planner::WavefrontTermination;

double milliseconds(const Clock::duration duration) noexcept
{
  return std::chrono::duration<double, std::milli>(duration).count();
}

const char * terminationName(const WavefrontTermination termination) noexcept
{
  switch (termination) {
    case WavefrontTermination::kInvalidRequest: return "invalid_request";
    case WavefrontTermination::kGoalConnected: return "goal_connected";
    case WavefrontTermination::kFrontierExhausted: return "frontier_exhausted";
    case WavefrontTermination::kMaxNodesReached: return "max_nodes";
    case WavefrontTermination::kMaxExpansionsReached: return "max_expansions";
    case WavefrontTermination::kMaxBuildTimeReached: return "max_build_time";
  }
  return "unknown";
}

bool finitePosition(const geometry_msgs::msg::Point & point) noexcept
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

geometry_msgs::msg::Quaternion normalizedQuaternion(
  const geometry_msgs::msg::Quaternion & input)
{
  if (!std::isfinite(input.x) || !std::isfinite(input.y) ||
    !std::isfinite(input.z) || !std::isfinite(input.w))
  {
    throw nav2_core::PlannerException("Goal quaternion contains non-finite components");
  }
  const double norm = std::hypot(std::hypot(input.x, input.y), std::hypot(input.z, input.w));
  if (!std::isfinite(norm) || norm <= 1.0e-12) {
    throw nav2_core::PlannerException("Goal quaternion has a zero or invalid norm");
  }
  geometry_msgs::msg::Quaternion output;
  output.x = input.x / norm;
  output.y = input.y / norm;
  output.z = input.z / norm;
  output.w = input.w / norm;
  return output;
}

bool validRawPath(const std::vector<TerrainPoint> & path, const TerrainEvaluator & evaluator)
{
  if (path.empty()) {return false;}
  for (const auto & point : path) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      !evaluator.evaluateNode({point.x, point.y}).valid)
    {
      return false;
    }
  }
  for (std::size_t index = 1U; index < path.size(); ++index) {
    const auto edge = evaluator.evaluateEdge(
      {path[index - 1U].x, path[index - 1U].y}, {path[index].x, path[index].y});
    if (!edge.valid || !std::isfinite(edge.cost)) {return false;}
  }
  return true;
}

std::vector<Point2D> densify(
  const std::vector<TerrainPoint> & path,
  const TerrainEvaluator & evaluator,
  const double spacing,
  const Point2D exact_start,
  const Point2D exact_goal)
{
  if (path.empty()) {return {};}
  std::vector<Point2D> dense;
  dense.push_back(exact_start);
  for (std::size_t segment = 1U; segment < path.size(); ++segment) {
    const Point2D from{path[segment - 1U].x, path[segment - 1U].y};
    const Point2D to{path[segment].x, path[segment].y};
    const double length = std::hypot(to.x - from.x, to.y - from.y);
    const std::size_t intervals = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(length / spacing)));
    for (std::size_t sample = 1U; sample <= intervals; ++sample) {
      const double ratio = static_cast<double>(sample) / static_cast<double>(intervals);
      const Point2D point{
        from.x + ratio * (to.x - from.x), from.y + ratio * (to.y - from.y)};
      if (!evaluator.snapshot().elevationAt(point.x, point.y)) {
        throw nav2_core::PlannerException("Densified path entered an unknown heightmap cell");
      }
      dense.push_back(point);
    }
  }
  dense.front() = exact_start;
  dense.back() = exact_goal;
  return dense;
}

}  // namespace

void HeightmapWavefrontNav2Planner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  auto node = parent_.lock();
  if (!node || !costmap_ros_) {
    throw nav2_core::PlannerException("HeightmapWavefront requires a lifecycle node and costmap");
  }
  logger_ = node->get_logger();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  auto declare = [&node, this](const std::string & key, const auto & default_value) {
      return node->declare_parameter(name_ + "." + key, default_value);
    };
  auto positiveSize = [&declare](const std::string & key, const std::int64_t value) {
      const auto parameter = declare(key, value);
      if (parameter <= 0) {throw nav2_core::PlannerException(key + " must be > 0");}
      return static_cast<std::size_t>(parameter);
    };

  input_cloud_topic_ = declare("input_cloud_topic", input_cloud_topic_);
  map_resolution_m_ = declare("map_resolution_m", map_resolution_m_);
  lattice_tolerance_m_ = declare("lattice_tolerance_m", lattice_tolerance_m_);
  reject_duplicate_cells_ = declare("reject_duplicate_cells", reject_duplicate_cells_);
  max_grid_cells_ = positiveSize("max_grid_cells", 5000000);
  max_map_receive_age_s_ = declare("max_map_receive_age_s", max_map_receive_age_s_);

  evaluator_parameters_.pca_radius_m = declare("pca_analysis_radius_m", 0.30);
  evaluator_parameters_.min_pca_points = positiveSize("pca_min_points", 6);
  evaluator_parameters_.footprint_radius_m = declare("support_radius_m", 0.26);
  evaluator_parameters_.min_footprint_observed_ratio =
    declare("minimum_observed_support_ratio", 1.0);
  evaluator_parameters_.max_slope_deg = declare("max_slope_deg", 15.0);
  const double roughness = declare("max_roughness_m", -1.0);
  evaluator_parameters_.max_roughness_m = roughness < 0.0 ?
    std::numeric_limits<double>::infinity() : roughness;
  evaluator_parameters_.max_step_height_m = declare("max_step_height_m", 0.08);
  evaluator_parameters_.edge_sample_spacing_m = declare("edge_check_spacing_m", 0.025);
  evaluator_parameters_.check_footprint_along_edge =
    declare("check_footprint_along_edge", true);

  planner_parameters_.node_sampling_distance_m = declare("node_sampling_distance_m", 0.30);
  planner_parameters_.num_expansion_samples = positiveSize("samples_per_expansion", 20);
  planner_parameters_.merge_radius_m = declare("merge_radius_m", 0.20);
  planner_parameters_.neighbor_connection_radius_m =
    declare("neighbor_connection_radius_m", 0.45);
  planner_parameters_.goal_connection_distance_m =
    declare("goal_connection_distance_m", 0.45);
  planner_parameters_.max_nodes = positiveSize("max_nodes", 4000);
  planner_parameters_.max_expansions = positiveSize("max_expansions", 4000);
  planner_parameters_.max_build_time_ms = positiveSize("max_build_time_ms", 800);
  planner_parameters_.stop_when_goal_connected = declare("stop_when_goal_connected", true);
  planner_parameters_.risk_weights.distance = declare("distance_weight", 1.0);
  planner_parameters_.risk_weights.slope = declare("slope_risk_weight", 3.0);
  planner_parameters_.risk_weights.step = declare("step_risk_weight", 0.0);
  const double roughness_weight = declare("roughness_risk_weight", 0.0);
  path_output_spacing_m_ = declare("path_output_spacing_m", path_output_spacing_m_);
  if (roughness_weight != 0.0) {
    throw nav2_core::PlannerException("roughness_risk_weight is unsupported and must be zero");
  }
  validateParameters();
  planner_ = std::make_unique<rubi_heightmap_wavefront_planner::WavefrontPlanner>(
    planner_parameters_);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  cloud_subscription_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, qos,
    std::bind(&HeightmapWavefrontNav2Planner::onCloud, this, std::placeholders::_1));
  RCLCPP_INFO(
    logger_, "Configured %s plugin; cloud='%s' global_frame='%s' max_build_time_ms=%zu",
    name_.c_str(), input_cloud_topic_.c_str(), global_frame_.c_str(),
    planner_parameters_.max_build_time_ms);
}

void HeightmapWavefrontNav2Planner::validateParameters() const
{
  if (global_frame_.empty() || !std::isfinite(map_resolution_m_) || map_resolution_m_ <= 0.0 ||
    !std::isfinite(lattice_tolerance_m_) || lattice_tolerance_m_ < 0.0 ||
    lattice_tolerance_m_ >= 0.5 * map_resolution_m_ ||
    !std::isfinite(max_map_receive_age_s_) || max_map_receive_age_s_ <= 0.0 ||
    !std::isfinite(path_output_spacing_m_) || path_output_spacing_m_ <= 0.0 ||
    evaluator_parameters_.edge_sample_spacing_m > 0.5 * map_resolution_m_)
  {
    throw nav2_core::PlannerException("HeightmapWavefront parameter contract violation");
  }
}

void HeightmapWavefrontNav2Planner::cleanup()
{
  cloud_subscription_.reset();
  planner_.reset();
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_state_.reset();
  }
  costmap_ros_.reset();
  tf_.reset();
  RCLCPP_INFO(logger_, "Cleaned up %s plugin", name_.c_str());
}

void HeightmapWavefrontNav2Planner::activate()
{
  RCLCPP_INFO(logger_, "Activated %s plugin", name_.c_str());
}

void HeightmapWavefrontNav2Planner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivated %s plugin", name_.c_str());
}

void HeightmapWavefrontNav2Planner::onCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  try {
    auto parsed = parseFastdemSnapshot(
      *cloud, map_resolution_m_, lattice_tolerance_m_, reject_duplicate_cells_,
      max_grid_cells_);
    std::uint64_t generation = 1U;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (map_state_ && map_state_->frame_id == parsed.frame_id &&
        map_state_->content_hash == parsed.content_hash)
      {
        auto refreshed = std::make_shared<MapState>(*map_state_);
        refreshed->received_at = Clock::now();
        map_state_ = std::move(refreshed);
        return;
      }
      if (map_state_) {generation = map_state_->generation + 1U;}
      auto next = std::make_shared<MapState>();
      next->snapshot = std::move(parsed.snapshot);
      next->frame_id = std::move(parsed.frame_id);
      next->content_hash = parsed.content_hash;
      next->generation = generation;
      next->received_at = Clock::now();
      map_state_ = std::move(next);
    }
    RCLCPP_INFO(logger_, "Accepted FastDEM snapshot generation=%lu", generation);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger_, "Rejected FastDEM snapshot; preserving last-good map: %s", error.what());
  }
}

std::shared_ptr<const HeightmapWavefrontNav2Planner::MapState>
HeightmapWavefrontNav2Planner::mapState() const
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  return map_state_;
}

nav_msgs::msg::Path HeightmapWavefrontNav2Planner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  const auto started = Clock::now();
  std::uint64_t generation = 0U;
  rubi_heightmap_wavefront_planner::PlanResult result;
  try {
    auto path = createPlanImpl(start, goal, generation, result);
    const double elapsed = milliseconds(Clock::now() - started);
    RCLCPP_INFO(
      logger_, "generation=%lu success=true termination=%s nodes=%zu edges=%zu "
      "expansions=%zu graph_build_ms=%.3f create_plan_ms=%.3f path_pose_count=%zu",
      generation, terminationName(result.termination), result.nodes.size(), result.edges.size(),
      result.expansions, result.build_time_ms, elapsed, path.poses.size());
    return path;
  } catch (const std::exception & error) {
    const double elapsed = milliseconds(Clock::now() - started);
    RCLCPP_ERROR(
      logger_, "generation=%lu success=false termination=%s nodes=%zu edges=%zu "
      "expansions=%zu graph_build_ms=%.3f create_plan_ms=%.3f path_pose_count=0 reason='%s'",
      generation, terminationName(result.termination), result.nodes.size(), result.edges.size(),
      result.expansions, result.build_time_ms, elapsed, error.what());
    throw nav2_core::PlannerException(error.what());
  }
}

nav_msgs::msg::Path HeightmapWavefrontNav2Planner::createPlanImpl(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::uint64_t & generation,
  rubi_heightmap_wavefront_planner::PlanResult & result)
{
  auto state = mapState();
  if (!state) {throw nav2_core::PlannerException("No accepted FastDEM snapshot");}
  generation = state->generation;
  if (Clock::now() - state->received_at >
    std::chrono::duration<double>(max_map_receive_age_s_))
  {
    throw nav2_core::PlannerException("FastDEM snapshot is stale");
  }
  if (state->frame_id != global_frame_ || start.header.frame_id != global_frame_ ||
    goal.header.frame_id != global_frame_)
  {
    throw nav2_core::PlannerException("Heightmap, start, goal, and global costmap frames must match");
  }
  if (!finitePosition(start.pose.position) || !finitePosition(goal.pose.position)) {
    throw nav2_core::PlannerException("Start or goal position is non-finite");
  }
  const auto goal_orientation = normalizedQuaternion(goal.pose.orientation);
  const Point2D start_xy{start.pose.position.x, start.pose.position.y};
  const Point2D goal_xy{goal.pose.position.x, goal.pose.position.y};
  TerrainEvaluator initial_evaluator(*state->snapshot, evaluator_parameters_);
  result = planner_->plan(initial_evaluator, start_xy, goal_xy);
  if (!result.success || result.path.empty()) {
    throw nav2_core::PlannerException(
            result.message.empty() ? "Wavefront failed to produce a path" : result.message);
  }

  for (std::size_t attempt = 0U; attempt < 3U; ++attempt) {
    auto validated_state = mapState();
    if (!validated_state || validated_state->frame_id != global_frame_) {
      throw nav2_core::PlannerException("Heightmap frame changed during planning");
    }
    TerrainEvaluator output_evaluator(*validated_state->snapshot, evaluator_parameters_);
    if (validated_state->generation != generation &&
      !validRawPath(result.path, output_evaluator))
    {
      throw nav2_core::PlannerException("Latest heightmap invalidated the planned path");
    }
    auto dense = densify(
      result.path, output_evaluator, path_output_spacing_m_, start_xy, goal_xy);
    const auto final_state = mapState();
    if (!final_state || final_state->generation != validated_state->generation ||
      final_state->frame_id != validated_state->frame_id)
    {
      continue;
    }
    generation = validated_state->generation;
    if (dense.empty()) {throw nav2_core::PlannerException("Wavefront returned an empty path");}
    nav_msgs::msg::Path output;
    auto node = parent_.lock();
    if (!node) {throw nav2_core::PlannerException("Planner lifecycle parent expired");}
    output.header.frame_id = global_frame_;
    output.header.stamp = node->now();
    for (std::size_t index = 0U; index < dense.size(); ++index) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = output.header;
      pose.pose.position.x = dense[index].x;
      pose.pose.position.y = dense[index].y;
      pose.pose.position.z = 0.0;
      if (index + 1U < dense.size()) {
        const double yaw = std::atan2(
          dense[index + 1U].y - dense[index].y,
          dense[index + 1U].x - dense[index].x);
        pose.pose.orientation.z = std::sin(0.5 * yaw);
        pose.pose.orientation.w = std::cos(0.5 * yaw);
      } else {
        pose.pose.orientation = goal_orientation;
      }
      output.poses.push_back(std::move(pose));
    }
    return output;
  }
  throw nav2_core::PlannerException("Heightmap changed repeatedly during path validation");
}

}  // namespace pongbot_navigation

PLUGINLIB_EXPORT_CLASS(
  pongbot_navigation::HeightmapWavefrontNav2Planner,
  nav2_core::GlobalPlanner)
