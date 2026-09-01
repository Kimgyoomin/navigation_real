#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "rubi_heightmap_step_wavefront_planner/heightmap_snapshot.hpp"
#include "rubi_heightmap_step_wavefront_planner/path_revalidation.hpp"
#include "rubi_heightmap_step_wavefront_planner/plan_lifecycle.hpp"
#include "rubi_heightmap_step_wavefront_planner/planner_visualization.hpp"
#include "rubi_heightmap_step_wavefront_planner/planning/planning_fsm.hpp"
#include "rubi_heightmap_step_wavefront_planner/planning/planning_query_resolver.hpp"
#include "rubi_heightmap_step_wavefront_planner/ros/pointcloud2_heightmap_adapter.hpp"
#include "rubi_heightmap_step_wavefront_planner/step_evaluator.hpp"
#include "rubi_heightmap_step_wavefront_planner/step_wavefront_planner.hpp"

namespace rubi_heightmap_step_wavefront_planner
{
namespace
{

using Clock = std::chrono::steady_clock;
constexpr double kQuaternionNormSquaredEpsilon = 1.0e-12;
constexpr std::size_t kMaxTfRetryCount = 3U;

double milliseconds(const Clock::duration duration) noexcept
{
  return std::chrono::duration<double, std::milli>(duration).count();
}

std::string machineToken(std::string message)
{
  for (char & character : message) {
    const auto unsigned_character = static_cast<unsigned char>(character);
    if (!std::isalnum(unsigned_character)) {
      character = '_';
    }
  }
  return message.empty() ? "none" : message;
}

std::optional<geometry_msgs::msg::Quaternion> normalizedQuaternion(
  const geometry_msgs::msg::Quaternion & input) noexcept
{
  if (!std::isfinite(input.x) || !std::isfinite(input.y) ||
    !std::isfinite(input.z) || !std::isfinite(input.w))
  {
    return std::nullopt;
  }
  const double norm_squared = input.x * input.x + input.y * input.y +
    input.z * input.z + input.w * input.w;
  if (!std::isfinite(norm_squared) || norm_squared <= kQuaternionNormSquaredEpsilon) {
    return std::nullopt;
  }
  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  geometry_msgs::msg::Quaternion output;
  output.x = input.x * inverse_norm;
  output.y = input.y * inverse_norm;
  output.z = input.z * inverse_norm;
  output.w = input.w * inverse_norm;
  return output;
}

}  // namespace

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode()
  : Node("rubi_heightmap_step_wavefront_planner")
  {
    loadParameters();
    PlanningFsmParameters fsm_parameters;
    fsm_parameters.path_invalid_confirmations = path_invalid_confirmations_;
    fsm_parameters.path_recovery_confirmations = path_recovery_confirmations_;
    fsm_parameters.max_replan_attempts = max_replan_attempts_;
    fsm_parameters.replan_retry_period = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(replan_retry_period_s_));
    fsm_parameters.replan_retry_requires_new_map = replan_retry_requires_new_map_;
    planning_fsm_ = std::make_unique<PlanningFsm>(fsm_parameters);
    evaluator_parameters_.hard_clearance_radius_m = hard_clearance_radius_m_;
    evaluator_parameters_.edge_check_spacing_m = edge_check_spacing_m_;
    evaluator_parameters_.max_crossable_height_jump_m = max_crossable_height_jump_m_;
    evaluator_parameters_.height_noise_floor_m = height_noise_floor_m_;
    evaluator_parameters_.height_cost_exponent = height_cost_exponent_;
    evaluator_parameters_.distance_weight = distance_weight_;
    evaluator_parameters_.height_cost_weight = height_cost_weight_;
    evaluator_parameters_.preferred_clearance_radius_m = preferred_clearance_radius_m_;
    evaluator_parameters_.clearance_cost_weight = clearance_cost_weight_;
    evaluator_parameters_.clearance_cost_exponent = clearance_cost_exponent_;
    planner_parameters_.node_sampling_distance_m = node_sampling_distance_m_;
    planner_parameters_.samples_per_expansion = samples_per_expansion_;
    planner_parameters_.merge_radius_m = merge_radius_m_;
    planner_parameters_.neighbor_connection_radius_m = neighbor_connection_radius_m_;
    planner_parameters_.goal_connection_distance_m = goal_connection_distance_m_;
    planner_parameters_.max_nodes = max_nodes_;
    planner_parameters_.max_expansions = max_expansions_;
    planner_parameters_.max_graph_build_time_ms = max_graph_build_time_ms_;
    planner_parameters_.post_goal_expansions = post_goal_expansions_;
    planner_ = std::make_unique<StepWavefrontPlanner>(planner_parameters_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    const auto output_qos = rclcpp::QoS(1).reliable().transient_local();
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(path_topic_, output_qos);
    nodes_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      debug_nodes_topic_, output_qos);
    edges_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      debug_edges_topic_, output_qos);
    rejected_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      debug_rejected_topic_, output_qos);
    revalidation_failure_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      debug_revalidation_failure_topic_, output_qos);
    query_snap_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      debug_query_snap_topic_, output_qos);
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::QoS(1).reliable().durability_volatile(),
      std::bind(&PlannerNode::onCloud, this, std::placeholders::_1));
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(10).reliable().durability_volatile(),
      std::bind(&PlannerNode::onGoal, this, std::placeholders::_1));
    retry_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&PlannerNode::onRetryTimer, this));
    worker_ = std::thread(&PlannerNode::planningWorker, this);
    RCLCPP_INFO(
      get_logger(),
      "Step Wavefront ready: base_frame='%s' resolution=%.3f clearance=%.3f "
      "h_max=%.3f height_weight=%.2f post_goal_expansions=%zu max_grid_cells=%zu",
      base_frame_.c_str(), map_resolution_m_, hard_clearance_radius_m_,
      max_crossable_height_jump_m_, height_cost_weight_, post_goal_expansions_,
      max_grid_cells_);
  }

  ~PlannerNode() override
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      stop_worker_ = true;
    }
    planning_cv_.notify_one();
    if (worker_.joinable()) {worker_.join();}
  }

private:
  struct PlanRequest
  {
    geometry_msgs::msg::PoseStamped goal;
    bool automatic{false};
    Clock::time_point enqueued_at;
    std::shared_ptr<const HeightmapSnapshot> map;
    std::string frame_id;
    std::uint64_t map_generation{0U};
    std::uint64_t goal_epoch{0U};
    std::size_t tf_retry_count{0U};
  };

  void loadParameters()
  {
    input_cloud_topic_ = declare_parameter("input_cloud_topic", "/fastdem/mapping/cloud_global");
    goal_topic_ = declare_parameter("goal_topic", "/goal_pose");
    path_topic_ = declare_parameter("path_topic", "/rubi/heightmap_step_planner/path");
    debug_nodes_topic_ = declare_parameter(
      "debug_nodes_topic", "/rubi/heightmap_step_planner/debug/nodes");
    debug_edges_topic_ = declare_parameter(
      "debug_edges_topic", "/rubi/heightmap_step_planner/debug/edges");
    debug_rejected_topic_ = declare_parameter(
      "debug_rejected_topic", "/rubi/heightmap_step_planner/debug/rejected");
    debug_revalidation_failure_topic_ = declare_parameter(
      "debug_revalidation_failure_topic",
      "/rubi/heightmap_step_planner/debug/revalidation_failure");
    debug_query_snap_topic_ = declare_parameter(
      "debug_query_snap_topic", "/rubi/heightmap_step_planner/debug/query_snap");
    base_frame_ = declare_parameter("base_frame", "base_link");
    map_resolution_m_ = declare_parameter("map_resolution_m", 0.05);
    lattice_tolerance_m_ = declare_parameter("lattice_tolerance_m", 0.01);
    max_grid_cells_ = sizeParameter("max_grid_cells", 5000000);
    transform_timeout_s_ = declare_parameter("transform_timeout_s", 0.20);
    snap_start_to_valid_map_ = declare_parameter("snap_start_to_valid_map", false);
    snap_goal_to_valid_map_ = declare_parameter("snap_goal_to_valid_map", false);
    start_snap_radius_m_ = declare_parameter("start_snap_radius_m", 0.30);
    goal_snap_radius_m_ = declare_parameter("goal_snap_radius_m", 0.25);
    for (const auto & radius : {
        std::pair<const char *, double>{"start_snap_radius_m", start_snap_radius_m_},
        std::pair<const char *, double>{"goal_snap_radius_m", goal_snap_radius_m_}})
    {
      if (!std::isfinite(radius.second) || radius.second < 0.0) {
        throw std::invalid_argument(std::string(radius.first) + " must be finite and >= 0");
      }
    }
    hard_clearance_radius_m_ = declare_parameter("hard_clearance_radius_m", 0.20);
    edge_check_spacing_m_ = declare_parameter("edge_check_spacing_m", 0.025);
    max_crossable_height_jump_m_ = declare_parameter("max_crossable_height_jump_m", 0.08);
    height_noise_floor_m_ = declare_parameter("height_noise_floor_m", 0.01);
    height_cost_exponent_ = declare_parameter("height_cost_exponent", 2.0);
    distance_weight_ = declare_parameter("distance_weight", 1.0);
    height_cost_weight_ = declare_parameter("height_cost_weight", 5.0);
    preferred_clearance_radius_m_ = declare_parameter(
      "preferred_clearance_radius_m", hard_clearance_radius_m_);
    clearance_cost_weight_ = declare_parameter("clearance_cost_weight", 0.0);
    clearance_cost_exponent_ = declare_parameter("clearance_cost_exponent", 2.0);
    node_sampling_distance_m_ = declare_parameter("node_sampling_distance_m", 0.30);
    samples_per_expansion_ = sizeParameter("samples_per_expansion", 20);
    merge_radius_m_ = declare_parameter("merge_radius_m", 0.20);
    neighbor_connection_radius_m_ = declare_parameter("neighbor_connection_radius_m", 0.45);
    goal_connection_distance_m_ = declare_parameter("goal_connection_distance_m", 0.45);
    max_nodes_ = sizeParameter("max_nodes", 4000);
    max_expansions_ = sizeParameter("max_expansions", 4000);
    max_graph_build_time_ms_ = sizeParameter("max_graph_build_time_ms", 5000);
    post_goal_expansions_ = sizeParameterAllowZero("post_goal_expansions", 50);
    path_output_spacing_m_ = declare_parameter("path_output_spacing_m", 0.05);
    path_invalid_confirmations_ = sizeParameter("path_invalid_confirmations", 2);
    path_recovery_confirmations_ = sizeParameter("path_recovery_confirmations", 2);
    max_replan_attempts_ = sizeParameter("max_replan_attempts", 5);
    replan_retry_period_s_ = declare_parameter("replan_retry_period_s", 0.5);
    replan_retry_requires_new_map_ = declare_parameter(
      "replan_retry_requires_new_map", true);
    if (!std::isfinite(replan_retry_period_s_) || replan_retry_period_s_ < 0.0) {
      throw std::invalid_argument("replan_retry_period_s must be finite and >= 0");
    }
    node_marker_scale_m_ = declare_parameter("node_marker_scale_m", 0.08);
    edge_marker_width_m_ = declare_parameter("edge_marker_width_m", 0.025);
    path_marker_width_m_ = declare_parameter("path_marker_width_m", 0.08);
    rejected_marker_scale_m_ = declare_parameter("rejected_marker_scale_m", 0.07);
    max_rejected_markers_ = sizeParameter("max_rejected_markers", 5000);
  }

  std::size_t sizeParameter(const std::string & name, std::int64_t fallback)
  {
    const auto value = declare_parameter<std::int64_t>(name, fallback);
    if (value <= 0) {throw std::invalid_argument(name + " must be > 0");}
    return static_cast<std::size_t>(value);
  }

  std::size_t sizeParameterAllowZero(const std::string & name, std::int64_t fallback)
  {
    const auto value = declare_parameter<std::int64_t>(name, fallback);
    if (value < 0) {throw std::invalid_argument(name + " must be >= 0");}
    return static_cast<std::size_t>(value);
  }

  void logFsmTransitionsLocked()
  {
    for (const auto & transition : planning_fsm_->takeTransitions()) {
      RCLCPP_INFO(
        get_logger(), "FSM %s --%s[%s]--> %s",
        std::string(toString(transition.from)).c_str(),
        std::string(toString(transition.event)).c_str(),
        transition.reason.c_str(),
        std::string(toString(transition.to)).c_str());
    }
  }

  bool maybeEnqueueRetryLocked(const Clock::time_point now)
  {
    if (!last_goal_ || !planning_fsm_->retryReady(now, map_generation_)) {
      logFsmTransitionsLocked();
      return false;
    }
    const bool enqueued = enqueueLocked(*last_goal_, true);
    logFsmTransitionsLocked();
    return enqueued;
  }

  void onRetryTimer()
  {
    bool notify = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      notify = maybeEnqueueRetryLocked(Clock::now());
    }
    if (notify) {planning_cv_.notify_one();}
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    try {
      auto snapshot = std::make_shared<const HeightmapSnapshot>(
        PointCloud2HeightmapAdapter{}.makeSnapshot(
          *cloud, {map_resolution_m_, lattice_tolerance_m_, max_grid_cells_}));
      bool frame_changed = false;
      bool revalidate = false;
      bool notify = false;
      std::uint64_t generation = 0U;
      std::unique_lock<std::mutex> output_lock(output_mutex_);
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const bool had_map = static_cast<bool>(map_);
        if (map_ && map_frame_ == cloud->header.frame_id &&
          map_->contentHash() == snapshot->contentHash())
        {
          return;
        }
        frame_changed = map_ && map_frame_ != cloud->header.frame_id;
        map_ = snapshot;
        map_frame_ = cloud->header.frame_id;
        generation = ++map_generation_;
        if (frame_changed) {
          active_path_.clear();
          active_goal_orientation_.reset();
          pending_goal_.reset();
          last_goal_.reset();
          queued_request_.reset();
          path_progress_ = 0U;
          ++goal_epoch_;
          planning_fsm_->onFrameChanged(true);
        } else if (!had_map) {
          const bool has_pending_goal = static_cast<bool>(pending_goal_);
          planning_fsm_->onMapReceived(has_pending_goal);
          if (pending_goal_) {
            last_goal_ = *pending_goal_;
            notify = enqueueLocked(*pending_goal_, false);
            pending_goal_.reset();
          }
        } else if (!active_path_.empty() &&
          (planning_fsm_->state() == PlanningState::kTracking ||
          planning_fsm_->state() == PlanningState::kVerifyingPath))
        {
          revalidate = true;
        } else if (planning_fsm_->state() == PlanningState::kWaitingRetry) {
          notify = maybeEnqueueRetryLocked(Clock::now());
        }
        logFsmTransitionsLocked();
      }
      if (frame_changed) {
        publishFullReset(cloud->header.frame_id);
      }
      output_lock.unlock();
      RCLCPP_INFO(
        get_logger(), "Accepted step heightmap generation=%lu frame='%s' observed=%zu "
        "grid=%zux%zu hash=%016lx",
        static_cast<unsigned long>(generation), cloud->header.frame_id.c_str(),
        snapshot->observedCount(), snapshot->sizeX(), snapshot->sizeY(),
        static_cast<unsigned long>(snapshot->contentHash()));
      if (revalidate) {revalidateActivePath();}
      if (notify) {planning_cv_.notify_one();}
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Rejected step heightmap: %s", error.what());
    }
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal)
  {
    bool no_map = false;
    bool notify = false;
    std::string frame;
    std::unique_lock<std::mutex> output_lock(output_mutex_);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      ++goal_epoch_;
      queued_request_.reset();
      active_path_.clear();
      active_goal_orientation_.reset();
      path_progress_ = 0U;
      planning_fsm_->onGoalReceived(static_cast<bool>(map_));
      if (!map_) {
        no_map = true;
        pending_goal_ = *goal;
        last_goal_.reset();
      } else {
        frame = map_frame_;
        pending_goal_.reset();
        last_goal_ = *goal;
        notify = enqueueLocked(*goal, false);
      }
      logFsmTransitionsLocked();
    }
    if (no_map) {
      output_lock.unlock();
      RCLCPP_WARN(get_logger(), "No accepted heightmap; stored Goal as pending");
      return;
    }
    const rclcpp::Time stamp = now();
    publishEmptyPath(frame, stamp);
    revalidation_failure_publisher_->publish(
      makeDeleteRevalidationFailureMarker(frame, stamp));
    query_snap_publisher_->publish(makeDeleteAllMarkers(frame, stamp));
    output_lock.unlock();
    if (notify) {planning_cv_.notify_one();}
  }

  bool enqueueLocked(const geometry_msgs::msg::PoseStamped & goal, const bool automatic)
  {
    if (!map_) {return false;}
    if (queued_request_ && !queued_request_->automatic && automatic) {return false;}
    queued_request_ = PlanRequest{
      goal, automatic, Clock::now(), map_, map_frame_, map_generation_, goal_epoch_};
    return true;
  }

  void planningWorker()
  {
    while (true) {
      PlanRequest request;
      {
        std::unique_lock<std::mutex> lock(state_mutex_);
        planning_cv_.wait(lock, [this]() {return stop_worker_ || queued_request_.has_value();});
        if (stop_worker_) {return;}
        request = *queued_request_;
        queued_request_.reset();
      }
      executePlan(request);
    }
  }

  void failCurrentRequest(
    const PlanRequest & request,
    const std::string & reason,
    const bool clear_all_markers)
  {
    bool current = false;
    std::lock_guard<std::mutex> output_lock(output_mutex_);
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      current = request.goal_epoch == goal_epoch_ && request.frame_id == map_frame_;
      if (current) {
        active_path_.clear();
        active_goal_orientation_.reset();
        path_progress_ = 0U;
        planning_fsm_->onPlanFailed(Clock::now());
        logFsmTransitionsLocked();
      }
    }
    if (!current) {return;}
    if (clear_all_markers) {
      publishFullReset(request.frame_id);
    } else {
      publishEmptyPath(request.frame_id, now());
    }
    RCLCPP_ERROR(get_logger(), "Planning request failed: %s", reason.c_str());
  }

  void revalidateActivePath()
  {
    const auto started = Clock::now();
    try {
      std::shared_ptr<const HeightmapSnapshot> snapshot;
      std::vector<TerrainPoint> path;
      std::string frame;
      std::uint64_t generation = 0U;
      std::uint64_t epoch = 0U;
      std::size_t previous_progress = 0U;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        snapshot = map_;
        path = active_path_;
        frame = map_frame_;
        generation = map_generation_;
        epoch = goal_epoch_;
        previous_progress = path_progress_;
      }
      const auto transform = tf_buffer_->lookupTransform(
        frame, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_s_));
      const Point2D base{
        transform.transform.translation.x, transform.transform.translation.y};
      const std::size_t progress = nearestPathIndex(path, base, previous_progress);
      const StepEvaluator evaluator(*snapshot, evaluator_parameters_);
      const auto validation = validateRemainingPath(path, progress, evaluator);
      PathVerificationDecision decision;
      bool notify = false;
      std::vector<TerrainPoint> retained_path;
      std::optional<geometry_msgs::msg::Quaternion> retained_orientation;
      PlanningState state = PlanningState::kBlocked;
      std::size_t invalid_streak = 0U;
      std::size_t valid_streak = 0U;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (generation != map_generation_ || epoch != goal_epoch_ ||
          frame != map_frame_ || active_path_.empty())
        {
          return;
        }
        path_progress_ = progress;
        decision = planning_fsm_->observePath(
          validation.valid,
          validation.reason == StepInvalidReason::kInvalidInput,
          generation,
          toString(validation.reason));
        if (decision.republish_retained_path) {
          retained_path = active_path_;
          retained_orientation = active_goal_orientation_;
        }
        if (decision.clear_retained_path) {
          active_path_.clear();
          active_goal_orientation_.reset();
          path_progress_ = 0U;
        }
        if (decision.start_replan && last_goal_) {
          notify = enqueueLocked(*last_goal_, true);
        }
        state = planning_fsm_->state();
        invalid_streak = planning_fsm_->invalidStreak();
        valid_streak = planning_fsm_->validStreak();
        logFsmTransitionsLocked();
      }
      const double elapsed_ms = milliseconds(Clock::now() - started);
      RCLCPP_INFO(
        get_logger(), "Path verification: generation=%lu state=%s valid=%s reason=%s "
        "failing_segment=%zu from=(%.3f,%.3f) to=(%.3f,%.3f) "
        "min_clearance=%.3f max_height_jump=%.3f max_clearance_height_jump=%.3f "
        "observed_support_ratio=%.3f invalid_streak=%zu/%zu valid_streak=%zu/%zu "
        "revalidation_time_ms=%.3f",
        static_cast<unsigned long>(generation), std::string(toString(state)).c_str(),
        validation.valid ? "true" : "false",
        std::string(toString(validation.reason)).c_str(), validation.failing_segment,
        validation.failing_from.x, validation.failing_from.y,
        validation.failing_to.x, validation.failing_to.y,
        validation.minimum_clearance_m, validation.max_height_jump_m,
        validation.max_clearance_height_jump_m, validation.observed_support_ratio,
        invalid_streak, path_invalid_confirmations_, valid_streak,
        path_recovery_confirmations_, elapsed_ms);

      const rclcpp::Time stamp = now();
      if (!validation.valid) {
        const std::size_t from_index = std::min(validation.failing_segment, path.size() - 1U);
        const std::size_t to_index = std::min(from_index + 1U, path.size() - 1U);
        std::lock_guard<std::mutex> output_lock(output_mutex_);
        bool current = false;
        {
          std::lock_guard<std::mutex> state_lock(state_mutex_);
          current = epoch == goal_epoch_ && generation == map_generation_ &&
            frame == map_frame_;
        }
        if (!current) {return;}
        if (decision.suspend_motion) {publishEmptyPath(frame, stamp);}
        revalidation_failure_publisher_->publish(makeRevalidationFailureMarker(
            path[from_index], path[to_index], frame, stamp,
            std::max(0.10, 2.0 * path_marker_width_m_)));
      } else if (decision.republish_retained_path && retained_orientation) {
        std::lock_guard<std::mutex> output_lock(output_mutex_);
        bool current = false;
        {
          std::lock_guard<std::mutex> state_lock(state_mutex_);
          current = epoch == goal_epoch_ && generation == map_generation_ &&
            planning_fsm_->state() == PlanningState::kTracking && !active_path_.empty();
        }
        if (current) {
          path_publisher_->publish(makePath(
              retained_path, frame, *retained_orientation, stamp));
          revalidation_failure_publisher_->publish(
            makeDeleteRevalidationFailureMarker(frame, stamp));
        }
      }
      if (notify) {planning_cv_.notify_one();}
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Path revalidation skipped: %s", error.what());
    }
  }

  std::vector<TerrainPoint> densify(
    const PlanResult & result,
    const HeightmapSnapshot & snapshot) const
  {
    std::vector<TerrainPoint> dense;
    for (std::size_t path_index = 0U; path_index < result.path_node_ids.size(); ++path_index) {
      const Point2D from = result.nodes[result.path_node_ids[path_index]].point;
      const Point2D to = path_index + 1U < result.path_node_ids.size() ?
        result.nodes[result.path_node_ids[path_index + 1U]].point : from;
      const double length = std::hypot(to.x - from.x, to.y - from.y);
      const std::size_t intervals = path_index + 1U < result.path_node_ids.size() ?
        std::max<std::size_t>(
        1U, static_cast<std::size_t>(
          std::ceil(length / path_output_spacing_m_))) : 0U;
      const std::size_t begin = dense.empty() ? 0U : 1U;
      for (std::size_t sample = begin; sample <= intervals; ++sample) {
        const double ratio = intervals == 0U ? 0.0 :
          static_cast<double>(sample) / static_cast<double>(intervals);
        const Point2D point{
          from.x + ratio * (to.x - from.x), from.y + ratio * (to.y - from.y)};
        const auto z = snapshot.elevationAt(point);
        if (!z) {throw std::runtime_error("densified Path entered unknown cell");}
        dense.push_back({point.x, point.y, *z});
      }
    }
    return dense;
  }

  nav_msgs::msg::Path makePath(
    const std::vector<TerrainPoint> & dense,
    const std::string & frame,
    const geometry_msgs::msg::Quaternion & goal_orientation,
    const rclcpp::Time & stamp) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame;
    path.header.stamp = stamp;
    for (std::size_t index = 0U; index < dense.size(); ++index) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = dense[index].x;
      pose.pose.position.y = dense[index].y;
      pose.pose.position.z = dense[index].z;
      if (index + 1U < dense.size()) {
        const double yaw = std::atan2(
          dense[index + 1U].y - dense[index].y,
          dense[index + 1U].x - dense[index].x);
        pose.pose.orientation.z = std::sin(0.5 * yaw);
        pose.pose.orientation.w = std::cos(0.5 * yaw);
      } else {
        pose.pose.orientation = goal_orientation;
      }
      path.poses.push_back(std::move(pose));
    }
    return path;
  }

  void executePlan(const PlanRequest & request)
  {
    const auto total_start = Clock::now();
    const double queue_wait_time_ms = milliseconds(total_start - request.enqueued_at);
    try {
      const auto input_orientation = normalizedQuaternion(request.goal.pose.orientation);
      if (!input_orientation) {throw std::invalid_argument("Goal quaternion is invalid");}
      geometry_msgs::msg::PoseStamped normalized_goal = request.goal;
      normalized_goal.pose.orientation = *input_orientation;
      const auto tf_start = Clock::now();
      const auto start_transform = tf_buffer_->lookupTransform(
        request.frame_id, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_s_));
      geometry_msgs::msg::PoseStamped goal_in_map;
      if (normalized_goal.header.frame_id == request.frame_id) {
        goal_in_map = normalized_goal;
      } else {
        const auto goal_transform = tf_buffer_->lookupTransform(
          request.frame_id, normalized_goal.header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(transform_timeout_s_));
        tf2::doTransform(normalized_goal, goal_in_map, goal_transform);
      }
      const auto map_orientation = normalizedQuaternion(goal_in_map.pose.orientation);
      if (!map_orientation) {throw std::invalid_argument("Transformed Goal quaternion is invalid");}
      goal_in_map.pose.orientation = *map_orientation;
      const double tf_time_ms = milliseconds(Clock::now() - tf_start);
      const Point2D requested_start{
        start_transform.transform.translation.x,
        start_transform.transform.translation.y};
      const Point2D requested_goal{
        goal_in_map.pose.position.x, goal_in_map.pose.position.y};
      const StepEvaluator evaluator(*request.map, evaluator_parameters_);
      const PlanningQueryResolutionAttempt start_attempt = query_resolver_.resolveDetailed(
        *request.map, evaluator, requested_start, start_snap_radius_m_,
        snap_start_to_valid_map_);
      const PlanningQueryResolutionAttempt goal_attempt = query_resolver_.resolveDetailed(
        *request.map, evaluator, requested_goal, goal_snap_radius_m_,
        snap_goal_to_valid_map_);

      if (start_attempt.resolution && goal_attempt.resolution) {
        const auto & resolved_start = *start_attempt.resolution;
        const auto & resolved_goal = *goal_attempt.resolution;
        RCLCPP_INFO(
          get_logger(), "query_resolution start_requested=(%.3f,%.3f) "
          "start_effective=(%.3f,%.3f) start_snapped=%s start_snap_m=%.3f "
          "start_reason=%s start_candidates=%zu goal_requested=(%.3f,%.3f) "
          "goal_effective=(%.3f,%.3f) goal_snapped=%s goal_snap_m=%.3f "
          "goal_reason=%s goal_candidates=%zu",
          resolved_start.requested.x, resolved_start.requested.y,
          resolved_start.effective.x, resolved_start.effective.y,
          resolved_start.snapped ? "true" : "false", resolved_start.snap_distance_m,
          std::string(toString(resolved_start.requested_reason)).c_str(),
          resolved_start.evaluated_candidate_count,
          resolved_goal.requested.x, resolved_goal.requested.y,
          resolved_goal.effective.x, resolved_goal.effective.y,
          resolved_goal.snapped ? "true" : "false", resolved_goal.snap_distance_m,
          std::string(toString(resolved_goal.requested_reason)).c_str(),
          resolved_goal.evaluated_candidate_count);
      }

      PlanResult result;
      if (!start_attempt.resolution) {
        result.message = "start_query_resolution_failed";
        RCLCPP_WARN(
          get_logger(), "Start query resolution failed: requested=(%.3f,%.3f) "
          "initial_reason=%s search_radius_m=%.3f evaluated_candidates=%zu",
          requested_start.x, requested_start.y,
          std::string(toString(start_attempt.requested_reason)).c_str(),
          start_snap_radius_m_, start_attempt.evaluated_candidate_count);
      } else if (!goal_attempt.resolution) {
        result.message = "goal_query_resolution_failed";
        RCLCPP_WARN(
          get_logger(), "Goal query resolution failed: requested=(%.3f,%.3f) "
          "initial_reason=%s search_radius_m=%.3f evaluated_candidates=%zu",
          requested_goal.x, requested_goal.y,
          std::string(toString(goal_attempt.requested_reason)).c_str(),
          goal_snap_radius_m_, goal_attempt.evaluated_candidate_count);
      } else {
        result = planner_->plan(
          evaluator, start_attempt.resolution->effective,
          goal_attempt.resolution->effective);
      }
      const auto postprocess_start = Clock::now();
      std::vector<TerrainPoint> dense;
      if (result.success) {dense = densify(result, *request.map);}

      std::shared_ptr<const HeightmapSnapshot> output_map;
      std::string output_frame;
      std::uint64_t validated_generation = 0U;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (request.goal_epoch != goal_epoch_ || request.frame_id != map_frame_ || !map_) {
          return;
        }
        output_map = map_;
        output_frame = map_frame_;
        validated_generation = map_generation_;
      }
      if (result.success && output_map != request.map) {
        const StepEvaluator latest_evaluator(*output_map, evaluator_parameters_);
        const auto validation = validateRemainingPath(dense, 0U, latest_evaluator);
        if (!validation.valid) {
          failCurrentRequest(request, "planned Path invalid on latest map", false);
          return;
        }
        for (auto & pose : dense) {
          const auto z = output_map->elevationAt({pose.x, pose.y});
          if (!z) {return;}
          pose.z = *z;
        }
      }
      const rclcpp::Time stamp = now();
      const nav_msgs::msg::Path path = makePath(
        dense, output_frame, goal_in_map.pose.orientation, stamp);
      VisualizationParameters visualization_parameters;
      visualization_parameters.node_scale_m = node_marker_scale_m_;
      visualization_parameters.edge_width_m = edge_marker_width_m_;
      visualization_parameters.path_width_m = path_marker_width_m_;
      visualization_parameters.rejected_scale_m = rejected_marker_scale_m_;
      visualization_parameters.max_rejected_markers = max_rejected_markers_;
      const auto visualization = makeVisualization(
        result, dense, output_frame, stamp, visualization_parameters);
      {
        std::lock_guard<std::mutex> output_lock(output_mutex_);
        {
          std::lock_guard<std::mutex> state_lock(state_mutex_);
          const PlanLifecycleToken token{
            request.goal_epoch, request.map_generation, request.frame_id};
          const bool revalidated_latest = validated_generation != request.map_generation;
          if (validated_generation != map_generation_ || !mayPublish(
              token, goal_epoch_, map_generation_, map_frame_, revalidated_latest))
          {
            return;
          }
          if (result.success) {
            active_path_ = dense;
            active_goal_orientation_ = goal_in_map.pose.orientation;
            path_progress_ = 0U;
            last_goal_ = request.goal;
            planning_fsm_->onPlanSucceeded();
          } else {
            active_path_.clear();
            active_goal_orientation_.reset();
            planning_fsm_->onPlanFailed(Clock::now());
          }
          logFsmTransitionsLocked();
        }
        path_publisher_->publish(path);
        nodes_publisher_->publish(visualization.nodes);
        edges_publisher_->publish(visualization.edges);
        rejected_publisher_->publish(visualization.rejected);
        query_snap_publisher_->publish(makeQuerySnapMarkers(
            start_attempt.resolution, goal_attempt.resolution,
            output_frame, stamp));
        if (result.success) {
          revalidation_failure_publisher_->publish(
            makeDeleteRevalidationFailureMarker(output_frame, stamp));
        }
      }
      const double postprocess_time_ms = milliseconds(Clock::now() - postprocess_start);
      const double total_time_ms = milliseconds(Clock::now() - total_start);
      std::string diagnostic_message = result.message;
      if (diagnostic_message == "start is invalid") {
        diagnostic_message = "graph_build_invalid_start";
      } else if (diagnostic_message == "goal is invalid") {
        diagnostic_message = "graph_build_invalid_goal";
      } else if (diagnostic_message.empty()) {
        diagnostic_message = result.success ? "plan_succeeded" : "planning_failed";
      } else {
        diagnostic_message = machineToken(diagnostic_message);
      }
      std::array<std::size_t, 7U> reason_counts{};
      std::size_t node_rejections = 0U;
      std::size_t edge_rejections = 0U;
      std::size_t duplicate_edges = 0U;
      for (const auto & rejection : result.rejected) {
        if (rejection.kind == RejectionKind::kNode) {++node_rejections;}
        if (rejection.kind == RejectionKind::kEdge) {++edge_rejections;}
        if (rejection.kind == RejectionKind::kDuplicateEdge) {++duplicate_edges;}
        const auto reason_index = static_cast<std::size_t>(rejection.reason);
        if (reason_index < reason_counts.size()) {++reason_counts[reason_index];}
      }
      RCLCPP_INFO(
        get_logger(), "request_type=%s planned_generation=%lu validated_generation=%lu "
        "success=%s termination=%s message=%s nodes=%zu edges=%zu expansions=%zu "
        "queue_wait_time_ms=%.3f tf_time_ms=%.3f graph_build_time_ms=%.3f "
        "astar_time_ms=%.3f core_total_time_ms=%.3f postprocess_time_ms=%.3f "
        "total_planning_time_ms=%.3f path_pose_count=%zu path_length_xy_m=%.3f "
        "path_height_event_count=%zu path_max_height_jump_m=%.3f "
        "path_height_score_m=%.3f path_min_clearance_m=%.3f "
        "path_clearance_score_m=%.3f path_total_cost=%.3f",
        request.automatic ? "automatic" : "external",
        static_cast<unsigned long>(request.map_generation),
        static_cast<unsigned long>(validated_generation), result.success ? "true" : "false",
        std::string(toString(result.termination)).c_str(), diagnostic_message.c_str(),
        result.nodes.size(),
        result.edges.size(), result.expansions, queue_wait_time_ms, tf_time_ms,
        result.graph_build_time_ms,
        result.astar_time_ms, result.core_total_time_ms, postprocess_time_ms,
        total_time_ms, path.poses.size(), result.path_metrics.length_xy_m,
        result.path_metrics.height_event_count, result.path_metrics.max_height_jump_m,
        result.path_metrics.height_score_m, result.path_metrics.minimum_clearance_m,
        result.path_metrics.clearance_score_m, result.path_metrics.total_cost);
      RCLCPP_INFO(
        get_logger(), "rejection_histogram node=%zu edge=%zu duplicate_edge=%zu "
        "out_of_bounds=%zu unknown=%zu insufficient_clearance_support=%zu "
        "clearance_violation=%zu step_limit=%zu invalid_input=%zu shown=%zu total=%zu",
        node_rejections, edge_rejections, duplicate_edges,
        reason_counts[static_cast<std::size_t>(StepInvalidReason::kOutOfBounds)],
        reason_counts[static_cast<std::size_t>(StepInvalidReason::kUnknown)],
        reason_counts[
          static_cast<std::size_t>(StepInvalidReason::kInsufficientClearanceSupport)],
        reason_counts[static_cast<std::size_t>(StepInvalidReason::kClearanceViolation)],
        reason_counts[static_cast<std::size_t>(StepInvalidReason::kStepLimit)],
        reason_counts[static_cast<std::size_t>(StepInvalidReason::kInvalidInput)],
        visualization.rejected_shown, visualization.rejected_total);
    } catch (const tf2::TransformException & error) {
      bool current = false;
      bool requeued = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current = request.goal_epoch == goal_epoch_ && request.frame_id == map_frame_;
        if (current && request.tf_retry_count < kMaxTfRetryCount) {
          PlanRequest retry = request;
          retry.tf_retry_count += 1U;
          retry.enqueued_at = Clock::now();
          queued_request_ = std::move(retry);
          requeued = true;
        }
      }
      if (requeued) {
        RCLCPP_WARN(
          get_logger(), "TF unavailable; retrying current request attempt=%zu/%zu: %s",
          request.tf_retry_count + 1U, kMaxTfRetryCount, error.what());
        planning_cv_.notify_one();
        return;
      }
      if (current) {
        failCurrentRequest(request, "TF unavailable after retries", true);
      }
      RCLCPP_ERROR(get_logger(), "Planning TF failed after retries: %s", error.what());
    } catch (const std::exception & error) {
      failCurrentRequest(request, error.what(), true);
    }
  }

  void publishEmptyPath(const std::string & frame, const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame.empty() ? "map" : frame;
    path.header.stamp = stamp;
    path_publisher_->publish(path);
  }

  void publishFullReset(const std::string & frame)
  {
    const rclcpp::Time stamp = now();
    const std::string safe_frame = frame.empty() ? "map" : frame;
    publishEmptyPath(safe_frame, stamp);
    const auto clear = makeDeleteAllMarkers(safe_frame, stamp);
    nodes_publisher_->publish(clear);
    edges_publisher_->publish(clear);
    rejected_publisher_->publish(clear);
    query_snap_publisher_->publish(clear);
    revalidation_failure_publisher_->publish(
      makeDeleteRevalidationFailureMarker(safe_frame, stamp));
  }

  std::string input_cloud_topic_, goal_topic_, path_topic_;
  std::string debug_nodes_topic_, debug_edges_topic_, debug_rejected_topic_;
  std::string debug_revalidation_failure_topic_, debug_query_snap_topic_, base_frame_;
  double map_resolution_m_{0.05}, lattice_tolerance_m_{0.01}, transform_timeout_s_{0.20};
  double hard_clearance_radius_m_{0.20}, edge_check_spacing_m_{0.025};
  double max_crossable_height_jump_m_{0.08}, height_noise_floor_m_{0.01};
  double height_cost_exponent_{2.0}, distance_weight_{1.0}, height_cost_weight_{5.0};
  double preferred_clearance_radius_m_{0.20}, clearance_cost_weight_{0.0};
  double clearance_cost_exponent_{2.0};
  double node_sampling_distance_m_{0.30}, merge_radius_m_{0.20};
  double neighbor_connection_radius_m_{0.45}, goal_connection_distance_m_{0.45};
  double path_output_spacing_m_{0.05}, node_marker_scale_m_{0.08};
  double edge_marker_width_m_{0.025}, path_marker_width_m_{0.08};
  double rejected_marker_scale_m_{0.07}, replan_retry_period_s_{0.5};
  double start_snap_radius_m_{0.30}, goal_snap_radius_m_{0.25};
  std::size_t max_grid_cells_{5000000U}, samples_per_expansion_{20U};
  std::size_t max_nodes_{4000U}, max_expansions_{4000U}, max_graph_build_time_ms_{5000U};
  std::size_t post_goal_expansions_{50U}, path_invalid_confirmations_{2U};
  std::size_t path_recovery_confirmations_{2U}, max_replan_attempts_{5U};
  std::size_t max_rejected_markers_{5000U};
  bool replan_retry_requires_new_map_{true};
  bool snap_start_to_valid_map_{false}, snap_goal_to_valid_map_{false};
  StepEvaluatorParameters evaluator_parameters_;
  StepWavefrontParameters planner_parameters_;
  std::unique_ptr<StepWavefrontPlanner> planner_;
  PlanningQueryResolver query_resolver_;
  std::shared_ptr<const HeightmapSnapshot> map_;
  std::string map_frame_;
  std::uint64_t map_generation_{0U}, goal_epoch_{0U};
  std::optional<geometry_msgs::msg::PoseStamped> pending_goal_, last_goal_;
  std::optional<geometry_msgs::msg::Quaternion> active_goal_orientation_;
  std::vector<TerrainPoint> active_path_;
  std::size_t path_progress_{0U};
  std::unique_ptr<PlanningFsm> planning_fsm_;
  std::mutex state_mutex_;
  std::mutex output_mutex_;
  std::condition_variable planning_cv_;
  std::optional<PlanRequest> queued_request_;
  bool stop_worker_{false};
  std::thread worker_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr nodes_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr edges_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rejected_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    revalidation_failure_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr query_snap_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
};

}  // namespace rubi_heightmap_step_wavefront_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rubi_heightmap_step_wavefront_planner::PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
