#include <algorithm>
#include <array>
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

#include "rubi_heightmap_step_wavefront_planner/heightmap_snapshot.hpp"
#include "rubi_heightmap_step_wavefront_planner/path_revalidation.hpp"
#include "rubi_heightmap_step_wavefront_planner/plan_lifecycle.hpp"
#include "rubi_heightmap_step_wavefront_planner/planner_visualization.hpp"
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

bool hostBigEndian() noexcept
{
  const std::uint16_t value = 0x0102U;
  return *reinterpret_cast<const std::uint8_t *>(&value) == 0x01U;
}

std::uint32_t swap32(const std::uint32_t value) noexcept
{
  return ((value & 0xffU) << 24U) | ((value & 0xff00U) << 8U) |
         ((value & 0xff0000U) >> 8U) | ((value & 0xff000000U) >> 24U);
}

float readFloat32(const std::uint8_t * data, const bool message_big_endian)
{
  std::uint32_t bits;
  std::memcpy(&bits, data, sizeof(bits));
  if (message_big_endian != hostBigEndian()) {bits = swap32(bits);}
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
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
    evaluator_parameters_.hard_clearance_radius_m = hard_clearance_radius_m_;
    evaluator_parameters_.edge_check_spacing_m = edge_check_spacing_m_;
    evaluator_parameters_.max_crossable_height_jump_m = max_crossable_height_jump_m_;
    evaluator_parameters_.height_noise_floor_m = height_noise_floor_m_;
    evaluator_parameters_.height_cost_exponent = height_cost_exponent_;
    evaluator_parameters_.distance_weight = distance_weight_;
    evaluator_parameters_.height_cost_weight = height_cost_weight_;
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
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::QoS(1).reliable().durability_volatile(),
      std::bind(&PlannerNode::onCloud, this, std::placeholders::_1));
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(10).reliable().durability_volatile(),
      std::bind(&PlannerNode::onGoal, this, std::placeholders::_1));
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
    base_frame_ = declare_parameter("base_frame", "base_link");
    map_resolution_m_ = declare_parameter("map_resolution_m", 0.05);
    lattice_tolerance_m_ = declare_parameter("lattice_tolerance_m", 0.01);
    max_grid_cells_ = sizeParameter("max_grid_cells", 5000000);
    transform_timeout_s_ = declare_parameter("transform_timeout_s", 0.20);
    hard_clearance_radius_m_ = declare_parameter("hard_clearance_radius_m", 0.20);
    edge_check_spacing_m_ = declare_parameter("edge_check_spacing_m", 0.025);
    max_crossable_height_jump_m_ = declare_parameter("max_crossable_height_jump_m", 0.08);
    height_noise_floor_m_ = declare_parameter("height_noise_floor_m", 0.01);
    height_cost_exponent_ = declare_parameter("height_cost_exponent", 2.0);
    distance_weight_ = declare_parameter("distance_weight", 1.0);
    height_cost_weight_ = declare_parameter("height_cost_weight", 5.0);
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

  std::vector<HeightPoint> parseCloud(const sensor_msgs::msg::PointCloud2 & cloud) const
  {
    const auto expected_row_step =
      static_cast<std::uint64_t>(cloud.width) * static_cast<std::uint64_t>(cloud.point_step);
    if (cloud.height != 1U || cloud.point_step == 0U ||
      expected_row_step > std::numeric_limits<std::uint32_t>::max() ||
      static_cast<std::uint64_t>(cloud.row_step) != expected_row_step ||
      cloud.data.size() != cloud.row_step || cloud.width == 0U)
    {
      throw std::invalid_argument("PointCloud2 must be a non-empty unorganized exact row");
    }
    auto fieldOffset = [&](const std::string & name) {
        for (const auto & field : cloud.fields) {
          if (field.name == name) {
            if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
              field.count != 1U || field.offset + sizeof(float) > cloud.point_step)
            {
              throw std::invalid_argument(name + " must be FLOAT32 count=1 within point_step");
            }
            return field.offset;
          }
        }
        throw std::invalid_argument("PointCloud2 is missing " + name);
      };
    const auto x_offset = fieldOffset("x");
    const auto y_offset = fieldOffset("y");
    const auto z_offset = fieldOffset("z");
    std::vector<HeightPoint> points;
    points.reserve(cloud.width);
    for (std::size_t index = 0U; index < cloud.width; ++index) {
      const auto * data = cloud.data.data() + index * cloud.point_step;
      points.push_back(
        {
          readFloat32(data + x_offset, cloud.is_bigendian),
          readFloat32(data + y_offset, cloud.is_bigendian),
          readFloat32(data + z_offset, cloud.is_bigendian)});
    }
    return points;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    try {
      auto snapshot = std::make_shared<HeightmapSnapshot>(
        HeightmapSnapshot::fromPoints(
          parseCloud(*cloud), map_resolution_m_, lattice_tolerance_m_, max_grid_cells_));
      bool frame_changed = false;
      bool revalidate = false;
      bool notify = false;
      std::uint64_t generation = 0U;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (map_ && map_frame_ == cloud->header.frame_id &&
          map_->contentHash() == snapshot->contentHash())
        {
          return;
        }
        frame_changed = map_ && map_frame_ != cloud->header.frame_id;
        map_ = std::move(snapshot);
        map_frame_ = cloud->header.frame_id;
        generation = ++map_generation_;
        if (frame_changed) {
          active_path_.clear();
          queued_request_.reset();
        } else if (!active_path_.empty()) {
          revalidate = true;
        } else if (pending_goal_) {
          notify = enqueueLocked(*pending_goal_, false);
          pending_goal_.reset();
        }
      }
      if (frame_changed) {
        std::lock_guard<std::mutex> output_lock(output_mutex_);
        publishFullReset(cloud->header.frame_id);
      }
      RCLCPP_INFO(
        get_logger(), "Accepted step heightmap generation=%lu frame='%s' observed=%zu "
        "grid=%zux%zu hash=%016lx",
        static_cast<unsigned long>(generation), cloud->header.frame_id.c_str(),
        map_->observedCount(), map_->sizeX(), map_->sizeY(),
        static_cast<unsigned long>(map_->contentHash()));
      if (revalidate) {revalidateActivePath();}
      if (notify) {planning_cv_.notify_one();}
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Rejected step heightmap: %s", error.what());
    }
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal)
  {
    bool no_map = false;
    bool clear_path = false;
    bool notify = false;
    std::string frame;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      ++goal_epoch_;
      automatic_replan_attempted_ = false;
      soft_failures_.reset();
      if (!map_) {
        no_map = true;
        pending_goal_ = *goal;
      } else {
        frame = map_frame_;
        clear_path = !active_path_.empty();
        active_path_.clear();
        last_goal_ = *goal;
        notify = enqueueLocked(*goal, false);
      }
    }
    if (no_map) {
      RCLCPP_WARN(get_logger(), "No accepted heightmap; stored Goal as pending");
      return;
    }
    if (clear_path) {
      std::lock_guard<std::mutex> output_lock(output_mutex_);
      publishEmptyPath(frame, now());
    }
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

  void revalidateActivePath()
  {
    const auto started = Clock::now();
    try {
      std::shared_ptr<const HeightmapSnapshot> snapshot;
      std::vector<TerrainPoint> path;
      std::string frame;
      std::uint64_t generation = 0U;
      std::size_t previous_progress = 0U;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        snapshot = map_;
        path = active_path_;
        frame = map_frame_;
        generation = map_generation_;
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
      bool invalidate = false;
      bool notify = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (generation != map_generation_ || frame != map_frame_ || active_path_.empty()) {
          return;
        }
        path_progress_ = progress;
        if (validation.valid) {
          soft_failures_.reset();
        } else if (soft_failures_.observe(
            validation.reason, path_invalid_confirmations_))
        {
          invalidate = true;
          active_path_.clear();
          if (!automatic_replan_attempted_ && last_goal_) {
            automatic_replan_attempted_ = true;
            notify = enqueueLocked(*last_goal_, true);
          }
        }
      }
      const double elapsed_ms = milliseconds(Clock::now() - started);
      RCLCPP_INFO(
        get_logger(), "Path revalidation generation=%lu valid=%s reason=%s "
        "revalidation_time_ms=%.3f",
        static_cast<unsigned long>(generation), validation.valid ? "true" : "false",
        std::string(toString(validation.reason)).c_str(), elapsed_ms);
      if (invalidate) {
        std::lock_guard<std::mutex> output_lock(output_mutex_);
        publishEmptyPath(frame, now());
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
      const Point2D start{
        start_transform.transform.translation.x,
        start_transform.transform.translation.y};
      const Point2D goal{goal_in_map.pose.position.x, goal_in_map.pose.position.y};
      const StepEvaluator evaluator(*request.map, evaluator_parameters_);
      PlanResult result = planner_->plan(evaluator, start, goal);
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
        if (!validation.valid) {return;}
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
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        if (request.goal_epoch != goal_epoch_ || output_frame != map_frame_ ||
          validated_generation != map_generation_)
        {
          return;
        }
        if (result.success) {
          active_path_ = dense;
          path_progress_ = 0U;
          last_goal_ = request.goal;
        }
        std::lock_guard<std::mutex> output_lock(output_mutex_);
        path_publisher_->publish(path);
        nodes_publisher_->publish(visualization.nodes);
        edges_publisher_->publish(visualization.edges);
        rejected_publisher_->publish(visualization.rejected);
      }
      const double postprocess_time_ms = milliseconds(Clock::now() - postprocess_start);
      const double total_time_ms = milliseconds(Clock::now() - total_start);
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
        "success=%s termination=%s nodes=%zu edges=%zu expansions=%zu "
        "queue_wait_time_ms=%.3f tf_time_ms=%.3f graph_build_time_ms=%.3f "
        "astar_time_ms=%.3f core_total_time_ms=%.3f postprocess_time_ms=%.3f "
        "total_planning_time_ms=%.3f path_pose_count=%zu path_length_xy_m=%.3f "
        "path_height_event_count=%zu path_max_height_jump_m=%.3f "
        "path_height_score_m=%.3f path_total_cost=%.3f",
        request.automatic ? "automatic" : "external",
        static_cast<unsigned long>(request.map_generation),
        static_cast<unsigned long>(validated_generation), result.success ? "true" : "false",
        std::string(toString(result.termination)).c_str(), result.nodes.size(),
        result.edges.size(), result.expansions, queue_wait_time_ms, tf_time_ms,
        result.graph_build_time_ms,
        result.astar_time_ms, result.core_total_time_ms, postprocess_time_ms,
        total_time_ms, path.poses.size(), result.path_metrics.length_xy_m,
        result.path_metrics.height_event_count, result.path_metrics.max_height_jump_m,
        result.path_metrics.height_score_m, result.path_metrics.total_cost);
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
        std::lock_guard<std::mutex> output_lock(output_mutex_);
        publishFullReset(request.frame_id);
      }
      RCLCPP_ERROR(get_logger(), "Planning TF failed after retries: %s", error.what());
    } catch (const std::exception & error) {
      bool current = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current = request.goal_epoch == goal_epoch_ && request.frame_id == map_frame_;
        if (current) {active_path_.clear();}
      }
      if (current) {
        std::lock_guard<std::mutex> output_lock(output_mutex_);
        publishFullReset(request.frame_id);
      }
      RCLCPP_ERROR(get_logger(), "Planning request failed: %s", error.what());
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
  }

  std::string input_cloud_topic_, goal_topic_, path_topic_;
  std::string debug_nodes_topic_, debug_edges_topic_, debug_rejected_topic_, base_frame_;
  double map_resolution_m_{0.05}, lattice_tolerance_m_{0.01}, transform_timeout_s_{0.20};
  double hard_clearance_radius_m_{0.20}, edge_check_spacing_m_{0.025};
  double max_crossable_height_jump_m_{0.08}, height_noise_floor_m_{0.01};
  double height_cost_exponent_{2.0}, distance_weight_{1.0}, height_cost_weight_{5.0};
  double node_sampling_distance_m_{0.30}, merge_radius_m_{0.20};
  double neighbor_connection_radius_m_{0.45}, goal_connection_distance_m_{0.45};
  double path_output_spacing_m_{0.05}, node_marker_scale_m_{0.08};
  double edge_marker_width_m_{0.025}, path_marker_width_m_{0.08};
  double rejected_marker_scale_m_{0.07};
  std::size_t max_grid_cells_{5000000U}, samples_per_expansion_{20U};
  std::size_t max_nodes_{4000U}, max_expansions_{4000U}, max_graph_build_time_ms_{5000U};
  std::size_t post_goal_expansions_{50U}, path_invalid_confirmations_{2U};
  std::size_t max_rejected_markers_{5000U};
  StepEvaluatorParameters evaluator_parameters_;
  StepWavefrontParameters planner_parameters_;
  std::unique_ptr<StepWavefrontPlanner> planner_;
  std::shared_ptr<HeightmapSnapshot> map_;
  std::string map_frame_;
  std::uint64_t map_generation_{0U}, goal_epoch_{0U};
  std::optional<geometry_msgs::msg::PoseStamped> pending_goal_, last_goal_;
  std::vector<TerrainPoint> active_path_;
  std::size_t path_progress_{0U};
  SoftFailureTracker soft_failures_;
  bool automatic_replan_attempted_{false};
  std::mutex state_mutex_;
  std::mutex output_mutex_;
  std::condition_variable planning_cv_;
  std::optional<PlanRequest> queued_request_;
  bool stop_worker_{false};
  std::thread worker_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr nodes_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr edges_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rejected_publisher_;
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
