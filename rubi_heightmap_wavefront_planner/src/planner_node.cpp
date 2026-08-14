#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
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

#include "rubi_heightmap_wavefront_planner/planner_visualization.hpp"
#include "rubi_heightmap_wavefront_planner/path_revalidation.hpp"
#include "rubi_heightmap_wavefront_planner/plan_lifecycle.hpp"
#include "rubi_heightmap_wavefront_planner/rrt_star_planner.hpp"
#include "rubi_heightmap_wavefront_planner/terrain_evaluator.hpp"
#include "rubi_heightmap_wavefront_planner/terrain_snapshot.hpp"
#include "rubi_heightmap_wavefront_planner/wavefront_planner.hpp"

namespace rubi_heightmap_wavefront_planner
{
namespace
{

constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;
constexpr double kEpsilon = 1.0e-12;
constexpr double kQuaternionNormSquaredEpsilon = 1.0e-12;

enum class PlannerMode
{
  kWavefront,
  kRrtStar
};

bool hostIsBigEndian() noexcept
{
  const std::uint16_t value = 0x0102U;
  std::uint8_t bytes[sizeof(value)]{};
  std::memcpy(bytes, &value, sizeof(value));
  return bytes[0] == 0x01U;
}

std::uint32_t byteSwap32(std::uint32_t value) noexcept
{
  return
    ((value & 0x000000ffU) << 24U) |
    ((value & 0x0000ff00U) << 8U) |
    ((value & 0x00ff0000U) >> 8U) |
    ((value & 0xff000000U) >> 24U);
}

void hashByte(std::uint64_t & hash, const std::uint8_t byte) noexcept
{
  hash ^= byte;
  hash *= kFnvPrime;
}

void hashUint32(std::uint64_t & hash, const std::uint32_t value) noexcept
{
  for (std::size_t shift = 0U; shift < 32U; shift += 8U) {
    hashByte(hash, static_cast<std::uint8_t>((value >> shift) & 0xffU));
  }
}

void hashString(std::uint64_t & hash, const std::string & value) noexcept
{
  for (const unsigned char byte : value) {
    hashByte(hash, static_cast<std::uint8_t>(byte));
  }
  // Separate a frame name from the first point even when the frame is empty.
  hashByte(hash, 0xffU);
}

std::string terminationName(const WavefrontTermination termination)
{
  switch (termination) {
    case WavefrontTermination::kInvalidRequest:
      return "invalid_request";
    case WavefrontTermination::kGoalConnected:
      return "goal_connected";
    case WavefrontTermination::kFrontierExhausted:
      return "frontier_exhausted";
    case WavefrontTermination::kMaxNodesReached:
      return "max_nodes";
    case WavefrontTermination::kMaxExpansionsReached:
      return "max_expansions";
    case WavefrontTermination::kMaxBuildTimeReached:
      return "max_build_time";
  }
  return "unknown";
}

bool finitePoint(const Point2D point) noexcept
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

std::optional<geometry_msgs::msg::Quaternion> normalizedQuaternion(
  const geometry_msgs::msg::Quaternion & quaternion) noexcept
{
  if (!std::isfinite(quaternion.x) || !std::isfinite(quaternion.y) ||
    !std::isfinite(quaternion.z) || !std::isfinite(quaternion.w))
  {
    return std::nullopt;
  }

  const double norm_squared =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  if (!std::isfinite(norm_squared) ||
    norm_squared <= kQuaternionNormSquaredEpsilon)
  {
    return std::nullopt;
  }

  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  geometry_msgs::msg::Quaternion normalized;
  normalized.x = quaternion.x * inverse_norm;
  normalized.y = quaternion.y * inverse_norm;
  normalized.z = quaternion.z * inverse_norm;
  normalized.w = quaternion.w * inverse_norm;
  return normalized;
}

}  // namespace

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode()
  : Node("rubi_heightmap_wavefront_planner")
  {
    loadParameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto latched_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    path_publisher_ =
      create_publisher<nav_msgs::msg::Path>(path_topic_, latched_qos);
    nodes_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      debug_nodes_topic_, latched_qos);
    edges_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      debug_edges_topic_, latched_qos);
    rejected_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      debug_rejected_topic_, latched_qos);

    const auto cloud_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, cloud_qos,
      std::bind(&PlannerNode::onCloud, this, std::placeholders::_1));
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      std::bind(&PlannerNode::onGoal, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Heightmap planner ready: mode='%s', cloud='%s', goal='%s', base_frame='%s', "
      "resolution=%.3f m, lattice_tolerance=%.3f m, edge_spacing=%.3f m, "
      "support_radius=%.3f m, minimum_support=%.2f, max_step=%.3f m, "
      "max_slope=%.1f deg, invalid_confirmations=%zu, max_grid_cells=%zu",
      planner_mode_name_.c_str(), input_cloud_topic_.c_str(),
      goal_topic_.c_str(), base_frame_.c_str(), map_resolution_m_,
      lattice_tolerance_m_, terrain_parameters_.edge_sample_spacing_m,
      terrain_parameters_.footprint_radius_m,
      terrain_parameters_.min_footprint_observed_ratio,
      terrain_parameters_.max_step_height_m, terrain_parameters_.max_slope_deg,
      path_invalid_confirmations_,
      max_grid_cells_);

    planning_worker_ = std::thread(&PlannerNode::planningWorker, this);
  }

  ~PlannerNode() override
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      stop_worker_ = true;
    }
    planning_cv_.notify_one();
    if (planning_worker_.joinable()) {
      planning_worker_.join();
    }
  }

private:
  struct MapState
  {
    std::shared_ptr<const TerrainSnapshot> snapshot;
    std::string frame_id;
    std::uint64_t content_hash{0U};
    std::uint64_t generation{0U};
  };

  struct ParsedCloud
  {
    std::vector<TerrainPoint> points;
    std::uint64_t content_hash{0U};
  };

  struct ResultPublication
  {
    bool published{false};
    std::size_t path_pose_count{0U};
    std::size_t rejected_total{0U};
    std::size_t rejected_shown{0U};
    bool rejected_truncated{false};
  };

  struct PlanRequest
  {
    geometry_msgs::msg::PoseStamped goal;
    std::shared_ptr<const MapState> map;
    std::uint64_t goal_epoch{0U};
    std::uint64_t request_sequence{0U};
    bool automatic_replan{false};
  };

  struct PendingGoal
  {
    geometry_msgs::msg::PoseStamped goal;
    std::uint64_t goal_epoch{0U};
  };

  struct ActivePlanState
  {
    std::uint64_t plan_id{0U};
    std::uint64_t goal_epoch{0U};
    std::uint64_t planned_generation{0U};
    std::uint64_t validated_generation{0U};
    geometry_msgs::msg::PoseStamped requested_goal;
    std::vector<TerrainPoint> dense_path;
    std::size_t progress_segment{0U};
    std::size_t soft_invalid_streak{0U};
    TerrainInvalidReason last_invalid_reason{TerrainInvalidReason::kNone};
    bool executable{false};
    bool auto_replan_attempted{false};
  };

  enum class PublishStatus
  {
    kPublished,
    kStaleGoalOrFrame,
    kSupersededMap,
  };

  static std::size_t positiveSizeParameter(
    const std::int64_t value, const std::string & name)
  {
    if (value <= 0) {
      throw std::invalid_argument(name + " must be > 0");
    }
    if (
      static_cast<std::uint64_t>(value) >
      static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
    {
      throw std::invalid_argument(name + " exceeds size_t");
    }
    return static_cast<std::size_t>(value);
  }

  static std::size_t nonnegativeSizeParameter(
    const std::int64_t value, const std::string & name)
  {
    if (value < 0) {
      throw std::invalid_argument(name + " must be >= 0");
    }
    if (
      static_cast<std::uint64_t>(value) >
      static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
    {
      throw std::invalid_argument(name + " exceeds size_t");
    }
    return static_cast<std::size_t>(value);
  }

  static void requireFinitePositive(const double value, const std::string & name)
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and > 0");
    }
  }

  static void requireFiniteNonnegative(
    const double value, const std::string & name)
  {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument(name + " must be finite and >= 0");
    }
  }

  void loadParameters()
  {
    planner_mode_name_ =
      declare_parameter<std::string>("planner_mode", "rrt_star");
    if (planner_mode_name_ == "wavefront") {
      planner_mode_ = PlannerMode::kWavefront;
    } else if (planner_mode_name_ == "rrt_star") {
      planner_mode_ = PlannerMode::kRrtStar;
    } else {
      throw std::invalid_argument(
              "planner_mode must be either 'wavefront' or 'rrt_star'");
    }

    input_cloud_topic_ = declare_parameter<std::string>(
      "input_cloud_topic", "/fastdem/mapping/cloud_global");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
    path_topic_ = declare_parameter<std::string>(
      "path_topic", "/rubi/heightmap_planner/path");
    debug_nodes_topic_ = declare_parameter<std::string>(
      "debug_nodes_topic", "/rubi/heightmap_planner/debug/nodes");
    debug_edges_topic_ = declare_parameter<std::string>(
      "debug_edges_topic", "/rubi/heightmap_planner/debug/edges");
    debug_rejected_topic_ = declare_parameter<std::string>(
      "debug_rejected_topic", "/rubi/heightmap_planner/debug/rejected");
    base_frame_ = declare_parameter<std::string>("base_frame", "body");

    map_resolution_m_ = declare_parameter<double>("map_resolution_m", 0.10);
    lattice_tolerance_m_ =
      declare_parameter<double>("lattice_tolerance_m", 0.02);
    const bool reject_duplicate_cells =
      declare_parameter<bool>("reject_duplicate_cells", true);
    max_grid_cells_ = positiveSizeParameter(
      declare_parameter<std::int64_t>("max_grid_cells", 5000000),
      "max_grid_cells");
    transform_timeout_s_ =
      declare_parameter<double>("transform_timeout_s", 0.25);

    terrain_parameters_.pca_radius_m =
      declare_parameter<double>("pca_analysis_radius_m", 0.30);
    terrain_parameters_.min_pca_points = positiveSizeParameter(
      declare_parameter<std::int64_t>("pca_min_points", 6),
      "pca_min_points");
    terrain_parameters_.footprint_radius_m =
      declare_parameter<double>("support_radius_m", 0.20);
    terrain_parameters_.min_footprint_observed_ratio =
      declare_parameter<double>("minimum_observed_support_ratio", 1.00);
    terrain_parameters_.max_slope_deg =
      declare_parameter<double>("max_slope_deg", 15.0);
    const double max_roughness_m =
      declare_parameter<double>("max_roughness_m", -1.0);
    terrain_parameters_.max_roughness_m =
      max_roughness_m < 0.0 ?
      std::numeric_limits<double>::infinity() : max_roughness_m;
    terrain_parameters_.max_step_height_m =
      declare_parameter<double>("max_step_height_m", 0.08);
    terrain_parameters_.edge_sample_spacing_m =
      declare_parameter<double>("edge_check_spacing_m", 0.05);
    terrain_parameters_.check_footprint_along_edge =
      declare_parameter<bool>("check_footprint_along_edge", true);

    planner_parameters_.node_sampling_distance_m =
      declare_parameter<double>("node_sampling_distance_m", 0.50);
    planner_parameters_.num_expansion_samples = positiveSizeParameter(
      declare_parameter<std::int64_t>("samples_per_expansion", 12),
      "samples_per_expansion");
    planner_parameters_.merge_radius_m =
      declare_parameter<double>("merge_radius_m", 0.25);
    planner_parameters_.neighbor_connection_radius_m =
      declare_parameter<double>("neighbor_connection_radius_m", 0.75);
    planner_parameters_.goal_connection_distance_m =
      declare_parameter<double>("goal_connection_distance_m", 0.75);
    planner_parameters_.max_nodes = positiveSizeParameter(
      declare_parameter<std::int64_t>("max_nodes", 4000), "max_nodes");
    planner_parameters_.max_expansions = positiveSizeParameter(
      declare_parameter<std::int64_t>("max_expansions", 4000),
      "max_expansions");
    planner_parameters_.max_build_time_ms = positiveSizeParameter(
      declare_parameter<std::int64_t>("max_build_time_ms", 2000),
      "max_build_time_ms");
    planner_parameters_.stop_when_goal_connected =
      declare_parameter<bool>("stop_when_goal_connected", true);
    planner_parameters_.slope_normalization_deg =
      terrain_parameters_.max_slope_deg;
    planner_parameters_.risk_weights.distance =
      declare_parameter<double>("distance_weight", 1.0);
    planner_parameters_.risk_weights.slope =
      declare_parameter<double>("slope_risk_weight", 3.0);
    planner_parameters_.risk_weights.step =
      declare_parameter<double>("step_risk_weight", 0.0);
    const double roughness_risk_weight =
      declare_parameter<double>("roughness_risk_weight", 0.0);

    rrt_star_parameters_.max_iterations = positiveSizeParameter(
      declare_parameter<std::int64_t>("rrt_star.max_iterations", 5000),
      "rrt_star.max_iterations");
    rrt_star_parameters_.goal_bias =
      declare_parameter<double>("rrt_star.goal_bias", 0.05);
    rrt_star_parameters_.steer_distance_m =
      declare_parameter<double>("rrt_star.steer_distance_m", 0.50);
    rrt_star_parameters_.rewire_radius_min_m =
      declare_parameter<double>("rrt_star.rewire_radius_min_m", 0.30);
    rrt_star_parameters_.rewire_radius_max_m =
      declare_parameter<double>("rrt_star.rewire_radius_max_m", 1.00);
    rrt_star_parameters_.goal_connection_distance_m =
      declare_parameter<double>("rrt_star.goal_connection_distance_m", 0.75);
    rrt_star_parameters_.max_nodes = positiveSizeParameter(
      declare_parameter<std::int64_t>("rrt_star.max_nodes", 4000),
      "rrt_star.max_nodes");
    rrt_star_parameters_.max_planning_time_ms = nonnegativeSizeParameter(
      declare_parameter<std::int64_t>("rrt_star.max_planning_time_ms", 2000),
      "rrt_star.max_planning_time_ms");
    rrt_star_parameters_.stop_on_first_solution =
      declare_parameter<bool>("rrt_star.stop_on_first_solution", false);
    rrt_star_parameters_.random_seed = static_cast<std::uint64_t>(
      nonnegativeSizeParameter(
        declare_parameter<std::int64_t>("rrt_star.random_seed", 42),
        "rrt_star.random_seed"));
    rrt_star_parameters_.slope_normalization_deg =
      terrain_parameters_.max_slope_deg;
    rrt_star_parameters_.risk_weights = planner_parameters_.risk_weights;

    path_output_spacing_m_ =
      declare_parameter<double>("path_output_spacing_m", 0.05);
    node_marker_scale_m_ =
      declare_parameter<double>("node_marker_scale_m", 0.08);
    edge_marker_width_m_ =
      declare_parameter<double>("edge_marker_width_m", 0.025);
    path_marker_width_m_ =
      declare_parameter<double>("path_marker_width_m", 0.08);
    rejected_marker_scale_m_ =
      declare_parameter<double>("rejected_marker_scale_m", 0.07);
    max_rejected_markers_ = positiveSizeParameter(
      declare_parameter<std::int64_t>("max_rejected_markers", 5000),
      "max_rejected_markers");
    path_invalid_confirmations_ = positiveSizeParameter(
      declare_parameter<std::int64_t>("path_invalid_confirmations", 2),
      "path_invalid_confirmations");

    const std::vector<std::pair<std::string, std::string>> named_values{
      {"input_cloud_topic", input_cloud_topic_},
      {"goal_topic", goal_topic_},
      {"path_topic", path_topic_},
      {"debug_nodes_topic", debug_nodes_topic_},
      {"debug_edges_topic", debug_edges_topic_},
      {"debug_rejected_topic", debug_rejected_topic_},
      {"base_frame", base_frame_}};
    for (const auto & item : named_values) {
      if (item.second.empty()) {
        throw std::invalid_argument(item.first + " must not be empty");
      }
    }
    requireFinitePositive(map_resolution_m_, "map_resolution_m");
    requireFiniteNonnegative(lattice_tolerance_m_, "lattice_tolerance_m");
    if (lattice_tolerance_m_ >= 0.5 * map_resolution_m_) {
      throw std::invalid_argument(
              "lattice_tolerance_m must be less than half map_resolution_m");
    }
    requireFinitePositive(transform_timeout_s_, "transform_timeout_s");
    requireFinitePositive(path_output_spacing_m_, "path_output_spacing_m");
    requireFinitePositive(node_marker_scale_m_, "node_marker_scale_m");
    requireFinitePositive(edge_marker_width_m_, "edge_marker_width_m");
    requireFinitePositive(path_marker_width_m_, "path_marker_width_m");
    requireFinitePositive(rejected_marker_scale_m_, "rejected_marker_scale_m");
    requireFiniteNonnegative(roughness_risk_weight, "roughness_risk_weight");
    if (!reject_duplicate_cells) {
      throw std::invalid_argument(
              "reject_duplicate_cells=false conflicts with the strict immutable "
              "TerrainSnapshot contract");
    }
    if (roughness_risk_weight > 0.0) {
      throw std::invalid_argument(
              "roughness_risk_weight is nonzero, but the current planner cores "
              "have no roughness soft-cost field");
    }

    // Constructors provide the authoritative validation for the shared core APIs.
    const TerrainSnapshot validation_snapshot(
      map_resolution_m_, 0.0, 0.0, 1U, 1U,
      std::vector<double>{0.0}, std::vector<std::uint8_t>{1U});
    const TerrainEvaluator validation_evaluator(
      validation_snapshot, terrain_parameters_);
    (void)validation_evaluator;
    if (planner_mode_ == PlannerMode::kWavefront) {
      wavefront_planner_ =
        std::make_unique<WavefrontPlanner>(planner_parameters_);
    } else {
      rrt_star_planner_ =
        std::make_unique<RrtStarPlanner>(rrt_star_parameters_);
    }
  }

  static const sensor_msgs::msg::PointField & requireFloatField(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const std::string & name)
  {
    const sensor_msgs::msg::PointField * match = nullptr;
    for (const auto & field : cloud.fields) {
      if (field.name != name) {
        continue;
      }
      if (match != nullptr) {
        throw std::invalid_argument("PointCloud2 has duplicate '" + name + "' fields");
      }
      match = &field;
    }
    if (match == nullptr) {
      throw std::invalid_argument("PointCloud2 is missing required '" + name + "' field");
    }
    if (
      match->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      match->count != 1U)
    {
      throw std::invalid_argument(
              "PointCloud2 field '" + name + "' must be FLOAT32 count=1");
    }
    if (
      match->offset > cloud.point_step ||
      cloud.point_step - match->offset < sizeof(float))
    {
      throw std::invalid_argument(
              "PointCloud2 field '" + name + "' exceeds point_step");
    }
    return *match;
  }

  static float readFloat(
    const std::uint8_t * data, const bool message_big_endian,
    std::uint32_t & host_bits)
  {
    std::uint32_t bits = 0U;
    std::memcpy(&bits, data, sizeof(bits));
    if (message_big_endian != hostIsBigEndian()) {
      bits = byteSwap32(bits);
    }
    host_bits = bits;
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  void preflightLattice(const std::vector<TerrainPoint> & points) const
  {
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    for (const auto & point : points) {
      min_x = std::min(min_x, point.x);
      min_y = std::min(min_y, point.y);
    }

    std::size_t max_ix = 0U;
    std::size_t max_iy = 0U;
    for (const auto & point : points) {
      const double grid_x = (point.x - min_x) / map_resolution_m_;
      const double grid_y = (point.y - min_y) / map_resolution_m_;
      const double rounded_x = std::round(grid_x);
      const double rounded_y = std::round(grid_y);
      if (
        std::abs(point.x - (min_x + rounded_x * map_resolution_m_)) >
        lattice_tolerance_m_ ||
        std::abs(point.y - (min_y + rounded_y * map_resolution_m_)) >
        lattice_tolerance_m_)
      {
        throw std::invalid_argument(
                "PointCloud2 contains a point off the configured elevation lattice");
      }
      if (
        rounded_x < 0.0 || rounded_y < 0.0 ||
        rounded_x >
        static_cast<double>(std::numeric_limits<std::size_t>::max() - 1U) ||
        rounded_y >
        static_cast<double>(std::numeric_limits<std::size_t>::max() - 1U))
      {
        throw std::invalid_argument("PointCloud2 lattice index exceeds size_t");
      }
      max_ix = std::max(max_ix, static_cast<std::size_t>(rounded_x));
      max_iy = std::max(max_iy, static_cast<std::size_t>(rounded_y));
    }

    const std::size_t size_x = max_ix + 1U;
    const std::size_t size_y = max_iy + 1U;
    if (size_x > std::numeric_limits<std::size_t>::max() / size_y) {
      throw std::invalid_argument("PointCloud2 lattice dimensions overflow size_t");
    }
    const std::size_t cell_count = size_x * size_y;
    if (cell_count > max_grid_cells_) {
      throw std::invalid_argument(
              "PointCloud2 dense lattice would require " +
              std::to_string(cell_count) + " cells, exceeding max_grid_cells=" +
              std::to_string(max_grid_cells_));
    }
  }

  ParsedCloud parseCloud(const sensor_msgs::msg::PointCloud2 & cloud) const
  {
    if (cloud.header.frame_id.empty()) {
      throw std::invalid_argument("PointCloud2 header.frame_id must not be empty");
    }
    if (cloud.height != 1U) {
      throw std::invalid_argument(
              "FastDEM elevation PointCloud2 must be unorganized (height == 1)");
    }
    if (cloud.width == 0U) {
      throw std::invalid_argument("FastDEM elevation PointCloud2 is empty");
    }
    if (cloud.point_step == 0U) {
      throw std::invalid_argument("PointCloud2 point_step must be > 0");
    }

    const std::size_t width = static_cast<std::size_t>(cloud.width);
    const std::size_t point_step = static_cast<std::size_t>(cloud.point_step);
    if (width > std::numeric_limits<std::size_t>::max() / point_step) {
      throw std::invalid_argument("PointCloud2 width * point_step overflows size_t");
    }
    const std::size_t expected_row_step = width * point_step;
    if (expected_row_step > std::numeric_limits<std::uint32_t>::max()) {
      throw std::invalid_argument("PointCloud2 row_step cannot represent its payload");
    }
    if (static_cast<std::size_t>(cloud.row_step) != expected_row_step) {
      throw std::invalid_argument(
              "PointCloud2 row_step must equal width * point_step exactly");
    }
    if (cloud.data.size() != expected_row_step) {
      throw std::invalid_argument(
              "PointCloud2 data.size() must equal row_step exactly for height == 1");
    }
    if (width > max_grid_cells_) {
      throw std::invalid_argument(
              "PointCloud2 observed point count exceeds max_grid_cells");
    }

    const auto & x_field = requireFloatField(cloud, "x");
    const auto & y_field = requireFloatField(cloud, "y");
    const auto & z_field = requireFloatField(cloud, "z");

    ParsedCloud parsed;
    parsed.points.reserve(width);
    parsed.content_hash = kFnvOffset;
    hashString(parsed.content_hash, cloud.header.frame_id);
    for (std::size_t index = 0U; index < width; ++index) {
      const std::size_t base = index * point_step;
      std::uint32_t x_bits = 0U;
      std::uint32_t y_bits = 0U;
      std::uint32_t z_bits = 0U;
      const float x = readFloat(
        cloud.data.data() + base + x_field.offset, cloud.is_bigendian, x_bits);
      const float y = readFloat(
        cloud.data.data() + base + y_field.offset, cloud.is_bigendian, y_bits);
      const float z = readFloat(
        cloud.data.data() + base + z_field.offset, cloud.is_bigendian, z_bits);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        throw std::invalid_argument(
                "PointCloud2 contains a non-finite x/y/z value at point " +
                std::to_string(index));
      }
      hashUint32(parsed.content_hash, x_bits);
      hashUint32(parsed.content_hash, y_bits);
      hashUint32(parsed.content_hash, z_bits);
      parsed.points.push_back(
        TerrainPoint{
          static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)});
    }
    preflightLattice(parsed.points);
    return parsed;
  }

  bool enqueueRequestLocked(
    const geometry_msgs::msg::PoseStamped & goal,
    const std::shared_ptr<const MapState> & map,
    const std::uint64_t goal_epoch,
    const bool automatic_replan)
  {
    if (map == nullptr) {
      return false;
    }
    if (
      queued_request_.has_value() &&
      !shouldReplaceQueuedRequest(
        queued_request_->automatic_replan, automatic_replan))
    {
      return false;
    }
    queued_request_ = PlanRequest{
      goal, map, goal_epoch, ++request_sequence_, automatic_replan};
    return true;
  }

  void requeueLatestMap(const PlanRequest & request)
  {
    bool notify = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (
        goal_epoch_ == request.goal_epoch && map_state_ != nullptr &&
        map_state_->frame_id == request.map->frame_id)
      {
        notify = enqueueRequestLocked(
          request.goal, map_state_, request.goal_epoch, true);
      }
    }
    if (notify) {
      planning_cv_.notify_one();
    }
  }

  bool isImmediateInvalidation(const TerrainInvalidReason reason) const noexcept
  {
    return
      reason == TerrainInvalidReason::kStepLimit ||
      reason == TerrainInvalidReason::kSlopeLimit ||
      reason == TerrainInvalidReason::kInvalidInput;
  }

  std::vector<Point2D> xyRoute(const std::vector<TerrainPoint> & path) const
  {
    std::vector<Point2D> route;
    route.reserve(path.size());
    for (const TerrainPoint & point : path) {
      route.push_back(Point2D{point.x, point.y});
    }
    return route;
  }

  PathValidationResult validateDensePath(
    const std::vector<TerrainPoint> & path,
    const Point2D robot_position,
    const std::size_t progress_segment,
    const TerrainEvaluator & terrain) const
  {
    if (path.size() == 1U) {
      const NodeEvaluation node = terrain.evaluateNode(Point2D{path[0U].x, path[0U].y});
      return PathValidationResult{
        node.valid && std::isfinite(node.elevation_m), node.reason, 0U, 0U, 0.0};
    }
    return validateRemainingPath(
      xyRoute(path), robot_position, progress_segment, terrain);
  }

  bool updatePathElevations(
    std::vector<TerrainPoint> & path, const TerrainSnapshot & snapshot) const
  {
    for (TerrainPoint & point : path) {
      const auto elevation = snapshot.elevationAt(point.x, point.y);
      if (!elevation) {
        return false;
      }
      point.z = *elevation;
    }
    return true;
  }

  void revalidateActivePlan(
    const std::shared_ptr<const MapState> & state,
    const ActivePlanState & active)
  {
    try {
      const auto timeout = tf2::durationFromSec(transform_timeout_s_);
      const auto transform = tf_buffer_->lookupTransform(
        state->frame_id, base_frame_, tf2::TimePointZero, timeout);
      const Point2D robot_position{
        transform.transform.translation.x, transform.transform.translation.y};
      if (!finitePoint(robot_position)) {
        RCLCPP_WARN(get_logger(), "Skipped path revalidation: base TF has non-finite XY");
        return;
      }
      const TerrainEvaluator terrain(*state->snapshot, terrain_parameters_);
      const PathValidationResult validation = validateDensePath(
        active.dense_path, robot_position, active.progress_segment, terrain);

      bool notify = false;
      bool confirmed = false;
      {
        std::unique_lock<std::mutex> state_lock(state_mutex_);
        if (
          map_state_ != state || !active_plan_.has_value() ||
          active_plan_->plan_id != active.plan_id ||
          !active_plan_->executable)
        {
          return;
        }
        if (validation.valid) {
          active_plan_->progress_segment = validation.progress_segment;
          active_plan_->validated_generation = state->generation;
          active_plan_->soft_invalid_streak = 0U;
          active_plan_->last_invalid_reason = TerrainInvalidReason::kNone;
          return;
        }

        if (isImmediateInvalidation(validation.reason)) {
          confirmed = true;
        } else if (active_plan_->last_invalid_reason == validation.reason) {
          ++active_plan_->soft_invalid_streak;
          confirmed = active_plan_->soft_invalid_streak >= path_invalid_confirmations_;
        } else {
          active_plan_->last_invalid_reason = validation.reason;
          active_plan_->soft_invalid_streak = 1U;
        }
        if (!confirmed) {
          RCLCPP_INFO(
            get_logger(),
            "Retained active Path after soft corridor failure reason=%s streak=%zu/%zu",
            std::string(toString(validation.reason)).c_str(),
            active_plan_->soft_invalid_streak, path_invalid_confirmations_);
          return;
        }

        active_plan_->executable = false;
        std::lock_guard<std::mutex> output_lock(output_mutex_);
        publishEmptyPathUnlocked(state->frame_id);
        last_empty_path_goal_epoch_ = active_plan_->goal_epoch;
        if (!active_plan_->auto_replan_attempted) {
          active_plan_->auto_replan_attempted = true;
          notify = enqueueRequestLocked(
            active_plan_->requested_goal, state, active_plan_->goal_epoch, true);
        }
      }
      RCLCPP_WARN(
        get_logger(),
        "Invalidated active Path: reason=%s failing_segment=%zu; auto_replan=%s",
        std::string(toString(validation.reason)).c_str(), validation.failing_segment,
        notify ? "queued" : "not_queued");
      if (notify) {
        planning_cv_.notify_one();
      }
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN(
        get_logger(), "Skipped path revalidation because map<-base TF failed: %s",
        error.what());
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        get_logger(), "Skipped path revalidation because evaluation failed: %s",
        error.what());
    }
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    try {
      ParsedCloud parsed = parseCloud(*cloud);
      auto snapshot = std::make_shared<const TerrainSnapshot>(
        TerrainSnapshot::fromPoints(
          parsed.points, map_resolution_m_, lattice_tolerance_m_,
          max_grid_cells_));
      if (snapshot->cellCount() > max_grid_cells_) {
        throw std::invalid_argument(
                "TerrainSnapshot exceeds max_grid_cells after construction");
      }

      bool frame_changed = false;
      bool notify = false;
      std::shared_ptr<const MapState> accepted_state;
      std::optional<ActivePlanState> active_to_validate;
      {
        std::unique_lock<std::mutex> state_lock(state_mutex_);
        if (
          map_state_ != nullptr &&
          map_state_->content_hash == parsed.content_hash &&
          map_state_->frame_id == cloud->header.frame_id)
        {
          return;
        }
        frame_changed =
          map_state_ != nullptr && map_state_->frame_id != cloud->header.frame_id;
        const std::uint64_t generation =
          map_state_ == nullptr ? 1U : map_state_->generation + 1U;
        auto next_state = std::make_shared<MapState>();
        next_state->snapshot = std::move(snapshot);
        next_state->frame_id = cloud->header.frame_id;
        next_state->content_hash = parsed.content_hash;
        next_state->generation = generation;
        map_state_ = next_state;
        accepted_state = next_state;

        if (frame_changed) {
          active_plan_.reset();
          std::lock_guard<std::mutex> output_lock(output_mutex_);
          publishFullResetUnlocked(accepted_state->frame_id);
          last_empty_path_goal_epoch_ = goal_epoch_;
        } else if (pending_goal_) {
          notify = enqueueRequestLocked(
            pending_goal_->goal, accepted_state, pending_goal_->goal_epoch, false);
          pending_goal_.reset();
        } else if (active_plan_.has_value() && active_plan_->executable) {
          active_to_validate = *active_plan_;
        }
      }
      if (notify) {
        planning_cv_.notify_one();
      }
      RCLCPP_INFO(
        get_logger(),
        "Accepted elevation snapshot generation=%lu frame='%s': "
        "observed=%zu, grid=%zux%zu (%zu cells), hash=%016lx; "
        "retained last Path/debug output until corridor revalidation",
        static_cast<unsigned long>(accepted_state->generation),
        accepted_state->frame_id.c_str(),
        accepted_state->snapshot->observedCount(),
        accepted_state->snapshot->sizeX(), accepted_state->snapshot->sizeY(),
        accepted_state->snapshot->cellCount(),
        static_cast<unsigned long>(accepted_state->content_hash));
      if (frame_changed) {
        RCLCPP_INFO(
          get_logger(), "Elevation snapshot frame changed; reset Path and debug markers");
      }
      if (active_to_validate) {
        revalidateActivePlan(accepted_state, *active_to_validate);
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(),
        "Rejected FastDEM elevation snapshot: %s. The previous accepted map, if any, "
        "remains active.", error.what());
    }
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal)
  {
    bool notify = false;
    bool invalidated = false;
    bool no_map = false;
    std::string frame_id{"map"};
    {
      std::unique_lock<std::mutex> state_lock(state_mutex_);
      const std::uint64_t goal_epoch = ++goal_epoch_;
      if (map_state_ == nullptr) {
        no_map = true;
        pending_goal_ = PendingGoal{*goal, goal_epoch};
      } else {
        frame_id = map_state_->frame_id;
        if (active_plan_.has_value() && active_plan_->executable) {
          active_plan_->executable = false;
          invalidated = true;
          std::lock_guard<std::mutex> output_lock(output_mutex_);
          publishEmptyPathUnlocked(frame_id);
          last_empty_path_goal_epoch_ = goal_epoch;
        }
        notify = enqueueRequestLocked(*goal, map_state_, goal_epoch, false);
      }
    }
    if (no_map) {
      RCLCPP_WARN(
        get_logger(), "No accepted elevation map yet; stored the latest goal as pending");
      return;
    }
    if (invalidated) {
      RCLCPP_INFO(get_logger(), "External Goal invalidated the previously executable Path");
    }
    if (notify) {
      planning_cv_.notify_one();
    }
  }

  void planningWorker()
  {
    while (true) {
      PlanRequest request;
      {
        std::unique_lock<std::mutex> lock(state_mutex_);
        planning_cv_.wait(
          lock, [this]() {
            return stop_worker_ || queued_request_.has_value();
          });
        if (stop_worker_) {
          return;
        }
        request = *queued_request_;
        queued_request_.reset();
      }
      processGoal(request);
    }
  }

  void processGoal(const PlanRequest & request)
  {
    if (request.goal.header.frame_id.empty()) {
      failAndClear(request, "Goal header.frame_id is empty");
      return;
    }
    try {
      const auto normalized_goal_orientation =
        normalizedQuaternion(request.goal.pose.orientation);
      if (!normalized_goal_orientation) {
        failAndClear(
          request,
          "Goal orientation quaternion has non-finite components or an invalid norm");
        return;
      }
      geometry_msgs::msg::PoseStamped normalized_goal = request.goal;
      normalized_goal.pose.orientation = *normalized_goal_orientation;

      const auto timeout = tf2::durationFromSec(transform_timeout_s_);
      const auto start_transform = tf_buffer_->lookupTransform(
        request.map->frame_id, base_frame_, tf2::TimePointZero, timeout);
      const Point2D start{
        start_transform.transform.translation.x,
        start_transform.transform.translation.y};
      geometry_msgs::msg::PoseStamped goal_in_map;
      if (request.goal.header.frame_id == request.map->frame_id) {
        goal_in_map = normalized_goal;
      } else {
        const auto goal_transform = tf_buffer_->lookupTransform(
          request.map->frame_id, request.goal.header.frame_id,
          tf2::TimePointZero, timeout);
        tf2::doTransform(normalized_goal, goal_in_map, goal_transform);
      }
      const auto transformed_goal_orientation =
        normalizedQuaternion(goal_in_map.pose.orientation);
      if (!transformed_goal_orientation) {
        failAndClear(
          request, "Transformed Goal orientation quaternion has an invalid norm");
        return;
      }
      goal_in_map.pose.orientation = *transformed_goal_orientation;
      const Point2D goal_xy{
        goal_in_map.pose.position.x, goal_in_map.pose.position.y};
      if (!finitePoint(start) || !finitePoint(goal_xy)) {
        failAndClear(request, "Start or transformed goal contains non-finite XY");
        return;
      }

      const TerrainEvaluator terrain(*request.map->snapshot, terrain_parameters_);
      PlanResult result = planner_mode_ == PlannerMode::kWavefront ?
        wavefront_planner_->plan(terrain, start, goal_xy) :
        rrt_star_planner_->plan(terrain, start, goal_xy);
      std::vector<TerrainPoint> dense_path;
      if (result.success) {
        std::string validation_error;
        if (!revalidateGraphPath(result, terrain, validation_error)) {
          failAndClear(request, "Final graph path revalidation failed: " + validation_error);
          return;
        }
        if (!densifyPath(result, *request.map->snapshot, dense_path, validation_error)) {
          failAndClear(request, "Path densification failed: " + validation_error);
          return;
        }
      }

      std::shared_ptr<const MapState> output_state = request.map;
      if (result.success) {
        for (std::size_t attempt = 0U; attempt < 3U; ++attempt) {
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (goal_epoch_ != request.goal_epoch || map_state_ == nullptr ||
              map_state_->frame_id != request.map->frame_id)
            {
              return;
            }
            output_state = map_state_;
          }
          if (output_state != request.map) {
            const TerrainEvaluator latest_terrain(
              *output_state->snapshot, terrain_parameters_);
            const PathValidationResult validation = validateDensePath(
              dense_path, Point2D{dense_path.front().x, dense_path.front().y},
              0U, latest_terrain);
            if (!validation.valid ||
              !updatePathElevations(dense_path, *output_state->snapshot))
            {
              RCLCPP_INFO(
                get_logger(),
                "Discarded stale-map candidate after generation=%lu corridor revalidation",
                static_cast<unsigned long>(output_state->generation));
              requeueLatestMap(request);
              return;
            }
          }
          const auto publication = publishResultForRequest(
            request, output_state, result, dense_path,
            goal_in_map.pose.orientation, now());
          if (publication.first == PublishStatus::kPublished) {
            logPlanningResult(request, output_state, result, publication.second);
            return;
          }
          if (publication.first == PublishStatus::kStaleGoalOrFrame) {
            return;
          }
        }
        requeueLatestMap(request);
        return;
      }

      const auto publication = publishResultForRequest(
        request, output_state, result, dense_path,
        goal_in_map.pose.orientation, now());
      if (publication.first == PublishStatus::kSupersededMap) {
        requeueLatestMap(request);
      } else if (publication.first == PublishStatus::kPublished) {
        logPlanningResult(request, output_state, result, publication.second);
      }
    } catch (const tf2::TransformException & error) {
      failAndClear(
        request, "TF lookup/goal transform failed for map<-" + base_frame_ + ": " +
        error.what());
    } catch (const std::exception & error) {
      failAndClear(request, std::string("Planning exception: ") + error.what());
    }
  }

  bool revalidateGraphPath(
    const PlanResult & result,
    const TerrainEvaluator & terrain,
    std::string & error) const
  {
    if (result.path_node_ids.empty()) {
      error = "path has no graph node";
      return false;
    }
    for (std::size_t index = 0U; index < result.path_node_ids.size(); ++index) {
      const NodeId node_id = result.path_node_ids[index];
      if (node_id >= result.nodes.size()) {
        error = "path contains an out-of-range node id";
        return false;
      }
      const auto & point = result.nodes[node_id].point;
      const NodeEvaluation node = terrain.evaluateNode(Point2D{point.x, point.y});
      if (!node.valid || !std::isfinite(node.elevation_m)) {
        error =
          "node " + std::to_string(node_id) + " is now invalid: " +
          std::string(toString(node.reason));
        return false;
      }
      if (index == 0U) {
        continue;
      }

      const NodeId previous_id = result.path_node_ids[index - 1U];
      bool graph_edge_exists = false;
      for (const auto & edge : result.edges) {
        if (
          (edge.from == previous_id && edge.to == node_id) ||
          (edge.from == node_id && edge.to == previous_id))
        {
          graph_edge_exists = edge.terrain.valid && std::isfinite(edge.cost);
          break;
        }
      }
      if (!graph_edge_exists) {
        error = "path references a missing or invalid graph edge";
        return false;
      }
      const auto & previous = result.nodes[previous_id].point;
      const EdgeEvaluation edge = terrain.evaluateEdge(
        Point2D{previous.x, previous.y}, Point2D{point.x, point.y});
      if (!edge.valid || !std::isfinite(edge.cost)) {
        error =
          "edge " + std::to_string(previous_id) + "->" +
          std::to_string(node_id) + " is invalid: " +
          std::string(toString(edge.reason));
        return false;
      }
    }
    return true;
  }

  bool densifyPath(
    const PlanResult & result,
    const TerrainSnapshot & snapshot,
    std::vector<TerrainPoint> & dense,
    std::string & error) const
  {
    dense.clear();
    if (result.path_node_ids.size() == 1U) {
      const NodeId id = result.path_node_ids.front();
      if (id >= result.nodes.size()) {
        error = "single-node path contains an out-of-range node id";
        return false;
      }
      const auto & point = result.nodes[id].point;
      const auto elevation = snapshot.elevationAt(point.x, point.y);
      if (!elevation) {
        error = "single-node path lies in unknown or out-of-bounds terrain";
        return false;
      }
      dense.push_back(TerrainPoint{point.x, point.y, *elevation});
      return true;
    }
    for (std::size_t segment = 1U; segment < result.path_node_ids.size(); ++segment) {
      const auto & from = result.nodes[result.path_node_ids[segment - 1U]].point;
      const auto & to = result.nodes[result.path_node_ids[segment]].point;
      const double dx = to.x - from.x;
      const double dy = to.y - from.y;
      const double length = std::hypot(dx, dy);
      if (!std::isfinite(length) || length <= kEpsilon) {
        error = "zero-length or non-finite graph segment";
        return false;
      }
      const std::size_t subdivisions = std::max<std::size_t>(
        1U, static_cast<std::size_t>(
          std::ceil(length / path_output_spacing_m_)));
      const std::size_t first_sample = dense.empty() ? 0U : 1U;
      for (std::size_t sample = first_sample; sample <= subdivisions; ++sample) {
        const double ratio =
          static_cast<double>(sample) / static_cast<double>(subdivisions);
        const double x = from.x + ratio * dx;
        const double y = from.y + ratio * dy;
        const auto elevation = snapshot.elevationAt(x, y);
        if (!elevation) {
          error = "densified pose fell in an unknown or out-of-bounds cell";
          return false;
        }
        dense.push_back(TerrainPoint{x, y, *elevation});
      }
    }
    if (dense.empty()) {
      error = "densified path is empty";
      return false;
    }
    return true;
  }

  nav_msgs::msg::Path makePathMessage(
    const std::vector<TerrainPoint> & points,
    const std::string & frame_id,
    const geometry_msgs::msg::Quaternion & goal_orientation_in_map,
    const rclcpp::Time & stamp) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = stamp;
    path.poses.reserve(points.size());
    for (std::size_t index = 0U; index < points.size(); ++index) {
      double yaw = 0.0;
      if (points.size() > 1U) {
        const TerrainPoint & tangent_from =
          index + 1U < points.size() ? points[index] : points[index - 1U];
        const TerrainPoint & tangent_to =
          index + 1U < points.size() ? points[index + 1U] : points[index];
        yaw = std::atan2(
          tangent_to.y - tangent_from.y,
          tangent_to.x - tangent_from.x);
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = points[index].x;
      pose.pose.position.y = points[index].y;
      pose.pose.position.z = points[index].z;
      pose.pose.orientation.z = std::sin(0.5 * yaw);
      pose.pose.orientation.w = std::cos(0.5 * yaw);
      path.poses.push_back(std::move(pose));
    }
    if (!path.poses.empty()) {
      path.poses.back().pose.orientation = goal_orientation_in_map;
    }
    return path;
  }

  std::pair<PublishStatus, ResultPublication> publishResultForRequest(
    const PlanRequest & request,
    const std::shared_ptr<const MapState> & expected_state,
    const PlanResult & result,
    const std::vector<TerrainPoint> & dense_path,
    const geometry_msgs::msg::Quaternion & goal_orientation_in_map,
    const rclcpp::Time & stamp)
  {
    const nav_msgs::msg::Path path =
      makePathMessage(
      dense_path, expected_state->frame_id, goal_orientation_in_map, stamp);
    PlannerVisualizationParameters visualization_parameters;
    visualization_parameters.marker_namespace = planner_mode_name_;
    visualization_parameters.node_marker_scale_m = node_marker_scale_m_;
    visualization_parameters.edge_marker_width_m = edge_marker_width_m_;
    visualization_parameters.path_marker_width_m = path_marker_width_m_;
    visualization_parameters.rejected_marker_scale_m =
      rejected_marker_scale_m_;
    visualization_parameters.max_rejected_markers = max_rejected_markers_;
    const PlannerVisualizationSnapshot visualization =
      makePlannerVisualization(
      result, *expected_state->snapshot, dense_path, expected_state->frame_id,
      static_cast<builtin_interfaces::msg::Time>(stamp),
      visualization_parameters);

    ResultPublication publication;
    publication.path_pose_count = path.poses.size();
    publication.rejected_total = visualization.rejected_total;
    publication.rejected_shown = visualization.rejected_shown;
    publication.rejected_truncated = visualization.rejected_truncated;
    {
      // Lock order is always state_mutex_ then output_mutex_. Heavy TF,
      // planning, visualization, and terrain evaluation happen before this.
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      if (map_state_ == nullptr || !mayCommitAfterRevalidation(
          PlanLifecycleToken{
          request.goal_epoch, request.map->generation, request.map->frame_id},
          goal_epoch_, map_state_->frame_id))
      {
        return {PublishStatus::kStaleGoalOrFrame, publication};
      }
      if (map_state_ != expected_state) {
        return {PublishStatus::kSupersededMap, publication};
      }
      if (result.success) {
        active_plan_ = ActivePlanState{
          ++plan_id_sequence_, request.goal_epoch, request.map->generation,
          expected_state->generation, request.goal, dense_path, 0U, 0U,
          TerrainInvalidReason::kNone, true, false};
      }
      const bool publish_path =
        result.success || last_empty_path_goal_epoch_ != request.goal_epoch;
      std::lock_guard<std::mutex> output_lock(output_mutex_);
      if (publish_path) {
        path_publisher_->publish(path);
        if (!result.success) {
          last_empty_path_goal_epoch_ = request.goal_epoch;
        }
      }
      nodes_publisher_->publish(visualization.nodes);
      edges_publisher_->publish(visualization.edges);
      rejected_publisher_->publish(visualization.rejected);
      publication.published = true;
    }
    if (publication.rejected_truncated) {
      RCLCPP_WARN(
        get_logger(),
        "Rejected visualization truncated: map_generation=%lu "
        "rejected_shown=%zu rejected_total=%zu cap=%zu",
        static_cast<unsigned long>(expected_state->generation),
        publication.rejected_shown, publication.rejected_total,
        max_rejected_markers_);
    }
    return {PublishStatus::kPublished, publication};
  }

  void publishEmptyPathUnlocked(const std::string & frame_id)
  {
    const auto stamp = now();
    const std::string safe_frame = frame_id.empty() ? "map" : frame_id;
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = safe_frame;
    empty_path.header.stamp = stamp;
    path_publisher_->publish(empty_path);
  }

  void publishDebugDeleteAllUnlocked(const std::string & frame_id)
  {
    const auto stamp = now();
    const std::string safe_frame = frame_id.empty() ? "map" : frame_id;
    const visualization_msgs::msg::MarkerArray clear =
      makeDeleteAllMarkerArray(
      safe_frame, static_cast<builtin_interfaces::msg::Time>(stamp));
    nodes_publisher_->publish(clear);
    edges_publisher_->publish(clear);
    rejected_publisher_->publish(clear);
  }

  void publishFullResetUnlocked(const std::string & frame_id)
  {
    publishEmptyPathUnlocked(frame_id);
    publishDebugDeleteAllUnlocked(frame_id);
  }

  void failAndClear(
    const PlanRequest & request,
    const std::string & message)
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (
      map_state_ == nullptr || map_state_ != request.map ||
      !mayCommitAfterRevalidation(
        PlanLifecycleToken{
        request.goal_epoch, request.map->generation, request.map->frame_id},
        goal_epoch_, map_state_->frame_id))
    {
      return;
    }
    active_plan_.reset();
    std::lock_guard<std::mutex> output_lock(output_mutex_);
    publishFullResetUnlocked(request.map->frame_id);
    last_empty_path_goal_epoch_ = request.goal_epoch;
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  }

  void logPlanningResult(
    const PlanRequest & request,
    const std::shared_ptr<const MapState> & output_state,
    const PlanResult & result,
    const ResultPublication & publication) const
  {
    if (result.success) {
      RCLCPP_INFO(
        get_logger(),
        "Planning result: mode=%s planned_generation=%lu validated_generation=%lu "
        "success=true termination=%s nodes=%zu edges=%zu rejected_total=%zu "
        "rejected_shown=%zu expansions=%zu build_time_ms=%.2f path_pose_count=%zu",
        planner_mode_name_.c_str(),
        static_cast<unsigned long>(request.map->generation),
        static_cast<unsigned long>(output_state->generation),
        terminationName(result.termination).c_str(), result.nodes.size(),
        result.edges.size(), publication.rejected_total, publication.rejected_shown,
        result.expansions, result.build_time_ms, publication.path_pose_count);
      return;
    }
    RCLCPP_WARN(
      get_logger(),
      "Planning result: mode=%s map_generation=%lu success=false termination=%s "
      "nodes=%zu edges=%zu rejected_total=%zu rejected_shown=%zu expansions=%zu "
      "build_time_ms=%.2f path_pose_count=%zu message='%s'",
      planner_mode_name_.c_str(),
      static_cast<unsigned long>(request.map->generation),
      terminationName(result.termination).c_str(), result.nodes.size(), result.edges.size(),
      publication.rejected_total, publication.rejected_shown, result.expansions,
      result.build_time_ms, publication.path_pose_count, result.message.c_str());
  }

  std::string input_cloud_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  std::string debug_nodes_topic_;
  std::string debug_edges_topic_;
  std::string debug_rejected_topic_;
  std::string base_frame_;
  std::string planner_mode_name_{"rrt_star"};
  PlannerMode planner_mode_{PlannerMode::kRrtStar};

  double map_resolution_m_{0.10};
  double lattice_tolerance_m_{0.02};
  std::size_t max_grid_cells_{5000000U};
  double transform_timeout_s_{0.25};
  double path_output_spacing_m_{0.05};
  double node_marker_scale_m_{0.08};
  double edge_marker_width_m_{0.025};
  double path_marker_width_m_{0.08};
  double rejected_marker_scale_m_{0.07};
  std::size_t max_rejected_markers_{5000U};
  std::size_t path_invalid_confirmations_{2U};

  TerrainEvaluatorParameters terrain_parameters_;
  WavefrontPlannerParameters planner_parameters_;
  RrtStarParameters rrt_star_parameters_;
  std::unique_ptr<WavefrontPlanner> wavefront_planner_;
  std::unique_ptr<RrtStarPlanner> rrt_star_planner_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr nodes_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr edges_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rejected_publisher_;

  std::mutex state_mutex_;
  std::mutex output_mutex_;
  std::shared_ptr<const MapState> map_state_;
  std::optional<PendingGoal> pending_goal_;
  std::optional<ActivePlanState> active_plan_;
  std::optional<PlanRequest> queued_request_;
  std::uint64_t goal_epoch_{0U};
  std::uint64_t request_sequence_{0U};
  std::uint64_t plan_id_sequence_{0U};
  std::uint64_t last_empty_path_goal_epoch_{
    std::numeric_limits<std::uint64_t>::max()};
  std::condition_variable planning_cv_;
  std::thread planning_worker_;
  bool stop_worker_{false};
};

}  // namespace rubi_heightmap_wavefront_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<rubi_heightmap_wavefront_planner::PlannerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("rubi_heightmap_wavefront_planner"),
      "Fatal planner node exception: %s", error.what());
    rclcpp::shutdown();
    return 1;
  } catch (...) {
    RCLCPP_FATAL(
      rclcpp::get_logger("rubi_heightmap_wavefront_planner"),
      "Fatal planner node exception: unknown error");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
