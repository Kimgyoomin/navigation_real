#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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

std::string rejectionKindName(const RejectedSampleKind kind)
{
  switch (kind) {
    case RejectedSampleKind::kNodeInvalid:
      return "node_invalid";
    case RejectedSampleKind::kExpansionEdgeInvalid:
      return "expansion_edge_invalid";
    case RejectedSampleKind::kMergeEdgeInvalid:
      return "merge_edge_invalid";
    case RejectedSampleKind::kGoalEdgeInvalid:
      return "goal_edge_invalid";
    case RejectedSampleKind::kNonFiniteEvaluation:
      return "non_finite";
    case RejectedSampleKind::kDuplicateEdge:
      return "duplicate_edge";
  }
  return "unknown";
}

auto makeColor(const float red, const float green, const float blue, const float alpha)
{
  visualization_msgs::msg::Marker marker;
  auto color = marker.color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
  return color;
}

auto rejectionColor(
  const TerrainInvalidReason reason,
  const RejectedSampleKind kind)
{
  switch (reason) {
    case TerrainInvalidReason::kOutOfBounds:
      return makeColor(0.25F, 0.25F, 0.25F, 0.90F);
    case TerrainInvalidReason::kUnknown:
      return makeColor(0.55F, 0.55F, 0.55F, 0.90F);
    case TerrainInvalidReason::kInsufficientFootprintSupport:
      return makeColor(1.00F, 0.50F, 0.05F, 0.95F);
    case TerrainInvalidReason::kInsufficientPcaSupport:
      return makeColor(1.00F, 0.90F, 0.05F, 0.95F);
    case TerrainInvalidReason::kSlopeLimit:
      return makeColor(1.00F, 0.05F, 0.05F, 0.95F);
    case TerrainInvalidReason::kRoughnessLimit:
      return makeColor(0.65F, 0.20F, 1.00F, 0.95F);
    case TerrainInvalidReason::kStepLimit:
      return makeColor(1.00F, 0.00F, 0.75F, 0.95F);
    case TerrainInvalidReason::kInvalidInput:
      return makeColor(0.05F, 0.05F, 0.05F, 0.95F);
    case TerrainInvalidReason::kNone:
      break;
  }
  if (kind == RejectedSampleKind::kDuplicateEdge) {
    return makeColor(0.00F, 0.85F, 0.90F, 0.85F);
  }
  return makeColor(0.10F, 0.10F, 0.10F, 0.90F);
}

bool finitePoint(const Point2D point) noexcept
{
  return std::isfinite(point.x) && std::isfinite(point.y);
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
      "resolution=%.3f m, max_grid_cells=%zu",
      planner_mode_name_.c_str(), input_cloud_topic_.c_str(),
      goal_topic_.c_str(), base_frame_.c_str(), map_resolution_m_,
      max_grid_cells_);
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
      declare_parameter<std::string>("planner_mode", "wavefront");
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
      parsed.points.push_back(TerrainPoint{
          static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)});
    }
    preflightLattice(parsed.points);
    return parsed;
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

      std::optional<geometry_msgs::msg::PoseStamped> pending_goal;
      bool materially_changed = false;
      bool replaced_existing_map = false;
      std::shared_ptr<const MapState> accepted_state;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (
          map_state_ != nullptr &&
          map_state_->content_hash == parsed.content_hash &&
          map_state_->frame_id == cloud->header.frame_id)
        {
          // Identical periodic FastDEM snapshots must not erase a latched path.
          return;
        }

        replaced_existing_map = map_state_ != nullptr;
        materially_changed = true;
        const std::uint64_t generation =
          map_state_ == nullptr ? 1U : map_state_->generation + 1U;
        auto next_state = std::make_shared<MapState>();
        next_state->snapshot = std::move(snapshot);
        next_state->frame_id = cloud->header.frame_id;
        next_state->content_hash = parsed.content_hash;
        next_state->generation = generation;
        map_state_ = next_state;
        accepted_state = next_state;
        if (pending_goal_) {
          pending_goal = std::move(pending_goal_);
          pending_goal_.reset();
        }
      }

      if (materially_changed && replaced_existing_map) {
        publishClear(accepted_state->frame_id);
        RCLCPP_INFO(
          get_logger(),
          "Elevation snapshot changed; cleared transient-local path and debug markers");
      }
      RCLCPP_INFO(
        get_logger(),
        "Accepted elevation snapshot generation=%lu frame='%s': "
        "observed=%zu, grid=%zux%zu (%zu cells), hash=%016lx",
        static_cast<unsigned long>(accepted_state->generation),
        accepted_state->frame_id.c_str(),
        accepted_state->snapshot->observedCount(),
        accepted_state->snapshot->sizeX(), accepted_state->snapshot->sizeY(),
        accepted_state->snapshot->cellCount(),
        static_cast<unsigned long>(accepted_state->content_hash));

      if (pending_goal) {
        RCLCPP_INFO(
          get_logger(), "Processing goal that was pending before the first valid map");
        processGoal(*pending_goal, accepted_state);
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(),
        "Rejected FastDEM elevation snapshot: %s. The previous accepted map, if any, "
        "remains active.",
        error.what());
    }
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal)
  {
    std::shared_ptr<const MapState> state;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state = map_state_;
      if (state == nullptr) {
        pending_goal_ = *goal;
      }
    }
    if (state == nullptr) {
      RCLCPP_WARN(
        get_logger(),
        "No accepted elevation map yet; stored the latest goal as pending");
      return;
    }
    processGoal(*goal, state);
  }

  void processGoal(
    const geometry_msgs::msg::PoseStamped & goal,
    const std::shared_ptr<const MapState> & state)
  {
    if (goal.header.frame_id.empty()) {
      failAndClear(state, "Goal header.frame_id is empty");
      return;
    }

    try {
      const auto timeout = tf2::durationFromSec(transform_timeout_s_);
      const auto start_transform = tf_buffer_->lookupTransform(
        state->frame_id, base_frame_, tf2::TimePointZero, timeout);
      const Point2D start{
        start_transform.transform.translation.x,
        start_transform.transform.translation.y};

      geometry_msgs::msg::PoseStamped goal_in_map;
      if (goal.header.frame_id == state->frame_id) {
        goal_in_map = goal;
      } else {
        const auto goal_transform = tf_buffer_->lookupTransform(
          state->frame_id, goal.header.frame_id, tf2::TimePointZero, timeout);
        tf2::doTransform(goal, goal_in_map, goal_transform);
      }
      const Point2D goal_xy{
        goal_in_map.pose.position.x, goal_in_map.pose.position.y};
      if (!finitePoint(start) || !finitePoint(goal_xy)) {
        failAndClear(state, "Start or transformed goal contains non-finite XY");
        return;
      }

      const TerrainEvaluator terrain(*state->snapshot, terrain_parameters_);
      PlanResult result;
      if (planner_mode_ == PlannerMode::kWavefront) {
        result = wavefront_planner_->plan(terrain, start, goal_xy);
      } else {
        result = rrt_star_planner_->plan(terrain, start, goal_xy);
      }
      if (!result.success) {
        publishClearForState(state);
        RCLCPP_WARN(
          get_logger(),
          "Planning failed on map generation=%lu: %s; termination=%s; "
          "rejected[node=%zu expansion_edge=%zu merge_edge=%zu goal_edge=%zu "
          "non_finite=%zu duplicate=%zu]",
          static_cast<unsigned long>(state->generation), result.message.c_str(),
          terminationName(result.termination).c_str(),
          result.reject_counts.node_invalid,
          result.reject_counts.expansion_edge_invalid,
          result.reject_counts.merge_edge_invalid,
          result.reject_counts.goal_edge_invalid,
          result.reject_counts.non_finite_evaluation,
          result.reject_counts.duplicate_edge);
        return;
      }

      std::string validation_error;
      if (!revalidateGraphPath(result, terrain, validation_error)) {
        failAndClear(
          state, "Final graph path revalidation failed: " + validation_error);
        return;
      }

      std::vector<TerrainPoint> dense_path;
      if (!densifyPath(result, *state->snapshot, dense_path, validation_error)) {
        failAndClear(
          state, "Path densification failed: " + validation_error);
        return;
      }

      const auto stamp = now();
      const nav_msgs::msg::Path path =
        makePathMessage(dense_path, state->frame_id, stamp);
      {
        // Serialize publishing with map-change clearing. If a new map replaced
        // this immutable snapshot while planning, silently discard the stale plan.
        std::lock_guard<std::mutex> output_lock(output_mutex_);
        {
          std::lock_guard<std::mutex> state_lock(state_mutex_);
          if (map_state_ != state) {
            RCLCPP_WARN(
              get_logger(),
              "Discarded a completed plan because its map snapshot was superseded");
            return;
          }
        }
        path_publisher_->publish(path);
        nodes_publisher_->publish(
          makeNodeMarkers(result, state->frame_id, stamp));
        edges_publisher_->publish(
          makeEdgeMarkers(result, dense_path, state->frame_id, stamp));
        rejected_publisher_->publish(
          makeRejectedMarkers(result, *state->snapshot, state->frame_id, stamp));
      }

      RCLCPP_INFO(
        get_logger(),
        "Path published: mode=%s, generation=%lu, nodes=%zu, edges=%zu, "
        "graph_path=%zu, dense_poses=%zu, work_units=%zu, rewires=%zu, "
        "build_time=%.2f ms, termination=%s",
        planner_mode_name_.c_str(),
        static_cast<unsigned long>(state->generation), result.nodes.size(),
        result.edges.size(), result.path_node_ids.size(), path.poses.size(),
        result.expansions, result.rewires, result.build_time_ms,
        terminationName(result.termination).c_str());
    } catch (const tf2::TransformException & error) {
      failAndClear(
        state,
        "TF lookup/goal transform failed for map<-" + base_frame_ + ": " +
        error.what());
    } catch (const std::exception & error) {
      failAndClear(state, std::string("Planning exception: ") + error.what());
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
    return path;
  }

  visualization_msgs::msg::Marker makeDeleteAllMarker(
    const std::string & frame_id,
    const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    return marker;
  }

  visualization_msgs::msg::MarkerArray makeNodeMarkers(
    const PlanResult & result,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(makeDeleteAllMarker(frame_id, stamp));
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = planner_mode_name_ + "_nodes";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = node_marker_scale_m_;
    marker.scale.y = node_marker_scale_m_;
    marker.scale.z = node_marker_scale_m_;
    for (const auto & node : result.nodes) {
      geometry_msgs::msg::Point point;
      point.x = node.point.x;
      point.y = node.point.y;
      point.z = node.point.z + 0.5 * node_marker_scale_m_;
      marker.points.push_back(point);
      switch (node.role) {
        case GraphNodeRole::kStart:
          marker.colors.push_back(makeColor(0.00F, 1.00F, 1.00F, 1.00F));
          break;
        case GraphNodeRole::kGoal:
          marker.colors.push_back(makeColor(0.85F, 0.10F, 1.00F, 1.00F));
          break;
        case GraphNodeRole::kSampled:
          marker.colors.push_back(makeColor(0.20F, 0.80F, 0.35F, 0.90F));
          break;
      }
    }
    array.markers.push_back(std::move(marker));
    return array;
  }

  visualization_msgs::msg::MarkerArray makeEdgeMarkers(
    const PlanResult & result,
    const std::vector<TerrainPoint> & dense_path,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(makeDeleteAllMarker(frame_id, stamp));

    visualization_msgs::msg::Marker edges;
    edges.header.frame_id = frame_id;
    edges.header.stamp = stamp;
    edges.ns = planner_mode_name_ + "_risk_edges";
    edges.id = 0;
    edges.type = visualization_msgs::msg::Marker::LINE_LIST;
    edges.action = visualization_msgs::msg::Marker::ADD;
    edges.pose.orientation.w = 1.0;
    edges.scale.x = edge_marker_width_m_;
    for (const auto & edge : result.edges) {
      if (edge.from >= result.nodes.size() || edge.to >= result.nodes.size()) {
        continue;
      }
      const double normalized_slope = std::clamp(
        edge.terrain.max_slope_deg /
        std::max(terrain_parameters_.max_slope_deg, kEpsilon),
        0.0, 1.0);
      const auto color = makeColor(
        1.0F,
        static_cast<float>(1.0 - normalized_slope),
        static_cast<float>(1.0 - normalized_slope),
        0.82F);
      for (const NodeId id : {edge.from, edge.to}) {
        geometry_msgs::msg::Point point;
        point.x = result.nodes[id].point.x;
        point.y = result.nodes[id].point.y;
        point.z = result.nodes[id].point.z + 0.02;
        edges.points.push_back(point);
        edges.colors.push_back(color);
      }
    }
    array.markers.push_back(std::move(edges));

    visualization_msgs::msg::Marker path;
    path.header.frame_id = frame_id;
    path.header.stamp = stamp;
    path.ns = planner_mode_name_ + "_final_path";
    path.id = 1;
    path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path.action = visualization_msgs::msg::Marker::ADD;
    path.pose.orientation.w = 1.0;
    path.scale.x = path_marker_width_m_;
    path.color = makeColor(0.00F, 1.00F, 0.25F, 1.00F);
    for (const auto & sample : dense_path) {
      geometry_msgs::msg::Point point;
      point.x = sample.x;
      point.y = sample.y;
      point.z = sample.z + 0.04;
      path.points.push_back(point);
    }
    array.markers.push_back(std::move(path));
    return array;
  }

  visualization_msgs::msg::MarkerArray makeRejectedMarkers(
    const PlanResult & result,
    const TerrainSnapshot & snapshot,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(makeDeleteAllMarker(frame_id, stamp));
    std::map<std::string, visualization_msgs::msg::Marker> buckets;
    const std::size_t count =
      std::min(result.rejected.size(), max_rejected_markers_);
    for (std::size_t index = 0U; index < count; ++index) {
      const auto & rejected = result.rejected[index];
      const std::string code =
        rejected.terrain_reason == TerrainInvalidReason::kNone ?
        rejectionKindName(rejected.kind) :
        std::string(toString(rejected.terrain_reason));
      auto insertion = buckets.emplace(code, visualization_msgs::msg::Marker{});
      auto & marker = insertion.first->second;
      if (insertion.second) {
        marker.header.frame_id = frame_id;
        marker.header.stamp = stamp;
        marker.ns = planner_mode_name_ + "_rejected/" + code;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = rejected_marker_scale_m_;
        marker.scale.y = rejected_marker_scale_m_;
        marker.scale.z = rejected_marker_scale_m_;
        marker.color = rejectionColor(rejected.terrain_reason, rejected.kind);
      }
      geometry_msgs::msg::Point point;
      point.x = rejected.candidate.x;
      point.y = rejected.candidate.y;
      const auto elevation = snapshot.elevationAt(point.x, point.y);
      if (elevation) {
        point.z = *elevation + 0.5 * rejected_marker_scale_m_;
      } else if (rejected.source < result.nodes.size()) {
        point.z =
          result.nodes[rejected.source].point.z +
          0.5 * rejected_marker_scale_m_;
      }
      marker.points.push_back(point);
    }
    int marker_id = 0;
    for (auto & item : buckets) {
      item.second.id = marker_id++;
      array.markers.push_back(std::move(item.second));
    }
    return array;
  }

  void publishClearUnlocked(const std::string & frame_id)
  {
    const auto stamp = now();
    const std::string safe_frame = frame_id.empty() ? "map" : frame_id;
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = safe_frame;
    empty_path.header.stamp = stamp;
    path_publisher_->publish(empty_path);

    visualization_msgs::msg::MarkerArray clear;
    clear.markers.push_back(makeDeleteAllMarker(safe_frame, stamp));
    nodes_publisher_->publish(clear);
    edges_publisher_->publish(clear);
    rejected_publisher_->publish(clear);
  }

  void publishClear(const std::string & frame_id)
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    publishClearUnlocked(frame_id);
  }

  bool publishClearForState(const std::shared_ptr<const MapState> & expected_state)
  {
    std::lock_guard<std::mutex> output_lock(output_mutex_);
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      if (map_state_ != expected_state) {
        return false;
      }
    }
    publishClearUnlocked(expected_state->frame_id);
    return true;
  }

  void failAndClear(
    const std::shared_ptr<const MapState> & expected_state,
    const std::string & message)
  {
    publishClearForState(expected_state);
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  }

  std::string input_cloud_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  std::string debug_nodes_topic_;
  std::string debug_edges_topic_;
  std::string debug_rejected_topic_;
  std::string base_frame_;
  std::string planner_mode_name_{"wavefront"};
  PlannerMode planner_mode_{PlannerMode::kWavefront};

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
  std::optional<geometry_msgs::msg::PoseStamped> pending_goal_;
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
