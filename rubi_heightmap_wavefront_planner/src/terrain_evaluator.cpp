#include "rubi_heightmap_wavefront_planner/terrain_evaluator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rubi_heightmap_wavefront_planner
{
namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kNumericalEpsilon = 1.0e-12;

using Matrix3 = std::array<std::array<double, 3>, 3>;

struct EigenSystem3
{
  std::array<double, 3> values{};
  Matrix3 vectors{};
};

bool finitePoint(Point2D point) noexcept
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

Matrix3 identityMatrix()
{
  return Matrix3{{
    {{1.0, 0.0, 0.0}},
    {{0.0, 1.0, 0.0}},
    {{0.0, 0.0, 1.0}},
  }};
}

EigenSystem3 symmetricEigenDecomposition(Matrix3 matrix)
{
  Matrix3 vectors = identityMatrix();

  // A 3x3 symmetric Jacobi solver is sufficient here and avoids a ROS/Eigen dependency.
  for (std::size_t iteration = 0; iteration < 32U; ++iteration) {
    std::size_t p = 0U;
    std::size_t q = 1U;
    double largest = std::abs(matrix[0][1]);
    for (const auto & pair : {
        std::pair<std::size_t, std::size_t>{0U, 2U},
        std::pair<std::size_t, std::size_t>{1U, 2U}})
    {
      const double candidate = std::abs(matrix[pair.first][pair.second]);
      if (candidate > largest) {
        largest = candidate;
        p = pair.first;
        q = pair.second;
      }
    }

    const double diagonal_scale =
      std::abs(matrix[0][0]) + std::abs(matrix[1][1]) + std::abs(matrix[2][2]) + 1.0;
    if (largest <= kNumericalEpsilon * diagonal_scale) {
      break;
    }

    const double apq = matrix[p][q];
    const double tau = (matrix[q][q] - matrix[p][p]) / (2.0 * apq);
    const double t =
      std::copysign(1.0, tau) /
      (std::abs(tau) + std::sqrt(1.0 + tau * tau));
    const double c = 1.0 / std::sqrt(1.0 + t * t);
    const double s = t * c;

    const double app = matrix[p][p];
    const double aqq = matrix[q][q];
    matrix[p][p] = app - t * apq;
    matrix[q][q] = aqq + t * apq;
    matrix[p][q] = 0.0;
    matrix[q][p] = 0.0;

    for (std::size_t k = 0U; k < 3U; ++k) {
      if (k == p || k == q) {
        continue;
      }
      const double akp = matrix[k][p];
      const double akq = matrix[k][q];
      matrix[k][p] = c * akp - s * akq;
      matrix[p][k] = matrix[k][p];
      matrix[k][q] = s * akp + c * akq;
      matrix[q][k] = matrix[k][q];
    }

    for (std::size_t k = 0U; k < 3U; ++k) {
      const double vkp = vectors[k][p];
      const double vkq = vectors[k][q];
      vectors[k][p] = c * vkp - s * vkq;
      vectors[k][q] = s * vkp + c * vkq;
    }
  }

  return EigenSystem3{
    {{matrix[0][0], matrix[1][1], matrix[2][2]}},
    vectors};
}

void validateParameters(const TerrainEvaluatorParameters & parameters)
{
  if (!std::isfinite(parameters.pca_radius_m) || parameters.pca_radius_m <= 0.0) {
    throw std::invalid_argument("pca_radius_m must be finite and positive");
  }
  if (parameters.min_pca_points < 3U) {
    throw std::invalid_argument("min_pca_points must be at least 3");
  }
  if (!std::isfinite(parameters.footprint_radius_m) || parameters.footprint_radius_m < 0.0) {
    throw std::invalid_argument("footprint_radius_m must be finite and non-negative");
  }
  if (
    !std::isfinite(parameters.min_footprint_observed_ratio) ||
    parameters.min_footprint_observed_ratio < 0.0 ||
    parameters.min_footprint_observed_ratio > 1.0)
  {
    throw std::invalid_argument("min_footprint_observed_ratio must be in [0, 1]");
  }
  if (
    !std::isfinite(parameters.max_slope_deg) ||
    parameters.max_slope_deg < 0.0 || parameters.max_slope_deg >= 90.0)
  {
    throw std::invalid_argument("max_slope_deg must be finite and in [0, 90)");
  }
  if (
    std::isnan(parameters.max_roughness_m) ||
    parameters.max_roughness_m < 0.0)
  {
    throw std::invalid_argument("max_roughness_m must be non-negative");
  }
  if (
    !std::isfinite(parameters.max_step_height_m) ||
    parameters.max_step_height_m < 0.0)
  {
    throw std::invalid_argument("max_step_height_m must be finite and non-negative");
  }
  if (
    !std::isfinite(parameters.edge_sample_spacing_m) ||
    parameters.edge_sample_spacing_m <= 0.0)
  {
    throw std::invalid_argument("edge_sample_spacing_m must be finite and positive");
  }
  if (
    !std::isfinite(parameters.slope_cost_weight) ||
    parameters.slope_cost_weight < 0.0)
  {
    throw std::invalid_argument("slope_cost_weight must be finite and non-negative");
  }
}

}  // namespace

std::string_view toString(TerrainInvalidReason reason) noexcept
{
  switch (reason) {
    case TerrainInvalidReason::kNone:
      return "none";
    case TerrainInvalidReason::kOutOfBounds:
      return "out_of_bounds";
    case TerrainInvalidReason::kUnknown:
      return "unknown";
    case TerrainInvalidReason::kInsufficientFootprintSupport:
      return "insufficient_footprint_support";
    case TerrainInvalidReason::kInsufficientPcaSupport:
      return "insufficient_pca_support";
    case TerrainInvalidReason::kSlopeLimit:
      return "slope_limit";
    case TerrainInvalidReason::kRoughnessLimit:
      return "roughness_limit";
    case TerrainInvalidReason::kStepLimit:
      return "step_limit";
    case TerrainInvalidReason::kInvalidInput:
      return "invalid_input";
  }
  return "invalid_input";
}

TerrainEvaluator::TerrainEvaluator(
  const TerrainSnapshot & snapshot,
  TerrainEvaluatorParameters parameters)
: snapshot_(snapshot), parameters_(std::move(parameters))
{
  validateParameters(parameters_);
  if (
    parameters_.edge_sample_spacing_m >
    0.5 * snapshot_.resolution() + kNumericalEpsilon)
  {
    throw std::invalid_argument(
            "edge_sample_spacing_m must be <= half the terrain resolution");
  }
}

const TerrainSnapshot & TerrainEvaluator::snapshot() const noexcept
{
  return snapshot_;
}

const TerrainEvaluatorParameters & TerrainEvaluator::parameters() const noexcept
{
  return parameters_;
}

SurfaceMetrics TerrainEvaluator::localSurface(Point2D position) const
{
  SurfaceMetrics result;
  if (!finitePoint(position)) {
    return result;
  }

  const double radius = parameters_.pca_radius_m;
  const double radius_squared = radius * radius;
  const double resolution = snapshot_.resolution();
  const auto min_ix = static_cast<std::int64_t>(
    std::ceil((position.x - radius - snapshot_.minXCenter()) / resolution));
  const auto max_ix = static_cast<std::int64_t>(
    std::floor((position.x + radius - snapshot_.minXCenter()) / resolution));
  const auto min_iy = static_cast<std::int64_t>(
    std::ceil((position.y - radius - snapshot_.minYCenter()) / resolution));
  const auto max_iy = static_cast<std::int64_t>(
    std::floor((position.y + radius - snapshot_.minYCenter()) / resolution));

  std::vector<std::array<double, 3>> samples;
  if (max_ix >= min_ix && max_iy >= min_iy) {
    const auto width = static_cast<std::size_t>(max_ix - min_ix + 1);
    const auto height = static_cast<std::size_t>(max_iy - min_iy + 1);
    if (width <= std::numeric_limits<std::size_t>::max() / height) {
      samples.reserve(width * height);
    }
  }

  const double inclusion_epsilon =
    std::max(kNumericalEpsilon, radius_squared * 1.0e-12);
  for (std::int64_t iy = min_iy; iy <= max_iy; ++iy) {
    for (std::int64_t ix = min_ix; ix <= max_ix; ++ix) {
      const double cell_x = snapshot_.minXCenter() + static_cast<double>(ix) * resolution;
      const double cell_y = snapshot_.minYCenter() + static_cast<double>(iy) * resolution;
      const double dx = cell_x - position.x;
      const double dy = cell_y - position.y;
      if (dx * dx + dy * dy > radius_squared + inclusion_epsilon) {
        continue;
      }
      if (
        ix < 0 || iy < 0 ||
        ix >= static_cast<std::int64_t>(snapshot_.sizeX()) ||
        iy >= static_cast<std::int64_t>(snapshot_.sizeY()))
      {
        continue;
      }

      const auto ux = static_cast<std::size_t>(ix);
      const auto uy = static_cast<std::size_t>(iy);
      const auto elevation = snapshot_.elevationAtCell(ux, uy);
      if (elevation) {
        samples.push_back({{cell_x, cell_y, *elevation}});
      }
    }
  }

  result.sample_count = samples.size();
  if (samples.size() < parameters_.min_pca_points) {
    return result;
  }

  std::array<double, 3> mean{{0.0, 0.0, 0.0}};
  for (const auto & sample : samples) {
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
      mean[axis] += sample[axis];
    }
  }
  for (double & value : mean) {
    value /= static_cast<double>(samples.size());
  }

  Matrix3 covariance{};
  for (const auto & sample : samples) {
    std::array<double, 3> centered{};
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
      centered[axis] = sample[axis] - mean[axis];
    }
    for (std::size_t row = 0U; row < 3U; ++row) {
      for (std::size_t column = row; column < 3U; ++column) {
        covariance[row][column] += centered[row] * centered[column];
      }
    }
  }

  const double denominator = static_cast<double>(samples.size() - 1U);
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = row; column < 3U; ++column) {
      covariance[row][column] /= denominator;
      covariance[column][row] = covariance[row][column];
    }
  }

  const EigenSystem3 eigen = symmetricEigenDecomposition(covariance);
  std::array<std::size_t, 3> order{{0U, 1U, 2U}};
  std::sort(
    order.begin(), order.end(),
    [&eigen](std::size_t lhs, std::size_t rhs) {
      return eigen.values[lhs] < eigen.values[rhs];
    });

  const double largest_eigenvalue = std::max(0.0, eigen.values[order[2]]);
  const double middle_eigenvalue = std::max(0.0, eigen.values[order[1]]);
  const double support_epsilon = std::max(kNumericalEpsilon, largest_eigenvalue * 1.0e-9);
  if (largest_eigenvalue <= kNumericalEpsilon || middle_eigenvalue <= support_epsilon) {
    return result;
  }

  const std::size_t normal_column = order[0];
  double nx = eigen.vectors[0][normal_column];
  double ny = eigen.vectors[1][normal_column];
  double nz = eigen.vectors[2][normal_column];
  const double normal_norm = std::sqrt(nx * nx + ny * ny + nz * nz);
  if (!std::isfinite(normal_norm) || normal_norm <= kNumericalEpsilon) {
    return result;
  }
  nx /= normal_norm;
  ny /= normal_norm;
  nz /= normal_norm;
  if (nz < 0.0) {
    nx = -nx;
    ny = -ny;
    nz = -nz;
  }

  result.valid = true;
  result.normal_x = nx;
  result.normal_y = ny;
  result.normal_z = nz;
  result.slope_deg =
    std::acos(std::clamp(nz, 0.0, 1.0)) * 180.0 / kPi;
  result.roughness_m = std::sqrt(std::max(0.0, eigen.values[normal_column]));
  return result;
}

NodeEvaluation TerrainEvaluator::evaluateNode(Point2D position) const
{
  return evaluatePosition(position, true);
}

EdgeEvaluation TerrainEvaluator::evaluateEdge(Point2D from, Point2D to) const
{
  EdgeEvaluation result;
  if (!finitePoint(from) || !finitePoint(to)) {
    return result;
  }

  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double length_xy = std::hypot(dx, dy);
  result.length_xy_m = length_xy;
  if (!std::isfinite(length_xy) || length_xy <= kNumericalEpsilon) {
    return result;
  }

  const std::size_t segment_count = std::max<std::size_t>(
    1U,
    static_cast<std::size_t>(
      std::ceil(length_xy / parameters_.edge_sample_spacing_m)));
  result.sample_count = segment_count + 1U;

  std::vector<Point2D> sample_positions;
  sample_positions.reserve(result.sample_count);

  // First pass: preserve a deterministic validity precedence of
  // out-of-bounds -> unknown -> step. A sharp step often also produces a
  // steep local PCA plane; reporting the step first makes the rejection
  // physically interpretable and keeps RViz diagnostics unambiguous.
  double previous_z = 0.0;
  bool have_previous = false;
  for (std::size_t sample_index = 0U; sample_index <= segment_count; ++sample_index) {
    const double interpolation =
      static_cast<double>(sample_index) / static_cast<double>(segment_count);
    const Point2D position{
      from.x + interpolation * dx,
      from.y + interpolation * dy};

    const auto index = snapshot_.worldToIndex(position.x, position.y);
    if (!index) {
      result.reason = TerrainInvalidReason::kOutOfBounds;
      return result;
    }
    const auto cell = snapshot_.query(position.x, position.y);
    if (!cell) {
      result.reason = TerrainInvalidReason::kUnknown;
      return result;
    }

    sample_positions.push_back(position);
    if (have_previous) {
      const double step = std::abs(cell->elevation_m - previous_z);
      result.max_step_m = std::max(result.max_step_m, step);
      if (step > parameters_.max_step_height_m) {
        result.reason = TerrainInvalidReason::kStepLimit;
        return result;
      }
      const double segment_xy = length_xy / static_cast<double>(segment_count);
      result.length_3d_m += std::hypot(segment_xy, cell->elevation_m - previous_z);
    }
    previous_z = cell->elevation_m;
    have_previous = true;
  }

  // Second pass: assess footprint support and the local PCA surface at every
  // edge sample, rather than checking endpoints only.
  double slope_sum = 0.0;
  for (const Point2D position : sample_positions) {
    const NodeEvaluation node =
      evaluatePosition(position, parameters_.check_footprint_along_edge);
    result.min_footprint_observed_ratio = std::min(
      result.min_footprint_observed_ratio, node.footprint_observed_ratio);
    result.max_step_m = std::max(
      result.max_step_m, node.max_footprint_step_m);
    if (!node.valid) {
      result.reason = node.reason;
      return result;
    }

    result.max_slope_deg = std::max(result.max_slope_deg, node.surface.slope_deg);
    result.max_roughness_m = std::max(
      result.max_roughness_m, node.surface.roughness_m);
    slope_sum += node.surface.slope_deg;
  }

  result.mean_slope_deg = slope_sum / static_cast<double>(result.sample_count);
  double normalized_slope_risk = 0.0;
  if (parameters_.max_slope_deg > kNumericalEpsilon) {
    normalized_slope_risk = std::clamp(
      result.mean_slope_deg / parameters_.max_slope_deg, 0.0, 1.0);
  }
  result.cost =
    result.length_3d_m *
    (1.0 + parameters_.slope_cost_weight * normalized_slope_risk);
  result.valid = true;
  result.reason = TerrainInvalidReason::kNone;
  return result;
}

TerrainEvaluator::FootprintMetrics TerrainEvaluator::footprintSupport(
  Point2D position) const
{
  FootprintMetrics result;
  if (!finitePoint(position)) {
    return result;
  }

  if (parameters_.footprint_radius_m <= kNumericalEpsilon) {
    result.total_cells = 1U;
    result.observed_cells = snapshot_.query(position.x, position.y) ? 1U : 0U;
    result.observed_ratio = static_cast<double>(result.observed_cells);
    return result;
  }

  const double radius = parameters_.footprint_radius_m;
  const double radius_squared = radius * radius;
  const double resolution = snapshot_.resolution();
  const auto min_ix = static_cast<std::int64_t>(
    std::ceil((position.x - radius - snapshot_.minXCenter()) / resolution));
  const auto max_ix = static_cast<std::int64_t>(
    std::floor((position.x + radius - snapshot_.minXCenter()) / resolution));
  const auto min_iy = static_cast<std::int64_t>(
    std::ceil((position.y - radius - snapshot_.minYCenter()) / resolution));
  const auto max_iy = static_cast<std::int64_t>(
    std::floor((position.y + radius - snapshot_.minYCenter()) / resolution));
  const double inclusion_epsilon =
    std::max(kNumericalEpsilon, radius_squared * 1.0e-12);

  for (std::int64_t iy = min_iy; iy <= max_iy; ++iy) {
    for (std::int64_t ix = min_ix; ix <= max_ix; ++ix) {
      const double cell_x = snapshot_.minXCenter() + static_cast<double>(ix) * resolution;
      const double cell_y = snapshot_.minYCenter() + static_cast<double>(iy) * resolution;
      const double dx = cell_x - position.x;
      const double dy = cell_y - position.y;
      if (dx * dx + dy * dy > radius_squared + inclusion_epsilon) {
        continue;
      }

      ++result.total_cells;
      if (
        ix >= 0 && iy >= 0 &&
        ix < static_cast<std::int64_t>(snapshot_.sizeX()) &&
        iy < static_cast<std::int64_t>(snapshot_.sizeY()) &&
        snapshot_.isObserved(
          static_cast<std::size_t>(ix), static_cast<std::size_t>(iy)))
      {
        ++result.observed_cells;

        const auto elevation = snapshot_.elevationAtCell(
          static_cast<std::size_t>(ix), static_cast<std::size_t>(iy));
        static constexpr std::array<std::pair<std::int64_t, std::int64_t>, 4>
        kPreviouslyVisitedNeighbors{{
          {-1, -1}, {0, -1}, {1, -1}, {-1, 0}
        }};
        for (const auto & [offset_x, offset_y] : kPreviouslyVisitedNeighbors) {
          const std::int64_t neighbor_ix = ix + offset_x;
          const std::int64_t neighbor_iy = iy + offset_y;
          if (
            neighbor_ix < 0 || neighbor_iy < 0 ||
            neighbor_ix >= static_cast<std::int64_t>(snapshot_.sizeX()) ||
            neighbor_iy >= static_cast<std::int64_t>(snapshot_.sizeY()))
          {
            continue;
          }

          const double neighbor_x =
            snapshot_.minXCenter() + static_cast<double>(neighbor_ix) * resolution;
          const double neighbor_y =
            snapshot_.minYCenter() + static_cast<double>(neighbor_iy) * resolution;
          const double neighbor_dx = neighbor_x - position.x;
          const double neighbor_dy = neighbor_y - position.y;
          if (
            neighbor_dx * neighbor_dx + neighbor_dy * neighbor_dy >
            radius_squared + inclusion_epsilon)
          {
            continue;
          }

          const auto neighbor_elevation = snapshot_.elevationAtCell(
            static_cast<std::size_t>(neighbor_ix),
            static_cast<std::size_t>(neighbor_iy));
          if (elevation && neighbor_elevation) {
            result.max_adjacent_step_m = std::max(
              result.max_adjacent_step_m,
              std::abs(*elevation - *neighbor_elevation));
          }
        }
      }
    }
  }

  if (result.total_cells > 0U) {
    result.observed_ratio =
      static_cast<double>(result.observed_cells) /
      static_cast<double>(result.total_cells);
  }
  return result;
}

NodeEvaluation TerrainEvaluator::evaluatePosition(
  Point2D position, bool check_footprint) const
{
  NodeEvaluation result;
  if (!finitePoint(position)) {
    return result;
  }

  const auto index = snapshot_.worldToIndex(position.x, position.y);
  if (!index) {
    result.reason = TerrainInvalidReason::kOutOfBounds;
    return result;
  }
  const auto cell = snapshot_.query(position.x, position.y);
  if (!cell) {
    result.reason = TerrainInvalidReason::kUnknown;
    return result;
  }
  result.elevation_m = cell->elevation_m;

  if (check_footprint) {
    const FootprintMetrics footprint = footprintSupport(position);
    result.footprint_observed_ratio = footprint.observed_ratio;
    result.max_footprint_step_m = footprint.max_adjacent_step_m;
    if (
      footprint.total_cells == 0U ||
      footprint.observed_ratio + kNumericalEpsilon <
      parameters_.min_footprint_observed_ratio)
    {
      result.reason = TerrainInvalidReason::kInsufficientFootprintSupport;
      return result;
    }
    if (footprint.max_adjacent_step_m > parameters_.max_step_height_m) {
      result.reason = TerrainInvalidReason::kStepLimit;
      return result;
    }
  } else {
    result.footprint_observed_ratio = 1.0;
  }

  result.surface = localSurface(position);
  if (!result.surface.valid) {
    result.reason = TerrainInvalidReason::kInsufficientPcaSupport;
    return result;
  }
  if (result.surface.slope_deg > parameters_.max_slope_deg) {
    result.reason = TerrainInvalidReason::kSlopeLimit;
    return result;
  }
  if (result.surface.roughness_m > parameters_.max_roughness_m) {
    result.reason = TerrainInvalidReason::kRoughnessLimit;
    return result;
  }

  result.valid = true;
  result.reason = TerrainInvalidReason::kNone;
  return result;
}

}  // namespace rubi_heightmap_wavefront_planner
