#include "pongbot_navigation/pure_pursuit_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pongbot_navigation::pure_pursuit
{
namespace
{

double squaredDistance(const Point2D & lhs, const Point2D & rhs)
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  return dx * dx + dy * dy;
}

double clampChange(double previous, double desired, double maximum_change)
{
  return previous + std::clamp(desired - previous, -maximum_change, maximum_change);
}

}  // namespace

bool isFinite(const Point2D & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool isFinitePath(const std::vector<Point2D> & path)
{
  return !path.empty() &&
         std::all_of(path.begin(), path.end(), [](const Point2D & point) {
           return isFinite(point);
         });
}

double distance(const Point2D & lhs, const Point2D & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

std::size_t findNearestPathIndex(
  const std::vector<Point2D> & path,
  const Point2D & robot,
  std::size_t previous_index,
  std::size_t backtrack_points)
{
  if (path.empty()) {
    return 0;
  }

  previous_index = std::min(previous_index, path.size() - 1);
  const std::size_t search_begin =
    previous_index > backtrack_points ? previous_index - backtrack_points : 0;
  std::size_t best_index = search_begin;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = search_begin; index < path.size(); ++index) {
    const double candidate = squaredDistance(path[index], robot);
    if (candidate < best_distance) {
      best_distance = candidate;
      best_index = index;
    }
  }

  // A small backwards search window helps localization noise, while the stored
  // progress itself remains monotonic until a replacement Path resets it.
  return std::max(previous_index, best_index);
}

std::optional<TargetPoint> selectLookaheadTarget(
  const std::vector<Point2D> & path,
  std::size_t nearest_index,
  double lookahead_distance)
{
  if (path.empty() || !std::isfinite(lookahead_distance) || lookahead_distance < 0.0) {
    return std::nullopt;
  }

  nearest_index = std::min(nearest_index, path.size() - 1);
  if (lookahead_distance == 0.0 || nearest_index == path.size() - 1) {
    return TargetPoint{path[nearest_index], nearest_index};
  }

  double accumulated = 0.0;
  for (std::size_t index = nearest_index + 1; index < path.size(); ++index) {
    const Point2D & start = path[index - 1];
    const Point2D & end = path[index];
    const double segment_length = distance(start, end);
    if (segment_length <= std::numeric_limits<double>::epsilon()) {
      continue;
    }
    if (accumulated + segment_length >= lookahead_distance) {
      const double ratio = (lookahead_distance - accumulated) / segment_length;
      return TargetPoint{
        Point2D{
          start.x + ratio * (end.x - start.x),
          start.y + ratio * (end.y - start.y)},
        index};
    }
    accumulated += segment_length;
  }

  return TargetPoint{path.back(), path.size() - 1};
}

Point2D toRobotFrame(const Point2D & point, const Pose2D & robot)
{
  const double dx = point.x - robot.x;
  const double dy = point.y - robot.y;
  const double cosine = std::cos(robot.yaw);
  const double sine = std::sin(robot.yaw);
  return Point2D{cosine * dx + sine * dy, -sine * dx + cosine * dy};
}

std::optional<TargetPoint> selectForwardTarget(
  const std::vector<Point2D> & path,
  const Pose2D & robot,
  std::size_t nearest_index,
  double lookahead_distance,
  double epsilon)
{
  auto target = selectLookaheadTarget(path, nearest_index, lookahead_distance);
  if (!target) {
    return std::nullopt;
  }
  if (toRobotFrame(target->point, robot).x > epsilon) {
    return target;
  }

  for (std::size_t index = target->path_index; index < path.size(); ++index) {
    if (toRobotFrame(path[index], robot).x > epsilon) {
      return TargetPoint{path[index], index};
    }
  }
  return std::nullopt;
}

std::optional<double> computeCurvature(
  const Point2D & target_in_robot_frame, double epsilon)
{
  const double squared_lookahead =
    target_in_robot_frame.x * target_in_robot_frame.x +
    target_in_robot_frame.y * target_in_robot_frame.y;
  if (!isFinite(target_in_robot_frame) || squared_lookahead <= epsilon) {
    return std::nullopt;
  }
  return 2.0 * target_in_robot_frame.y / squared_lookahead;
}

Command computeDesiredCommand(double curvature, const Limits & limits)
{
  if (!std::isfinite(curvature)) {
    return {};
  }
  const double scale = 1.0 / (1.0 + limits.curvature_velocity_gain * std::abs(curvature));
  const double linear = std::clamp(
    limits.nominal_linear_velocity * scale,
    limits.min_tracking_velocity,
    limits.max_linear_velocity);
  const double angular = std::clamp(
    linear * curvature,
    -limits.max_angular_velocity,
    limits.max_angular_velocity);
  return Command{std::max(0.0, linear), angular};
}

Command rateLimit(
  const Command & previous,
  const Command & desired,
  double dt,
  double max_linear_acceleration,
  double max_angular_acceleration)
{
  if (!std::isfinite(dt) || dt <= 0.0) {
    return previous;
  }
  return Command{
    clampChange(previous.linear, desired.linear, max_linear_acceleration * dt),
    clampChange(previous.angular, desired.angular, max_angular_acceleration * dt)};
}

bool withinGoalTolerance(
  const Point2D & robot, const Point2D & goal, double tolerance)
{
  return tolerance >= 0.0 && distance(robot, goal) <= tolerance;
}

}  // namespace pongbot_navigation::pure_pursuit
