#ifndef PONGBOT_NAVIGATION__PURE_PURSUIT_CORE_HPP_
#define PONGBOT_NAVIGATION__PURE_PURSUIT_CORE_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace pongbot_navigation::pure_pursuit
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct TargetPoint
{
  Point2D point;
  std::size_t path_index{0};
};

struct Command
{
  double linear{0.0};
  double angular{0.0};
};

struct Limits
{
  double nominal_linear_velocity{0.20};
  double min_tracking_velocity{0.05};
  double max_linear_velocity{0.70};
  double max_angular_velocity{1.50};
  double curvature_velocity_gain{1.0};
};

bool isFinite(const Point2D & point);
bool isFinitePath(const std::vector<Point2D> & path);
double distance(const Point2D & lhs, const Point2D & rhs);

std::size_t findNearestPathIndex(
  const std::vector<Point2D> & path,
  const Point2D & robot,
  std::size_t previous_index,
  std::size_t backtrack_points);

std::optional<TargetPoint> selectLookaheadTarget(
  const std::vector<Point2D> & path,
  std::size_t nearest_index,
  double lookahead_distance);

Point2D toRobotFrame(const Point2D & point, const Pose2D & robot);

std::optional<TargetPoint> selectForwardTarget(
  const std::vector<Point2D> & path,
  const Pose2D & robot,
  std::size_t nearest_index,
  double lookahead_distance,
  double epsilon = 1.0e-9);

std::optional<double> computeCurvature(
  const Point2D & target_in_robot_frame,
  double epsilon = 1.0e-9);

Command computeDesiredCommand(double curvature, const Limits & limits);
Command rateLimit(
  const Command & previous,
  const Command & desired,
  double dt,
  double max_linear_acceleration,
  double max_angular_acceleration);

bool withinGoalTolerance(
  const Point2D & robot, const Point2D & goal, double tolerance);

}  // namespace pongbot_navigation::pure_pursuit

#endif  // PONGBOT_NAVIGATION__PURE_PURSUIT_CORE_HPP_
