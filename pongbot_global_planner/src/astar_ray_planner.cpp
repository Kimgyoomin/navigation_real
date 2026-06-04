#include "pongbot_global_planner/astar_ray_planner.h"

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>
#include <utility>
#include <vector>

namespace pongbot_global_planner
{

namespace
{

struct OpenNode
{
  unsigned int idx;
  double f;
  double g;

  bool operator<(const OpenNode& other) const
  {
    // priority_queue is max-heap by default, so reverse for min-heap.
    return f > other.f;
  }
};

}  // namespace

AStarRayPlannerROS::AStarRayPlannerROS() {}

AStarRayPlannerROS::AStarRayPlannerROS(
  std::string name,
  costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}

void AStarRayPlannerROS::initialize(
  std::string name,
  costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_) {
    return;
  }

  costmap_ros_ = costmap_ros;

  // move_base loads plugin params under ~/<PluginName>/*
  ros::NodeHandle pnh("~/" + name);

  plan_pub_ = pnh.advertise<nav_msgs::Path>("plan", 1, true);
  plan_time_pub_ = pnh.advertise<std_msgs::Float64>("planning_time_ms", 1, true);

  pnh.param("allow_unknown", allow_unknown_, false);
  pnh.param("use_octile_heuristic", use_octile_heuristic_, true);

  int lethal_cost_tmp = static_cast<int>(lethal_cost_);
  pnh.param("lethal_cost", lethal_cost_tmp, 253);
  lethal_cost_tmp = std::max(1, std::min(255, lethal_cost_tmp));
  lethal_cost_ = static_cast<unsigned char>(lethal_cost_tmp);

  pnh.param("tie_breaker", tie_breaker_, 1.001);
  pnh.param("cost_weight_alpha", cost_weight_alpha_, 5.0);
  pnh.param("weight_h", weight_h_, 1.05);
  pnh.param("print_timing", print_timing_, true);

  pnh.param("ray_angle_bins", ray_angle_bins_, 16);
  pnh.param("max_segment_length", max_segment_length_, 1.5);
  pnh.param("resample_interval", resample_interval_, 0.25);
  pnh.param("line_cost_threshold", line_cost_threshold_, 180);

  ray_angle_bins_ = std::max(8, ray_angle_bins_);
  max_segment_length_ = std::max(0.10, max_segment_length_);
  resample_interval_ = std::max(0.05, resample_interval_);
  line_cost_threshold_ = std::max(1, std::min(252, line_cost_threshold_));
  cost_weight_alpha_ = std::max(0.0, cost_weight_alpha_);
  weight_h_ = std::max(0.0, weight_h_);
  tie_breaker_ = std::max(1.0, tie_breaker_);

  initialized_ = true;

  ROS_INFO_STREAM(
    "pongbot_global_planner::AStarRayPlannerROS initialized under ~/" << name
    << " allow_unknown=" << allow_unknown_
    << " octile=" << use_octile_heuristic_
    << " lethal_cost=" << static_cast<int>(lethal_cost_)
    << " tie=" << tie_breaker_
    << " alpha=" << cost_weight_alpha_
    << " w_h=" << weight_h_
    << " ray_bins=" << ray_angle_bins_
    << " max_segment=" << max_segment_length_
    << " resample=" << resample_interval_
    << " line_cost_threshold=" << line_cost_threshold_);
}

inline bool AStarRayPlannerROS::isCellFree(
  const costmap_2d::Costmap2D& cm,
  unsigned int mx,
  unsigned int my) const
{
  const unsigned char c = cm.getCost(mx, my);

  if (c == costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }

  return c < lethal_cost_;
}

inline bool AStarRayPlannerROS::isCellSafeForRay(
  const costmap_2d::Costmap2D& cm,
  unsigned int mx,
  unsigned int my) const
{
  const unsigned char c = cm.getCost(mx, my);

  if (c == costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }

  if (c >= lethal_cost_) {
    return false;
  }

  if (static_cast<int>(c) > line_cost_threshold_) {
    return false;
  }

  return true;
}

inline double AStarRayPlannerROS::heuristic(
  int x0,
  int y0,
  int x1,
  int y1,
  double resolution) const
{
  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);

  if (use_octile_heuristic_) {
    const double D = resolution;
    const double D2 = std::sqrt(2.0) * resolution;
    return D * (dx + dy) + (D2 - 2.0 * D) * std::min(dx, dy);
  }

  return resolution * std::hypot(static_cast<double>(dx), static_cast<double>(dy));
}

inline double AStarRayPlannerROS::yawBetween(
  double x0,
  double y0,
  double x1,
  double y1)
{
  return std::atan2(y1 - y0, x1 - x0);
}

bool AStarRayPlannerROS::snapToFree(
  const costmap_2d::Costmap2D& cm,
  unsigned& mx,
  unsigned& my) const
{
  const int W = static_cast<int>(cm.getSizeInCellsX());
  const int H = static_cast<int>(cm.getSizeInCellsY());

  auto inBounds = [&](int x, int y) {
    return x >= 0 && y >= 0 && x < W && y < H;
  };

  auto isFreeCell = [&](int x, int y) {
    if (!inBounds(x, y)) {
      return false;
    }
    return isCellFree(cm, static_cast<unsigned int>(x), static_cast<unsigned int>(y));
  };

  if (isFreeCell(static_cast<int>(mx), static_cast<int>(my))) {
    return true;
  }

  static const int dx4[4] = {1, -1, 0, 0};
  static const int dy4[4] = {0, 0, 1, -1};

  std::queue<std::pair<int, int>> q;
  std::vector<char> visited(W * H, 0);

  auto idx = [W](int x, int y) {
    return y * W + x;
  };

  const int sx = static_cast<int>(mx);
  const int sy = static_cast<int>(my);

  if (!inBounds(sx, sy)) {
    return false;
  }

  q.push(std::make_pair(sx, sy));
  visited[idx(sx, sy)] = 1;

  while (!q.empty()) {
    const auto cur = q.front();
    q.pop();

    const int x = cur.first;
    const int y = cur.second;

    if (isFreeCell(x, y)) {
      mx = static_cast<unsigned int>(x);
      my = static_cast<unsigned int>(y);
      return true;
    }

    for (int k = 0; k < 4; ++k) {
      const int nx = x + dx4[k];
      const int ny = y + dy4[k];

      if (!inBounds(nx, ny)) {
        continue;
      }

      const int ni = idx(nx, ny);
      if (visited[ni]) {
        continue;
      }

      visited[ni] = 1;
      q.push(std::make_pair(nx, ny));
    }
  }

  return false;
}

bool AStarRayPlannerROS::lineSegmentCost(
  const costmap_2d::Costmap2D& cm,
  unsigned int x0,
  unsigned int y0,
  unsigned int x1,
  unsigned int y1,
  double& total_cost) const
{
  double wx0, wy0, wx1, wy1;
  cm.mapToWorld(x0, y0, wx0, wy0);
  cm.mapToWorld(x1, y1, wx1, wy1);

  const double dx = wx1 - wx0;
  const double dy = wy1 - wy0;
  const double dist = std::hypot(dx, dy);

  if (dist < 1e-9) {
    total_cost = 0.0;
    return true;
  }

  const double resolution = cm.getResolution();
  const double sample_step = std::max(0.5 * resolution, 0.01);
  const int steps = std::max(1, static_cast<int>(std::ceil(dist / sample_step)));
  const double ds = dist / static_cast<double>(steps);

  total_cost = dist;

  for (int s = 0; s <= steps; ++s) {
    const double t = static_cast<double>(s) / static_cast<double>(steps);
    const double wx = wx0 + t * dx;
    const double wy = wy0 + t * dy;

    unsigned int mx, my;
    if (!cm.worldToMap(wx, wy, mx, my)) {
      return false;
    }

    if (s == 0) {
      if (!isCellFree(cm, mx, my)) {
        return false;
      }
    } else {
      if (!isCellSafeForRay(cm, mx, my)) {
        return false;
      }
    }

    if (s > 0) {
      const unsigned char c = cm.getCost(mx, my);
      const double norm = std::min(1.0, static_cast<double>(c) / 252.0);
      total_cost += ds * cost_weight_alpha_ * norm;
    }
  }

  return true;
}

AStarRayPlannerROS::RayResult AStarRayPlannerROS::castRay(
  const costmap_2d::Costmap2D& cm,
  unsigned int cx,
  unsigned int cy,
  double angle) const
{
  RayResult result;

  double cwx, cwy;
  cm.mapToWorld(cx, cy, cwx, cwy);

  const double resolution = cm.getResolution();
  const double ray_step = std::max(0.5 * resolution, 0.01);
  const int max_steps =
    std::max(1, static_cast<int>(std::ceil(max_segment_length_ / ray_step)));

  unsigned int last_valid_x = cx;
  unsigned int last_valid_y = cy;
  bool moved = false;

  for (int step = 1; step <= max_steps; ++step) {
    const double dist = static_cast<double>(step) * ray_step;
    const double wx = cwx + std::cos(angle) * dist;
    const double wy = cwy + std::sin(angle) * dist;

    unsigned int mx, my;
    if (!cm.worldToMap(wx, wy, mx, my)) {
      break;
    }

    if (mx == last_valid_x && my == last_valid_y) {
      continue;
    }

    if (!isCellSafeForRay(cm, mx, my)) {
      break;
    }

    double seg_cost = 0.0;
    if (!lineSegmentCost(cm, cx, cy, mx, my, seg_cost)) {
      break;
    }

    last_valid_x = mx;
    last_valid_y = my;
    result.cost = seg_cost;
    result.length = dist;
    moved = true;
  }

  if (!moved) {
    return result;
  }

  result.valid = true;
  result.x = last_valid_x;
  result.y = last_valid_y;
  return result;
}

bool AStarRayPlannerROS::makePlan(
  const geometry_msgs::PoseStamped& start,
  const geometry_msgs::PoseStamped& goal,
  std::vector<geometry_msgs::PoseStamped>& plan)
{
  plan.clear();

  if (!initialized_) {
    ROS_ERROR("[AStarRayPlannerROS] makePlan called but not initialized");
    return false;
  }

  if (!costmap_ros_) {
    ROS_ERROR("[AStarRayPlannerROS] costmap_ros_ is null");
    return false;
  }

  if (start.header.frame_id != goal.header.frame_id) {
    ROS_ERROR_STREAM(
      "[AStarRayPlannerROS] frame mismatch: "
      << start.header.frame_id << " vs " << goal.header.frame_id);
    return false;
  }

  const std::string frame = costmap_ros_->getGlobalFrameID();

  costmap_2d::Costmap2D local_copy;
  {
    costmap_2d::Costmap2D* cm = costmap_ros_->getCostmap();
    std::lock_guard<costmap_2d::Costmap2D::mutex_t> lk(*(cm->getMutex()));
    local_copy = *cm;
  }

  const ros::Time stamp = ros::Time::now();

  const auto t0 = std::chrono::steady_clock::now();

  const bool ok = searchRayAstar(local_copy, start, goal, plan, frame, stamp);

  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();

  std_msgs::Float64 dt;
  dt.data = ms;
  plan_time_pub_.publish(dt);

  if (print_timing_) {
    std::ostringstream oss;
    oss << "\033[1;35m"
        << "[Ray A*] planning_time = "
        << std::fixed << std::setprecision(3) << ms << " ms"
        << "  (poses=" << plan.size() << ")"
        << "\033[0m";
    ROS_INFO_STREAM_NAMED("pongbot_astar_ray", oss.str());
  }

  if (ok) {
    for (auto& p : plan) {
      p.header.stamp = stamp;
      p.header.frame_id = frame;
    }

    nav_msgs::Path path_msg;
    path_msg.header.stamp = stamp;
    path_msg.header.frame_id = frame;
    path_msg.poses = plan;
    plan_pub_.publish(path_msg);
  } else {
    ROS_WARN("[AStarRayPlannerROS] no path found");
  }

  return ok;
}

bool AStarRayPlannerROS::searchRayAstar(
  const costmap_2d::Costmap2D& cm_copy,
  const geometry_msgs::PoseStamped& start,
  const geometry_msgs::PoseStamped& goal,
  std::vector<geometry_msgs::PoseStamped>& plan,
  const std::string& frame,
  const ros::Time& stamp)
{
  const unsigned int W = cm_copy.getSizeInCellsX();
  const unsigned int H = cm_copy.getSizeInCellsY();

  if (W == 0 || H == 0) {
    ROS_ERROR("[AStarRayPlannerROS] empty costmap");
    return false;
  }

  unsigned int sx, sy, gx, gy;

  if (!cm_copy.worldToMap(start.pose.position.x, start.pose.position.y, sx, sy)) {
    ROS_WARN("[AStarRayPlannerROS] start outside map");
    return false;
  }

  if (!cm_copy.worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
    ROS_WARN("[AStarRayPlannerROS] goal outside map");
    return false;
  }

  if (!isCellFree(cm_copy, sx, sy)) {
    if (!snapToFree(cm_copy, sx, sy)) {
      ROS_WARN("[AStarRayPlannerROS] start in obstacle and snap failed");
      return false;
    }
  }

  if (!isCellFree(cm_copy, gx, gy)) {
    if (!snapToFree(cm_copy, gx, gy)) {
      ROS_WARN("[AStarRayPlannerROS] goal in obstacle and snap failed");
      return false;
    }
  }

  auto toIndex = [W](unsigned int x, unsigned int y) -> unsigned int {
    return y * W + x;
  };

  auto toXY = [W](unsigned int idx, unsigned int& x, unsigned int& y) {
    x = idx % W;
    y = idx / W;
  };

  const unsigned int sidx = toIndex(sx, sy);
  const unsigned int gidx = toIndex(gx, gy);

  if (sidx == gidx) {
    plan.push_back(start);
    plan.push_back(goal);
    return true;
  }

  std::vector<double> gscore(W * H, std::numeric_limits<double>::infinity());
  std::vector<int> parent(W * H, -1);
  std::vector<char> closed(W * H, 0);

  std::priority_queue<OpenNode> open;

  gscore[sidx] = 0.0;
  open.push(OpenNode{
    sidx,
    weight_h_ * heuristic(static_cast<int>(sx), static_cast<int>(sy),
                          static_cast<int>(gx), static_cast<int>(gy),
                          cm_copy.getResolution()) * tie_breaker_,
    0.0});

  bool found = false;
  std::size_t expanded_nodes = 0;
  constexpr double TWO_PI = 6.28318530717958647692;

  while (!open.empty()) {
    const OpenNode cur = open.top();
    open.pop();

    if (cur.g != gscore[cur.idx]) {
      continue;
    }

    if (closed[cur.idx]) {
      continue;
    }

    closed[cur.idx] = 1;
    ++expanded_nodes;

    if (cur.idx == gidx) {
      found = true;
      break;
    }

    unsigned int cx, cy;
    toXY(cur.idx, cx, cy);

    // Direct goal connection if line-of-sight is valid.
    double goal_segment_cost = 0.0;
    if (lineSegmentCost(cm_copy, cx, cy, gx, gy, goal_segment_cost)) {
      const double tentative = cur.g + goal_segment_cost;

      if (tentative < gscore[gidx]) {
        gscore[gidx] = tentative;
        parent[gidx] = static_cast<int>(cur.idx);

        open.push(OpenNode{
          gidx,
          tentative,
          tentative});
      }
    }

    // Ray macro-action expansion.
    for (int k = 0; k < ray_angle_bins_; ++k) {
      const double angle =
        TWO_PI * static_cast<double>(k) / static_cast<double>(ray_angle_bins_);

      const RayResult ray = castRay(cm_copy, cx, cy, angle);
      if (!ray.valid) {
        continue;
      }

      const unsigned int nidx = toIndex(ray.x, ray.y);
      if (nidx == cur.idx || closed[nidx]) {
        continue;
      }

      const double tentative = cur.g + ray.cost;

      if (tentative < gscore[nidx]) {
        gscore[nidx] = tentative;
        parent[nidx] = static_cast<int>(cur.idx);

        const double h = heuristic(
          static_cast<int>(ray.x),
          static_cast<int>(ray.y),
          static_cast<int>(gx),
          static_cast<int>(gy),
          cm_copy.getResolution());

        open.push(OpenNode{
          nidx,
          tentative + weight_h_ * h * tie_breaker_,
          tentative});
      }
    }
  }

  if (!found) {
    ROS_WARN_STREAM(
      "[AStarRayPlannerROS] failed. expanded_nodes=" << expanded_nodes
      << " start=(" << sx << "," << sy << ")"
      << " goal=(" << gx << "," << gy << ")");
    return false;
  }

  // Reconstruct sparse segment path.
  std::vector<unsigned int> rev;
  for (int idx = static_cast<int>(gidx); idx >= 0; idx = parent[static_cast<unsigned int>(idx)]) {
    rev.push_back(static_cast<unsigned int>(idx));

    if (static_cast<unsigned int>(idx) == sidx) {
      break;
    }
  }

  std::reverse(rev.begin(), rev.end());

  if (rev.empty() || rev.front() != sidx || rev.back() != gidx) {
    ROS_WARN("[AStarRayPlannerROS] reconstruction failed");
    return false;
  }

  plan.clear();
  plan.reserve(rev.size() * 4 + 2);

  geometry_msgs::PoseStamped start_pose = start;
  start_pose.header.stamp = stamp;
  start_pose.header.frame_id = frame;
  plan.push_back(start_pose);

  auto makePose = [&](double wx, double wy) {
    geometry_msgs::PoseStamped p;
    p.header.stamp = stamp;
    p.header.frame_id = frame;
    p.pose.position.x = wx;
    p.pose.position.y = wy;
    p.pose.position.z = 0.0;
    p.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    return p;
  };

  auto appendSegment = [&](const geometry_msgs::PoseStamped& target) {
    if (plan.empty()) {
      plan.push_back(target);
      return;
    }

    const auto& prev = plan.back().pose.position;
    const auto& next = target.pose.position;

    const double dx = next.x - prev.x;
    const double dy = next.y - prev.y;
    const double dist = std::hypot(dx, dy);

    if (dist < 1e-6) {
      return;
    }

    const int n_samples =
      std::max(0, static_cast<int>(std::floor(dist / resample_interval_)));

    for (int i = 1; i <= n_samples; ++i) {
      const double d = static_cast<double>(i) * resample_interval_;

      if (d >= dist) {
        break;
      }

      const double t = d / dist;

      geometry_msgs::PoseStamped p;
      p.header.stamp = stamp;
      p.header.frame_id = frame;
      p.pose.position.x = prev.x + t * dx;
      p.pose.position.y = prev.y + t * dy;
      p.pose.position.z = 0.0;
      p.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
      plan.push_back(p);
    }

    plan.push_back(target);
  };

  for (std::size_t i = 1; i < rev.size(); ++i) {
    geometry_msgs::PoseStamped target;

    if (i + 1 == rev.size()) {
      target = goal;
      target.header.stamp = stamp;
      target.header.frame_id = frame;
    } else {
      unsigned int mx, my;
      toXY(rev[i], mx, my);

      double wx, wy;
      cm_copy.mapToWorld(mx, my, wx, wy);
      target = makePose(wx, wy);
    }

    appendSegment(target);
  }

  // Recompute heading orientation.
  if (plan.size() >= 2) {
    for (std::size_t i = 0; i + 1 < plan.size(); ++i) {
      const auto& a = plan[i].pose.position;
      const auto& b = plan[i + 1].pose.position;

      const double dx = b.x - a.x;
      const double dy = b.y - a.y;

      if (std::hypot(dx, dy) < 1e-6) {
        continue;
      }

      const double yaw = yawBetween(a.x, a.y, b.x, b.y);
      plan[i].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    }

    plan.back().pose.orientation = goal.pose.orientation;
  }

  ROS_INFO_STREAM(
    "[AStarRayPlannerROS] path found. segment_nodes=" << rev.size()
    << " final_points=" << plan.size()
    << " expanded_nodes=" << expanded_nodes
    << " ray_bins=" << ray_angle_bins_
    << " max_segment=" << max_segment_length_
    << " resample=" << resample_interval_
    << " line_cost_threshold=" << line_cost_threshold_);

  return true;
}

}  // namespace pongbot_global_planner

PLUGINLIB_EXPORT_CLASS(
  pongbot_global_planner::AStarRayPlannerROS,
  nav_core::BaseGlobalPlanner)