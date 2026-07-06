#include "pongbot_global_planner/astar_sparse_planner.h"

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
#include <mutex>
#include <queue>
#include <set>
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

AStarSparsePlannerROS::AStarSparsePlannerROS() {}

AStarSparsePlannerROS::AStarSparsePlannerROS(
  std::string name,
  costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}

void AStarSparsePlannerROS::initialize(
  std::string name,
  costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_) {
    return;
  }

  costmap_ros_ = costmap_ros;

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
  pnh.param("prevent_diagonal_corner_cutting", prevent_diagonal_corner_cutting_, true);
  pnh.param("print_timing", print_timing_, true);

  pnh.param("simplification_epsilon", simplification_epsilon_, 0.30);
  pnh.param("max_segment_length", max_segment_length_, 1.0);
  pnh.param("min_segment_length", min_segment_length_, 0.05);
  pnh.param("line_cost_threshold", line_cost_threshold_, 220);
  pnh.param("max_rdp_depth", max_rdp_depth_, 20);
  pnh.param("enable_local_splice", enable_local_splice_, true);
  pnh.param("reference_goal_tolerance", reference_goal_tolerance_, 0.30);
  pnh.param("local_splice_horizon", local_splice_horizon_, 4.0);
  pnh.param("local_splice_min_rejoin_dist", local_splice_min_rejoin_dist_, 1.0);
  pnh.param("local_splice_cost_threshold", local_splice_cost_threshold_, 160);

  reference_goal_tolerance_ = std::max(0.05, reference_goal_tolerance_);
  local_splice_horizon_ = std::max(0.5, local_splice_horizon_);
  local_splice_min_rejoin_dist_ = std::max(0.1, local_splice_min_rejoin_dist_);
  local_splice_cost_threshold_ = std::max(1, std::min(252, local_splice_cost_threshold_));

  simplification_epsilon_ = std::max(0.01, simplification_epsilon_);
  max_segment_length_ = std::max(0.10, max_segment_length_);
  min_segment_length_ = std::max(0.01, min_segment_length_);
  line_cost_threshold_ = std::max(1, std::min(252, line_cost_threshold_));
  max_rdp_depth_ = std::max(1, max_rdp_depth_);
  cost_weight_alpha_ = std::max(0.0, cost_weight_alpha_);
  weight_h_ = std::max(0.0, weight_h_);
  tie_breaker_ = std::max(1.0, tie_breaker_);

  initialized_ = true;

  ROS_INFO_STREAM(
    "pongbot_global_planner::AStarSparsePlannerROS initialized under ~/" << name
    << " allow_unknown=" << allow_unknown_
    << " octile=" << use_octile_heuristic_
    << " lethal_cost=" << static_cast<int>(lethal_cost_)
    << " tie=" << tie_breaker_
    << " alpha=" << cost_weight_alpha_
    << " w_h=" << weight_h_
    << " epsilon=" << simplification_epsilon_
    << " max_segment=" << max_segment_length_
    << " line_cost_threshold=" << line_cost_threshold_
    << " local_splice=" << enable_local_splice_
    << " local_splice_horizon=" << local_splice_horizon_
    << " local_splice_threshold=" << local_splice_cost_threshold_);
}

inline bool AStarSparsePlannerROS::isCellFree(
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

inline bool AStarSparsePlannerROS::isCellSafeForLine(
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

inline double AStarSparsePlannerROS::heuristic(
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

inline double AStarSparsePlannerROS::yawBetween(
  double x0,
  double y0,
  double x1,
  double y1)
{
  return std::atan2(y1 - y0, x1 - x0);
}

bool AStarSparsePlannerROS::snapToFree(
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

bool AStarSparsePlannerROS::lineSegmentValid(
  const costmap_2d::Costmap2D& cm,
  unsigned int x0,
  unsigned int y0,
  unsigned int x1,
  unsigned int y1) const
{
  double wx0, wy0, wx1, wy1;
  cm.mapToWorld(x0, y0, wx0, wy0);
  cm.mapToWorld(x1, y1, wx1, wy1);

  const double dx = wx1 - wx0;
  const double dy = wy1 - wy0;
  const double dist = std::hypot(dx, dy);

  if (dist < 1e-9) {
    return true;
  }

  const double resolution = cm.getResolution();
  const double sample_step = std::max(0.5 * resolution, 0.01);
  const int steps = std::max(1, static_cast<int>(std::ceil(dist / sample_step)));

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
      if (!isCellSafeForLine(cm, mx, my)) {
        return false;
      }
    }
  }

  return true;
}

double AStarSparsePlannerROS::pointLineDistanceWorld(
  const costmap_2d::Costmap2D& cm,
  unsigned int point_idx,
  unsigned int line_start_idx,
  unsigned int line_end_idx) const
{
  const unsigned int W = cm.getSizeInCellsX();

  const unsigned int px_m = point_idx % W;
  const unsigned int py_m = point_idx / W;
  const unsigned int ax_m = line_start_idx % W;
  const unsigned int ay_m = line_start_idx / W;
  const unsigned int bx_m = line_end_idx % W;
  const unsigned int by_m = line_end_idx / W;

  double px, py, ax, ay, bx, by;
  cm.mapToWorld(px_m, py_m, px, py);
  cm.mapToWorld(ax_m, ay_m, ax, ay);
  cm.mapToWorld(bx_m, by_m, bx, by);

  const double vx = bx - ax;
  const double vy = by - ay;
  const double wx = px - ax;
  const double wy = py - ay;

  const double c2 = vx * vx + vy * vy;
  if (c2 < 1e-12) {
    return std::hypot(px - ax, py - ay);
  }

  const double t = std::max(0.0, std::min(1.0, (wx * vx + wy * vy) / c2));
  const double proj_x = ax + t * vx;
  const double proj_y = ay + t * vy;

  return std::hypot(px - proj_x, py - proj_y);
}

bool AStarSparsePlannerROS::runDenseAstar(
  const costmap_2d::Costmap2D& cm,
  unsigned int sx,
  unsigned int sy,
  unsigned int gx,
  unsigned int gy,
  std::vector<unsigned int>& dense_indices) const
{
  dense_indices.clear();

  const unsigned int W = cm.getSizeInCellsX();
  const unsigned int H = cm.getSizeInCellsY();
  const unsigned int N = W * H;
  const double resolution = cm.getResolution();

  auto toIndex = [W](unsigned int x, unsigned int y) -> unsigned int {
    return y * W + x;
  };

  auto toXY = [W](unsigned int idx, unsigned int& x, unsigned int& y) {
    x = idx % W;
    y = idx / W;
  };

  const unsigned int sidx = toIndex(sx, sy);
  const unsigned int gidx = toIndex(gx, gy);

  std::vector<double> gscore(N, std::numeric_limits<double>::infinity());
  std::vector<int> parent(N, -1);
  std::vector<char> closed(N, 0);

  std::priority_queue<OpenNode> open;

  gscore[sidx] = 0.0;
  open.push(OpenNode{
    sidx,
    weight_h_ * heuristic(static_cast<int>(sx), static_cast<int>(sy),
                          static_cast<int>(gx), static_cast<int>(gy),
                          resolution) * tie_breaker_,
    0.0});

  static const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  bool found = false;

  while (!open.empty()) {
    const OpenNode cur = open.top();
    open.pop();

    if (std::abs(cur.g - gscore[cur.idx]) > 1e-9) {
      continue;
    }

    if (closed[cur.idx]) {
      continue;
    }

    closed[cur.idx] = 1;

    if (cur.idx == gidx) {
      found = true;
      break;
    }

    unsigned int cx, cy;
    toXY(cur.idx, cx, cy);

    for (int k = 0; k < 8; ++k) {
      const int tx = static_cast<int>(cx) + dx8[k];
      const int ty = static_cast<int>(cy) + dy8[k];

      if (tx < 0 || ty < 0 ||
          tx >= static_cast<int>(W) ||
          ty >= static_cast<int>(H)) {
        continue;
      }

      const unsigned int nx = static_cast<unsigned int>(tx);
      const unsigned int ny = static_cast<unsigned int>(ty);

      if (!isCellFree(cm, nx, ny)) {
        continue;
      }

      if (prevent_diagonal_corner_cutting_ && dx8[k] != 0 && dy8[k] != 0) {
        const unsigned int adj_x = static_cast<unsigned int>(static_cast<int>(cx) + dx8[k]);
        const unsigned int adj_y = static_cast<unsigned int>(static_cast<int>(cy) + dy8[k]);

        if (!isCellFree(cm, adj_x, cy) || !isCellFree(cm, cx, adj_y)) {
          continue;
        }
      }

      const unsigned int nidx = toIndex(nx, ny);
      if (closed[nidx]) {
        continue;
      }

      const double step_len =
        (dx8[k] != 0 && dy8[k] != 0) ? std::sqrt(2.0) * resolution : resolution;

      const unsigned char c = cm.getCost(nx, ny);
      const double norm = std::min(1.0, static_cast<double>(c) / 252.0);
      const double step_cost = step_len + step_len * cost_weight_alpha_ * norm;

      const double tentative = cur.g + step_cost;

      if (tentative < gscore[nidx]) {
        gscore[nidx] = tentative;
        parent[nidx] = static_cast<int>(cur.idx);

        const double h = heuristic(
          static_cast<int>(nx),
          static_cast<int>(ny),
          static_cast<int>(gx),
          static_cast<int>(gy),
          resolution);

        open.push(OpenNode{
          nidx,
          tentative + weight_h_ * h * tie_breaker_,
          tentative});
      }
    }
  }

  if (!found) {
    return false;
  }

  std::vector<unsigned int> rev;
  for (int idx = static_cast<int>(gidx); idx >= 0; idx = parent[static_cast<unsigned int>(idx)]) {
    rev.push_back(static_cast<unsigned int>(idx));

    if (static_cast<unsigned int>(idx) == sidx) {
      break;
    }
  }

  std::reverse(rev.begin(), rev.end());

  if (rev.empty() || rev.front() != sidx || rev.back() != gidx) {
    return false;
  }

  dense_indices = rev;
  return true;
}

void AStarSparsePlannerROS::simplifyRDPRecursive(
  const costmap_2d::Costmap2D& cm,
  const std::vector<unsigned int>& dense_indices,
  std::size_t start_pos,
  std::size_t end_pos,
  int depth,
  std::vector<std::size_t>& keep_positions) const
{
  if (end_pos <= start_pos + 1) {
    keep_positions.push_back(end_pos);
    return;
  }

  if (depth >= max_rdp_depth_) {
    for (std::size_t i = start_pos + 1; i <= end_pos; ++i) {
      keep_positions.push_back(i);
    }
    return;
  }

  const unsigned int W = cm.getSizeInCellsX();

  auto toXY = [W](unsigned int idx, unsigned int& x, unsigned int& y) {
    x = idx % W;
    y = idx / W;
  };

  unsigned int x0, y0, x1, y1;
  toXY(dense_indices[start_pos], x0, y0);
  toXY(dense_indices[end_pos], x1, y1);

  const bool line_valid = lineSegmentValid(cm, x0, y0, x1, y1);

  double max_dist = -1.0;
  std::size_t max_pos = (start_pos + end_pos) / 2;

  for (std::size_t i = start_pos + 1; i < end_pos; ++i) {
    const double d = pointLineDistanceWorld(
      cm,
      dense_indices[i],
      dense_indices[start_pos],
      dense_indices[end_pos]);

    if (d > max_dist) {
      max_dist = d;
      max_pos = i;
    }
  }

  if (line_valid && max_dist <= simplification_epsilon_) {
    keep_positions.push_back(end_pos);
    return;
  }

  simplifyRDPRecursive(cm, dense_indices, start_pos, max_pos, depth + 1, keep_positions);
  simplifyRDPRecursive(cm, dense_indices, max_pos, end_pos, depth + 1, keep_positions);
}

bool AStarSparsePlannerROS::makePlan(
  const geometry_msgs::PoseStamped& start,
  const geometry_msgs::PoseStamped& goal,
  std::vector<geometry_msgs::PoseStamped>& plan)
{
  plan.clear();

  if (!initialized_) {
    ROS_ERROR("[AStarSparsePlannerROS] makePlan called but not initialized");
    return false;
  }

  if (!costmap_ros_) {
    ROS_ERROR("[AStarSparsePlannerROS] costmap_ros_ is null");
    return false;
  }

  const std::string frame = costmap_ros_->getGlobalFrameID();

  costmap_2d::Costmap2D local_copy;
  {
    costmap_2d::Costmap2D* cm = costmap_ros_->getCostmap();
    std::lock_guard<costmap_2d::Costmap2D::mutex_t> lk(*(cm->getMutex()));
    local_copy = *cm;
  }

  // const ros::Time stamp = ros::Time::now();
  const ros::Time stamp = ros::Time(0);

  const auto t0 = std::chrono::steady_clock::now();

  // LOCAL - GLOBAL Graph 통합을 위해 추가 260706
  const bool goal_changed = goalChangedFromReference(goal);

  if (enable_local_splice_ && has_reference_plan_ && !goal_changed) {
    if (tryMakeLocalSplicePlan(local_copy, start, goal, frame, stamp, plan)) {
      const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();

      std_msgs::Float64 dt;
      dt.data = ms;
      plan_time_pub_.publish(dt);

      for (auto& p : plan) {
        p.header.stamp = stamp;
        p.header.frame_id = frame;
      }

      nav_msgs::Path path_msg;
      path_msg.header.stamp = stamp;
      path_msg.header.frame_id = frame;
      path_msg.poses = plan;
      plan_pub_.publish(path_msg);

      if (print_timing_) {
        ROS_INFO_STREAM_NAMED(
          "pongbot_astar_sparse",
          "\033[1;32m"
          << "[Sparse A* Local Splice] planning_time = "
          << std::fixed << std::setprecision(3) << ms << " ms"
          << "  (poses=" << plan.size() << ")"
          << "\033[0m");
      }

      return true;
    }
  }

  const bool ok = searchSparseAstar(local_copy, start, goal, plan, frame, stamp);

  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();

  std_msgs::Float64 dt;
  dt.data = ms;
  plan_time_pub_.publish(dt);

  if (print_timing_) {
    std::ostringstream oss;
    oss << "\033[1;36m"
        << "[Sparse A*] planning_time = "
        << std::fixed << std::setprecision(3) << ms << " ms"
        << "  (poses=" << plan.size() << ")"
        << "\033[0m";
    ROS_INFO_STREAM_NAMED("pongbot_astar_sparse", oss.str());
  }

  if (ok) {
    for (auto& p : plan) {
      p.header.stamp = stamp;
      p.header.frame_id = frame;
    }

    // Local - Global Graph insertion
    reference_plan_ = plan;
    has_reference_plan_ = true;

    nav_msgs::Path path_msg;
    path_msg.header.stamp = stamp;
    path_msg.header.frame_id = frame;
    path_msg.poses = plan;
    plan_pub_.publish(path_msg);
  } else {
    ROS_WARN("[AStarSparsePlannerROS] no path found");
  }

  return ok;
}

// PoseDistance 2D / Local - Graph insertion
double AStarSparsePlannerROS::poseDistance2D(
  const geometry_msgs::PoseStamped& a,
  const geometry_msgs::PoseStamped& b)
{
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  return std::hypot(dx, dy);
}

// find Closest Plan Index
std::size_t AStarSparsePlannerROS::findClosestPlanIndex(
  const std::vector<geometry_msgs::PoseStamped>& plan,
  const geometry_msgs::PoseStamped& pose) const
{
  std::size_t best_idx = 0;
  double best_dist = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < plan.size(); ++i) {
    const double d = poseDistance2D(plan[i], pose);
    if (d < best_dist) {
      best_dist = d;
      best_idx = i;
    }
  }

  return best_idx;
}

// poseSafeByCost
bool AStarSparsePlannerROS::poseSafeByCost(
  const costmap_2d::Costmap2D& cm,
  const geometry_msgs::PoseStamped& pose,
  const int cost_threshold) const
{
  unsigned int mx, my;
  if (!cm.worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
    return false;
  }

  const unsigned char c = cm.getCost(mx, my);

  if (c == costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }

  if (c >= lethal_cost_) {
    return false;
  }

  return static_cast<int>(c) <= cost_threshold;
}

// plan Segment Safe by Cost
bool AStarSparsePlannerROS::planSegmentSafeByCost(
  const costmap_2d::Costmap2D& cm,
  const geometry_msgs::PoseStamped& a,
  const geometry_msgs::PoseStamped& b,
  const int cost_threshold) const
{
  const double ax = a.pose.position.x;
  const double ay = a.pose.position.y;
  const double bx = b.pose.position.x;
  const double by = b.pose.position.y;

  const double dx = bx - ax;
  const double dy = by - ay;
  const double dist = std::hypot(dx, dy);

  if (dist < 1e-9) {
    return true;
  }

  const double resolution = cm.getResolution();
  const double step = std::max(0.5 * resolution, 0.01);
  const int steps = std::max(1, static_cast<int>(std::ceil(dist / step)));

  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const double wx = ax + t * dx;
    const double wy = ay + t * dy;

    unsigned int mx, my;
    if (!cm.worldToMap(wx, wy, mx, my)) {
      return false;
    }

    const unsigned char c = cm.getCost(mx, my);

    if (c == costmap_2d::NO_INFORMATION) {
      if (!allow_unknown_) {
        return false;
      }
      continue;
    }

    if (c >= lethal_cost_) {
      return false;
    }

    if (static_cast<int>(c) > cost_threshold) {
      return false;
    }
  }

  return true;
}

// try Make Local Splice Plan
bool AStarSparsePlannerROS::tryMakeLocalSplicePlan(
  const costmap_2d::Costmap2D& cm,
  const geometry_msgs::PoseStamped& start,
  const geometry_msgs::PoseStamped& goal,
  const std::string& frame,
  const ros::Time& stamp,
  std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!has_reference_plan_ || reference_plan_.size() < 2) {
    return false;
  }

  const std::size_t closest_idx =
    findClosestPlanIndex(reference_plan_, start);

  if (closest_idx >= reference_plan_.size() - 1) {
    return false;
  }

  bool blocked = false;
  std::size_t last_bad_idx = closest_idx;
  std::size_t horizon_idx = closest_idx;

  double accumulated = 0.0;
  geometry_msgs::PoseStamped prev = start;
  prev.header.frame_id = frame;
  prev.header.stamp = stamp;

  for (std::size_t i = closest_idx; i < reference_plan_.size(); ++i) {
    geometry_msgs::PoseStamped cur = reference_plan_[i];
    cur.header.frame_id = frame;
    cur.header.stamp = stamp;

    const double seg_len = poseDistance2D(prev, cur);
    accumulated += seg_len;

    if (accumulated > local_splice_horizon_) {
      horizon_idx = i;
      break;
    }

    if (!planSegmentSafeByCost(cm, prev, cur, local_splice_cost_threshold_)) {
      blocked = true;
      last_bad_idx = i;
    }

    prev = cur;
    horizon_idx = i;
  }

  if (!blocked) {
    plan.clear();

    geometry_msgs::PoseStamped start_pose = start;
    start_pose.header.frame_id = frame;
    start_pose.header.stamp = stamp;
    plan.push_back(start_pose);

    for (std::size_t i = closest_idx + 1; i < reference_plan_.size(); ++i) {
      geometry_msgs::PoseStamped p = reference_plan_[i];
      p.header.frame_id = frame;
      p.header.stamp = stamp;
      plan.push_back(p);
    }

    return plan.size() >= 2;
  }

  std::size_t rejoin_idx = last_bad_idx + 1;
  if (rejoin_idx >= reference_plan_.size()) {
    return false;
  }

  double rejoin_dist = 0.0;
  for (std::size_t i = closest_idx + 1; i <= rejoin_idx && i < reference_plan_.size(); ++i) {
    rejoin_dist += poseDistance2D(reference_plan_[i - 1], reference_plan_[i]);
  }

  while (rejoin_idx < reference_plan_.size()) {
    geometry_msgs::PoseStamped candidate = reference_plan_[rejoin_idx];
    candidate.header.frame_id = frame;
    candidate.header.stamp = stamp;

    const bool far_enough = rejoin_dist >= local_splice_min_rejoin_dist_;
    const bool within_horizon = rejoin_idx <= horizon_idx;
    const bool safe = poseSafeByCost(cm, candidate, local_splice_cost_threshold_);

    if (far_enough && within_horizon && safe) {
      break;
    }

    if (rejoin_idx + 1 >= reference_plan_.size()) {
      return false;
    }

    rejoin_dist += poseDistance2D(reference_plan_[rejoin_idx], reference_plan_[rejoin_idx + 1]);
    ++rejoin_idx;
  }

  if (rejoin_idx >= reference_plan_.size() || rejoin_idx > horizon_idx) {
    return false;
  }

  geometry_msgs::PoseStamped rejoin_goal = reference_plan_[rejoin_idx];
  rejoin_goal.header.frame_id = frame;
  rejoin_goal.header.stamp = stamp;

  std::vector<geometry_msgs::PoseStamped> detour;
  if (!searchSparseAstar(cm, start, rejoin_goal, detour, frame, stamp)) {
    ROS_WARN("[AStarSparsePlannerROS] local splice detour planning failed");
    return false;
  }

  if (detour.empty()) {
    return false;
  }

  plan.clear();
  plan.reserve(detour.size() + reference_plan_.size() - rejoin_idx);

  for (const auto& p_in : detour) {
    geometry_msgs::PoseStamped p = p_in;
    p.header.frame_id = frame;
    p.header.stamp = stamp;
    plan.push_back(p);
  }

  for (std::size_t i = rejoin_idx + 1; i < reference_plan_.size(); ++i) {
    geometry_msgs::PoseStamped p = reference_plan_[i];
    p.header.frame_id = frame;
    p.header.stamp = stamp;
    plan.push_back(p);
  }

  if (plan.size() >= 2) {
    for (std::size_t i = 0; i + 1 < plan.size(); ++i) {
      const auto& a = plan[i].pose.position;
      const auto& b = plan[i + 1].pose.position;
      const double yaw = yawBetween(a.x, a.y, b.x, b.y);
      plan[i].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    }
    plan.back().pose.orientation = goal.pose.orientation;
  }

  ROS_INFO_STREAM(
    "[AStarSparsePlannerROS] local splice success. rejoin_idx="
    << rejoin_idx
    << " detour_points=" << detour.size()
    << " fused_points=" << plan.size());

  return plan.size() >= 2;
}

bool AStarSparsePlannerROS::searchSparseAstar(
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
    ROS_ERROR("[AStarSparsePlannerROS] empty costmap");
    return false;
  }

  unsigned int sx, sy, gx, gy;

  if (!cm_copy.worldToMap(start.pose.position.x, start.pose.position.y, sx, sy)) {
    ROS_WARN("[AStarSparsePlannerROS] start outside map");
    return false;
  }

  if (!cm_copy.worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
    ROS_WARN("[AStarSparsePlannerROS] goal outside map");
    return false;
  }

  if (!isCellFree(cm_copy, sx, sy)) {
    if (!snapToFree(cm_copy, sx, sy)) {
      ROS_WARN("[AStarSparsePlannerROS] start in obstacle and snap failed");
      return false;
    }
  }

  bool goal_snapped = false;
  if (!isCellFree(cm_copy, gx, gy)) {
    if (!snapToFree(cm_copy, gx, gy)) {
      ROS_WARN("[AStarSparsePlannerROS] goal in obstacle and snap failed");
      return false;
    }
    goal_snapped = true;
  }

  std::vector<unsigned int> dense_indices;
  if (!runDenseAstar(cm_copy, sx, sy, gx, gy, dense_indices)) {
    ROS_WARN_STREAM(
      "[AStarSparsePlannerROS] dense A* failed. start=("
      << sx << "," << sy << ") goal=(" << gx << "," << gy << ")");
    return false;
  }

  if (dense_indices.size() < 2) {
    return false;
  }

  std::vector<std::size_t> keep_positions;
  keep_positions.reserve(dense_indices.size());
  keep_positions.push_back(0);
  simplifyRDPRecursive(
    cm_copy,
    dense_indices,
    0,
    dense_indices.size() - 1,
    0,
    keep_positions);

  std::sort(keep_positions.begin(), keep_positions.end());
  keep_positions.erase(
    std::unique(keep_positions.begin(), keep_positions.end()),
    keep_positions.end());

  plan.clear();
  plan.reserve(keep_positions.size() * 2 + 2);

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

  auto appendPoseWithMaxSegment = [&](const geometry_msgs::PoseStamped& target) {
    if (plan.empty()) {
      plan.push_back(target);
      return;
    }

    const auto& prev = plan.back().pose.position;
    const auto& next = target.pose.position;

    const double dx = next.x - prev.x;
    const double dy = next.y - prev.y;
    const double dist = std::hypot(dx, dy);

    if (dist < min_segment_length_) {
      return;
    }

    const int n_insert =
      std::max(0, static_cast<int>(std::floor(dist / max_segment_length_)));

    for (int i = 1; i <= n_insert; ++i) {
      const double d = static_cast<double>(i) * max_segment_length_;
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

  for (std::size_t i = 0; i < keep_positions.size(); ++i) {
    const std::size_t pos = keep_positions[i];

    geometry_msgs::PoseStamped p;

    if (i == 0) {
      p = start;
      p.header.stamp = stamp;
      p.header.frame_id = frame;
    } else if (i + 1 == keep_positions.size()) {
      if (goal_snapped) {
        unsigned int mx = dense_indices[pos] % W;
        unsigned int my = dense_indices[pos] / W;

        double wx, wy;
        cm_copy.mapToWorld(mx, my, wx, wy);

        p = makePose(wx, wy);
        p.pose.orientation = goal.pose.orientation;
      } else {
        p = goal;
        p.header.stamp = stamp;
        p.header.frame_id = frame;
      }
    } else {
      unsigned int mx = dense_indices[pos] % W;
      unsigned int my = dense_indices[pos] / W;

      double wx, wy;
      cm_copy.mapToWorld(mx, my, wx, wy);
      p = makePose(wx, wy);
    }

    appendPoseWithMaxSegment(p);
  }

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
    "[AStarSparsePlannerROS] dense_points=" << dense_indices.size()
    << " rdp_points=" << keep_positions.size()
    << " final_points=" << plan.size()
    << " epsilon=" << simplification_epsilon_
    << " max_segment=" << max_segment_length_
    << " line_cost_threshold=" << line_cost_threshold_);

  return true;
}

}  // namespace pongbot_global_planner

PLUGINLIB_EXPORT_CLASS(
  pongbot_global_planner::AStarSparsePlannerROS,
  nav_core::BaseGlobalPlanner)