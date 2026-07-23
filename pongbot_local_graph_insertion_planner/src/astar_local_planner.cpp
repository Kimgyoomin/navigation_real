#include "pongbot_local_graph_insertion_planner/astar_local_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pongbot_local_graph_insertion_planner {
namespace {
bool finitePose(const geometry_msgs::msg::Pose & pose) {
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
    std::isfinite(pose.orientation.x) && std::isfinite(pose.orientation.y) &&
    std::isfinite(pose.orientation.z) && std::isfinite(pose.orientation.w);
}
}

void AstarLocalPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent; name_ = std::move(name); tf_ = std::move(tf); costmap_ros_ = std::move(costmap_ros);
  auto node = parent_.lock();
  if (!node || !costmap_ros_) throw std::runtime_error("AstarLocal requires lifecycle node and global costmap");
  logger_ = node->get_logger(); frame_ = costmap_ros_->getGlobalFrameID();
  auto declare = [&node, this](const std::string & key, auto value) {
    return node->declare_parameter(name_ + "." + key, value);
  };
  local_costmap_topic_ = declare("local_costmap_topic", local_costmap_topic_);
  require_local_costmap_ = declare("require_local_costmap", require_local_costmap_);
  local_costmap_timeout_ = declare("local_costmap_timeout", local_costmap_timeout_);
  transform_timeout_ = declare("transform_timeout", transform_timeout_);
  allow_latest_transform_fallback_ = declare("allow_latest_transform_fallback", allow_latest_transform_fallback_);
  allow_unknown_ = declare("allow_unknown", allow_unknown_);
  blocked_cost_threshold_ = declare("blocked_cost_threshold", blocked_cost_threshold_);
  const auto ratio = declare("incremental_change_ratio_threshold", 0.20);
  max_planning_time_ = declare("max_planning_time", max_planning_time_);
  (void)declare("enable_shortcutting", false);
  publish_debug_fused_grid_ = declare("publish_debug_fused_grid", publish_debug_fused_grid_);
  debug_fused_grid_topic_ = declare("debug_fused_grid_topic", debug_fused_grid_topic_);
  if (blocked_cost_threshold_ < 1 || blocked_cost_threshold_ > 255 || local_costmap_timeout_ <= 0.0 ||
    transform_timeout_ < 0.0 || max_planning_time_ <= 0.0 || ratio < 0.0 || ratio > 1.0)
    throw std::runtime_error("AstarLocal parameter contract violation");
  dstar_ = DStarLite(ratio);
  local_subscription_ = node->create_subscription<nav2_msgs::msg::Costmap>(local_costmap_topic_,
    rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&AstarLocalPlanner::localCostmapCallback, this, std::placeholders::_1));
  if (publish_debug_fused_grid_)
    debug_publisher_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(debug_fused_grid_topic_, rclcpp::QoS(1).transient_local().reliable());
}
void AstarLocalPlanner::cleanup() {
  std::lock_guard<std::mutex> lock(planner_mutex_); local_subscription_.reset(); debug_publisher_.reset();
  dstar_ = DStarLite(0.20); previous_fused_ = {}; costmap_ros_.reset(); tf_.reset();
}
void AstarLocalPlanner::activate() {}
void AstarLocalPlanner::deactivate() {}
void AstarLocalPlanner::localCostmapCallback(nav2_msgs::msg::Costmap::SharedPtr message) {
  std::lock_guard<std::mutex> lock(local_mutex_); local_snapshot_ = {std::move(message), rclcpp::Clock(RCL_ROS_TIME).now()};
}

GridSnapshot AstarLocalPlanner::copyGlobalCostmap() const {
  GridSnapshot out; auto * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
  out.frame_id = frame_; out.size_x = costmap->getSizeInCellsX(); out.size_y = costmap->getSizeInCellsY();
  out.resolution = costmap->getResolution(); out.origin_x = costmap->getOriginX(); out.origin_y = costmap->getOriginY();
  const auto cells = out.size_x * out.size_y; out.costs.assign(costmap->getCharMap(), costmap->getCharMap() + cells);
  return out;
}

bool AstarLocalPlanner::fuseLocalOverlay(GridSnapshot & fused, std::size_t & overlay_cells,
  double & local_age, std::string & failure) const
{
  LocalSnapshot local; { std::lock_guard<std::mutex> lock(local_mutex_); local = local_snapshot_; }
  const auto node = parent_.lock(); const auto now = node->now();
  if (!local.message) { failure = "local_costmap_missing"; return !require_local_costmap_; }
  local_age = (now - rclcpp::Time(local.message->header.stamp)).seconds();
  if (!std::isfinite(local_age) || local_age > local_costmap_timeout_) { failure = "local_costmap_stale"; return !require_local_costmap_; }
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(frame_, local.message->header.frame_id,
      rclcpp::Time(local.message->header.stamp), rclcpp::Duration::from_seconds(transform_timeout_));
  } catch (const tf2::TransformException & e) {
    if (!allow_latest_transform_fallback_) { failure = std::string("local_tf_failed: ") + e.what(); return false; }
    try { transform = tf_->lookupTransform(frame_, local.message->header.frame_id, rclcpp::Time(0, 0, RCL_ROS_TIME),
      rclcpp::Duration::from_seconds(transform_timeout_)); }
    catch (const tf2::TransformException & latest) { failure = std::string("local_tf_latest_failed: ") + latest.what(); return false; }
  }
  const auto & m = *local.message; const auto w = m.metadata.size_x, h = m.metadata.size_y;
  if (!w || !h || m.data.size() != static_cast<std::size_t>(w) * h || m.metadata.resolution <= 0.0) {
    failure = "local_costmap_invalid"; return false;
  }
  tf2::Transform global_from_local, local_origin;
  tf2::fromMsg(transform.transform, global_from_local); tf2::fromMsg(m.metadata.origin, local_origin);
  for (std::size_t y = 0; y < h; ++y) for (std::size_t x = 0; x < w; ++x) {
    const auto cost = m.data[y * w + x];
    if (cost == 255 || cost == 0) continue;  // unknown ignored; free never clears global obstacles
    const bool blocked = cost >= static_cast<unsigned char>(blocked_cost_threshold_);
    double min_x=std::numeric_limits<double>::infinity(), min_y=min_x, max_x=-min_x, max_y=-min_x;
    for (int iy=0;iy<2;++iy) for (int ix=0;ix<2;++ix) {
      const auto point = global_from_local * local_origin * tf2::Vector3((x+ix)*m.metadata.resolution,(y+iy)*m.metadata.resolution,0);
      min_x=std::min(min_x,point.x()); max_x=std::max(max_x,point.x()); min_y=std::min(min_y,point.y()); max_y=std::max(max_y,point.y());
    }
    const auto ix0=static_cast<long long>(std::floor((min_x-fused.origin_x)/fused.resolution));
    const auto iy0=static_cast<long long>(std::floor((min_y-fused.origin_y)/fused.resolution));
    const auto ix1=static_cast<long long>(std::floor((max_x-fused.origin_x)/fused.resolution));
    const auto iy1=static_cast<long long>(std::floor((max_y-fused.origin_y)/fused.resolution));
    for(long long gy=iy0;gy<=iy1;++gy)for(long long gx=ix0;gx<=ix1;++gx)if(gx>=0&&gy>=0&&gx<static_cast<long long>(fused.size_x)&&gy<static_cast<long long>(fused.size_y)){
      auto & target=fused.costs[fused.index(static_cast<std::size_t>(gx),static_cast<std::size_t>(gy))];
      const auto before=target; target=blocked ? std::max(target,static_cast<unsigned char>(blocked_cost_threshold_)) : std::max(target,cost);
      if(target!=before)++overlay_cells;
    }
  }
  return true;
}
void AstarLocalPlanner::publishFusedGrid(const GridSnapshot & grid) const {
  if (!debug_publisher_) return;
  nav_msgs::msg::OccupancyGrid message;message.header.stamp=parent_.lock()->now();message.header.frame_id=grid.frame_id;
  message.info.resolution=grid.resolution;message.info.width=grid.size_x;message.info.height=grid.size_y;message.info.origin.position.x=grid.origin_x;message.info.origin.position.y=grid.origin_y;message.info.origin.orientation.w=1.0;
  message.data.resize(grid.costs.size());for(std::size_t i=0;i<grid.costs.size();++i)message.data[i]=grid.costs[i]==255?-1:static_cast<int8_t>(std::min(100,static_cast<int>(grid.costs[i])*100/252));
  debug_publisher_->publish(message);
}
geometry_msgs::msg::Quaternion AstarLocalPlanner::normalizedQuaternion(double yaw) {
  tf2::Quaternion q;q.setRPY(0,0,yaw);return tf2::toMsg(q);
}
nav_msgs::msg::Path AstarLocalPlanner::createPlan(const geometry_msgs::msg::PoseStamped & start,const geometry_msgs::msg::PoseStamped & goal) {
  nav_msgs::msg::Path path;path.header.frame_id=frame_;path.header.stamp=parent_.lock()->now();
  std::lock_guard<std::mutex> lock(planner_mutex_);
  if(start.header.frame_id!=frame_||goal.header.frame_id!=frame_||!finitePose(start.pose)||!finitePose(goal.pose)){RCLCPP_ERROR(logger_,"[AstarLocal] invalid endpoint/frame");return path;}
  auto fused=copyGlobalCostmap();std::size_t overlay_cells=0;double local_age=-1.0;std::string failure;
  if(!fuseLocalOverlay(fused,overlay_cells,local_age,failure)){RCLCPP_WARN(logger_,"[AstarLocal] planning failure: %s",failure.c_str());return path;}
  const auto toCell=[&fused](const geometry_msgs::msg::Pose & pose,std::size_t & c){const auto x=static_cast<long long>(std::floor((pose.position.x-fused.origin_x)/fused.resolution));const auto y=static_cast<long long>(std::floor((pose.position.y-fused.origin_y)/fused.resolution));if(x<0||y<0||x>=static_cast<long long>(fused.size_x)||y>=static_cast<long long>(fused.size_y))return false;c=fused.index(x,y);return true;};
  std::size_t s=0,g=0;if(!toCell(start.pose,s)||!toCell(goal.pose,g)){RCLCPP_WARN(logger_,"[AstarLocal] endpoint out of bounds");return path;}
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(max_planning_time_);
  SearchOptions options;options.allow_unknown=allow_unknown_;options.blocked_cost_threshold=static_cast<unsigned char>(blocked_cost_threshold_);options.cancelled=[deadline]{return std::chrono::steady_clock::now()>deadline;};
  std::size_t changed_cells = fused.costs.size();
  if (previous_fused_.valid() && previous_fused_.geometryEquals(fused)) {
    changed_cells = 0;
    for (std::size_t i = 0; i < fused.costs.size(); ++i) {
      changed_cells += fused.costs[i] != previous_fused_.costs[i];
    }
  }
  const auto result=dstar_.replan(fused,s,g,options);publishFusedGrid(fused);
  const auto mode=dstar_.lastFallbackReason()==FallbackReason::kChangedRatio?"FRESH_FALLBACK":(dstar_.lastFallbackReason()==FallbackReason::kInitialPlan?"FRESH_INIT":"DSTAR_REPAIR");
  RCLCPP_INFO(logger_,"[AstarLocal] mode=%s changed_cells=%zu overlay_cells=%zu local_age=%.3f fallback_reason=%s path_cost=%.6f path_size=%zu",mode,changed_cells,overlay_cells,local_age,fallbackReasonName(dstar_.lastFallbackReason()),result.cost,result.path.size());
  previous_fused_=fused;if(result.status!=SearchStatus::kSuccess||!validPath(fused,result.path,options)){RCLCPP_WARN(logger_,"[AstarLocal] planning failure: %s",result.reason.c_str());return path;}
  path.poses.reserve(result.path.size());for(std::size_t i=0;i<result.path.size();++i){geometry_msgs::msg::PoseStamped pose;pose.header=path.header;const auto cell=result.path[i];pose.pose.position.x=fused.origin_x+(fused.x(cell)+0.5)*fused.resolution;pose.pose.position.y=fused.origin_y+(fused.y(cell)+0.5)*fused.resolution;pose.pose.orientation=normalizedQuaternion(i+1<result.path.size()?std::atan2((static_cast<double>(fused.y(result.path[i+1]))-static_cast<double>(fused.y(cell)))*fused.resolution,(static_cast<double>(fused.x(result.path[i+1]))-static_cast<double>(fused.x(cell)))*fused.resolution):tf2::getYaw(goal.pose.orientation));path.poses.push_back(pose);}
  path.poses.front()=start;path.poses.back()=goal;return path;
}
}  // namespace pongbot_local_graph_insertion_planner
PLUGINLIB_EXPORT_CLASS(pongbot_local_graph_insertion_planner::AstarLocalPlanner, nav2_core::GlobalPlanner)
