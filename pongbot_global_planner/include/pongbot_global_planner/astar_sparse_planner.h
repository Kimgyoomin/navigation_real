#pragma once

#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <string>
#include <vector>

namespace pongbot_global_planner
{

class AStarSparsePlannerROS : public nav_core::BaseGlobalPlanner
{
public:
  AStarSparsePlannerROS();
  AStarSparsePlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

  bool makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
  bool initialized_ = false;
  costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;

  ros::Publisher plan_pub_;
  ros::Publisher plan_time_pub_;

  

  bool makeCroppedCostmap(
    const costmap_2d::Costmap2D& src,
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    double margin,
    costmap_2d::Costmap2D& dst) const;

  
  bool has_reference_plan_ = false;
 

  std::vector<geometry_msgs::PoseStamped> reference_plan_;

  static double poseDistance2D(
    const geometry_msgs::PoseStamped& a,
    const geometry_msgs::PoseStamped& b);

  bool goalChangedFromReference(
    const geometry_msgs::PoseStamped& goal) const;

  std::size_t findClosestPlanIndex(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const geometry_msgs::PoseStamped& pose) const;

  bool poseSafeByCost(
    const costmap_2d::Costmap2D& cm,
    const geometry_msgs::PoseStamped& pose,
    int cost_threshold) const;

  bool planSegmentSafeByCost(
    const costmap_2d::Costmap2D& cm,
    const geometry_msgs::PoseStamped& a,
    const geometry_msgs::PoseStamped& b,
    int cost_threshold) const;

  bool tryMakeLocalSplicePlan(
    const costmap_2d::Costmap2D& cm,
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    const std::string& frame,
    const ros::Time& stamp,
    std::vector<geometry_msgs::PoseStamped>& plan);

  // Dense A* parameters
  bool allow_unknown_ = false;
  bool use_octile_heuristic_ = true;
  unsigned char lethal_cost_ = 253;
  double tie_breaker_ = 1.001;
  double cost_weight_alpha_ = 5.0;
  double weight_h_ = 1.05;
  bool prevent_diagonal_corner_cutting_ = true;
  bool print_timing_ = true;

  // Collision-aware RDP simplification parameters
  double simplification_epsilon_ = 0.30;   // [m]
  double max_segment_length_ = 1.0;        // [m]
  double min_segment_length_ = 0.05;       // [m]
  int line_cost_threshold_ = 220;          // RDP shortcut이 지나면 안되는 영역

  // Local Splice / reference path reuse
  bool enable_local_splice_ = true;
  double reference_goal_tolerance_ = 0.30;       // [m]
  double local_splice_horizon_ = 4.0;            // [m]
  double local_splice_min_rejoin_dist_ = 1.0;    // [m]

  // Trigger : 기존 reference path가 위험했는지 감지
  int local_splice_trigger_cost_threshold_ = 120;

  // Planning : 새 detour A*에서 cost > threshold를 obstacle로 간주
  // 새 detour path를 만들 때, 진짜 금지할 영역
  int local_splice_planning_cost_threshold_ = 180; 

  bool local_splice_use_soft_cost_trigger_ = true;
  bool local_splice_debug_collision_check_ = false;

  // For LOCAL - GLOBAL graph integration
  bool local_splice_use_roi_ = true;
  double local_splice_roi_margin_ = 3.0;  // [m]

  int max_rdp_depth_ = 20;

  inline bool isCellFree(
    const costmap_2d::Costmap2D& cm,
    unsigned int mx,
    unsigned int my) const;

  inline bool isCellSafeForLine(
    const costmap_2d::Costmap2D& cm,
    unsigned int mx,
    unsigned int my) const;

  inline double heuristic(
    int x0,
    int y0,
    int x1,
    int y1,
    double resolution) const;

  static inline double yawBetween(
    double x0,
    double y0,
    double x1,
    double y1);

  bool snapToFree(
    const costmap_2d::Costmap2D& cm,
    unsigned& mx,
    unsigned& my) const;

  bool lineSegmentValid(
    const costmap_2d::Costmap2D& cm,
    unsigned int x0,
    unsigned int y0,
    unsigned int x1,
    unsigned int y1) const;

  double pointLineDistanceWorld(
    const costmap_2d::Costmap2D& cm,
    unsigned int point_idx,
    unsigned int line_start_idx,
    unsigned int line_end_idx) const;

  bool runDenseAstar(
    const costmap_2d::Costmap2D& cm,
    unsigned int sx,
    unsigned int sy,
    unsigned int gx,
    unsigned int gy,
    std::vector<unsigned int>& dense_indices) const;

  void simplifyRDPRecursive(
    const costmap_2d::Costmap2D& cm,
    const std::vector<unsigned int>& dense_indices,
    std::size_t start_pos,
    std::size_t end_pos,
    int depth,
    std::vector<std::size_t>& keep_positions) const;

  bool searchSparseAstar(
    const costmap_2d::Costmap2D& cm_copy,
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan,
    const std::string& frame,
    const ros::Time& stamp);

  

  bool isHardCollisionCost(unsigned char c) const;

  bool planSegmentBlockedForReplan(
    const costmap_2d::Costmap2D& cm,
    const geometry_msgs::PoseStamped& a,
    const geometry_msgs::PoseStamped& b,
    int soft_cost_threshold,
    int& max_cost,
    bool& hard_collision,
    double& first_bad_wx,
    double& first_bad_wy) const;

  void applySoftCostThresholdAsObstacle(
    costmap_2d::Costmap2D& cm,
    int cost_threshold) const;
};

}  // namespace pongbot_global_planner