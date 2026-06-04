#pragma once
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/time.h>


namespace pongbot_global_planner 
{

/**
 * @brief Global Planner plugin (A* with 2D costmap)
 * Class which move_base load to pluginlib
 */
// class which move_base will load to pluginlib ( make BaseGlobalPlanner )
class AStarPlannerROS : public nav_core::BaseGlobalPlanner
{
public:
    AStarPlannerROS();
    AStarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    // Lifecycle for plugin
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;


    // After Rviz 2D Nav Goal, API will make path which move_base can use(call)
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
        // State
        bool initialized_ = false;
        costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;

        // Visulization  / Debugging
        ros::Publisher plan_pub_;           // nav_msgs/Path (Rviz Path)
        ros::Publisher plan_time_pub_;      // std_msgs/Float64 [ms] (Planning Time)


        // Parameters (To ROS param) -> Would be used for hybrid A* later
        double step_size_;
        double yaw_disc_;
        double goal_tol_;


        // 2D A* parameter
        bool allow_unknown_         = false;        // NO_INFORMATION -> would u allow or not?
        bool use_octile_heuristic_  = true;         //
        unsigned char lethal_cost_  = 253;          // >= 253(INSCRIBED) cell is banned
        double tie_breaker_         = 1.001;        // f = g + h * tie_breaker_

        // cost gain / heuristic gain
        double  cost_weight_alpha_  = 6.0;          // far from wall
        double  weight_h_           = 1.05;         // 1.0 ~ 1.2 (A* gain)


        // ==== Helper ====
        inline bool isCellFree(const costmap_2d::Costmap2D& cm,
                                unsigned int mx, unsigned int my) const;

        inline double heuristic(int x0, int y0, int x1, int y1) const;

        static inline double yawBetween(double x0, double y0, double x1, double y1);

        bool searchGridAstar(const costmap_2d::Costmap2D& cm_copy,
                             const geometry_msgs::PoseStamped& start,
                             const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan,
                             const std::string& frame,
                             const ros::Time& stamp);


        // start/ goal snap
        bool snapToFree(const costmap_2d::Costmap2D& cm, unsigned& mx, unsigned& my) const;
        
        // For time calculation
        bool print_timing_          = true;
        /// TODO: module handlers for motionprimitives / heuristic / collisionchecker
};

}   // namespace pongbot_global_planner