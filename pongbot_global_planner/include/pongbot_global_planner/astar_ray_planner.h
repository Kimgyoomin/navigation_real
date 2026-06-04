#pragma once

#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <string>
#include <vector>

namespace pongbot_global_planner
{

/**
 * @brief Ray-casting macro-action A* global planner for ROS1 noetic
 * 
 * THis planner keeps the ROS1 move_base plugin interface, but replaces
 * cell-by-cell A* expansion with ray-casting macro-actions
 * 
 * Current Node -> cast rays in multiple directions
 *              -> take last valid cell before collision / margin / max distance
 *              -> use the endpoint as successor
 */

class AStarRayPlannerROS : public nav_core::BaseGlobalPlanner
{
public:
    AStarRayPlannerROS();    
    AStarRayPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

    bool makePlan(
        const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal,
        std::vector<geometry_msgs::PoseStamped>& plan
    ) override;
    
private:
    struct RayResult
    {
        bool valid{false};
        unsigned int x{0};
        unsigned int y{0};
        double cost{0.0};
        double length{0.0};
    };

    bool initialized_ = false;
    costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;

    ros::Publisher plan_pub_;
    ros::Publisher plan_time_pub_;

    // Legacy / baseline - compatible parameters
    bool allow_unknown_         = false;
    bool use_octile_heuristic_  = true;
    unsigned char lethal_cost_  = 253;
    double tie_breaker_         = 1.001;
    double cost_weight_alpha_   = 5.0;
    double weight_h_            = 1.05;
    bool print_timing_          = true;

    // Ray-casting macro-action parameters
    int ray_angle_bins_         = 16;
    double max_segment_length_  = 1/5;  // [m]
    double resample_interval_   = 0.25; // [m]
    int line_cost_threshold_    = 180;  // cost -> threshold => ray blocked

    // Helper functions
    inline bool isCellFree(
        const costmap_2d::Costmap2D& cm,
        unsigned int mx,
        unsigned int my
    ) const;

    inline bool isCellSafeForRay(
        const costmap_2d::Costmap2D& cm,
        unsigned int mx,
        unsigned int my
    ) const;

    inline double heuristic(
        int x0,
        int y0,
        int x1,
        int y1,
        double resolution
    ) const;

    static inline double yawBetween(
        double x0,
        double y0,
        double x1,
        double y1
    );

    bool snapToFree(
        const costmap_2d::Costmap2D& cm,
        unsigned& mx,
        unsigned& my
    ) const;

    bool lineSegmentCost(
        const costmap_2d::Costmap2D& cm,
        unsigned int x0,
        unsigned int y0,
        unsigned int x1,
        unsigned int y1,
        double& total_cost
    ) const;

    RayResult castRay(
        const costmap_2d::Costmap2D& cm,
        unsigned int cx,
        unsigned int cy,
        double angle
    ) const;

    bool searchRayAstar(
        const costmap_2d::Costmap2D& cm_copy,
        const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal,
        std::vector<geometry_msgs::PoseStamped>& plan,
        const std::string& frame,
        const ros::Time& stamp
    );


};
 
} // namespace pongbot_global_planner
