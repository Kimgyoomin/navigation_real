#include "pongbot_global_planner/astar_planner.h"

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <tf/transform_datatypes.h>

#include <queue>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>

#include <unordered_map>
#include <chrono>
#include <iomanip>
#include <sstream>

// PLUGIN_EXPORT_CLASS(pongbot_global_planner::AStarPlannerROS, nav_core::BaseGlobalPlanner)

namespace pongbot_global_planner
{

AStarPlannerROS::AStarPlannerROS() {}

AStarPlannerROS::AStarPlannerROS(std::string name, costmap_2d::Costmap2DROS* cm_ros)
{
    initialize(name, cm_ros);
}


void AStarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* cm_ros)
{
    if (initialized_) return;
    costmap_ros_ = cm_ros;


    // move_base namespace, Under private ns: ~/<PluginName>/*
    ros::NodeHandle pnh("~/" + name);


    // Topic (~/<PluginName>/plan, /planning_time_ms)
    plan_pub_       = pnh.advertise<nav_msgs::Path>("plan", 1, true);
    plan_time_pub_  = pnh.advertise<std_msgs::Float64>("planning_time_ms" , 1, true);



    // Load Params ( NOne -> default value )
    pnh.param("step_size",  step_size_, 0.2);
    pnh.param("yaw_disc",   yaw_disc_,  5.0 * M_PI/180.0);
    pnh.param("goal_tol",   goal_tol_,  0.07);


    // 2D A* parameter
    pnh.param("allow_unknown",          allow_unknown_,         false);
    pnh.param("use_octile_heuristic",   use_octile_heuristic_,  true);
    int lethal_cost_tmp     =   static_cast<int>(lethal_cost_);
    pnh.param("lethal_cost",            lethal_cost_tmp,        253);
    lethal_cost_            = static_cast<unsigned char>(lethal_cost_tmp);
    pnh.param("tie_breaker",            tie_breaker_,           1.001);
    pnh.param("cost_weight_alpha",      cost_weight_alpha_,     6.0);
    pnh.param("weight_h",               weight_h_,              1.05);

    // For time calculation
    pnh.param("print_timing",           print_timing_,          true);
    initialized_ = true;
    ROS_INFO_STREAM("pongbot_global_planner::AStarPlannerROS initialized under ~/" << name
                    << " allow_unknown ="     << allow_unknown_
                    << " octile ="           << use_octile_heuristic_
                    << " lethal_cost ="      << (int)lethal_cost_
                    << " tie ="              << tie_breaker_
                    << " alpha = "           << cost_weight_alpha_
                    << " w_h ="              << weight_h_);
}


inline bool AStarPlannerROS::isCellFree(const costmap_2d::Costmap2D& cm,
                                        unsigned int mx, unsigned int my) const {
    unsigned char c = cm.getCost(mx, my);
    if (c == costmap_2d::NO_INFORMATION) return allow_unknown_;
    
    // Over INSCRIBED(253) -> set as occupied
    return (c < lethal_cost_);
}



inline double AStarPlannerROS::heuristic(int x0, int y0, int x1, int y1) const {
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    if (use_octile_heuristic_) {

        // Octile: (dx + dy) + (sqrt(2) - 2)*min(dx, dy)
        const double SQRT2 = std::sqrt(2.0);
        return (dx + dy) + (SQRT2 - 2.0) * std::min(dx, dy);
    }
    else
    {
        // Euclidean
        return std::hypot(dx, dy);
    }
}


inline double AStarPlannerROS::yawBetween(double x0, double y0, double x1, double y1)
{
    return std::atan2(y1 - y0, x1 - x0);
}

bool AStarPlannerROS::snapToFree(const costmap_2d::Costmap2D& cm,
                                 unsigned& mx, unsigned& my) const
{
    const int W = (int)cm.getSizeInCellsX();
    const int H = (int)cm.getSizeInCellsY();

    auto isFreeCell = [&](int x, int y) {
        if (x<0 || y<0 || x>=W || y>=H) return false;
        unsigned char c = cm.getCost((unsigned)x, (unsigned)y);
        
        if(c == costmap_2d::NO_INFORMATION) return allow_unknown_;
        return (c < lethal_cost_);      // 0 ~ 252 : pass , over 253 : ban
    };


    if (isFreeCell((int)mx, (int)my)) return true;


    static const int dx[4] = {1, -1, 0, 0};
    static const int dy[4] = {0, 0, 1, -1};

    
    std::queue<std::pair<int, int>> q; 
    q.push(std::make_pair((int)mx, (int)my));

    std::vector<char> vis(W*H, 0);
    auto idx = [&](int x, int y){ return y * W + x; };
    vis[idx((int)mx, (int)my)] = 1;

    while(!q.empty()) {
        std::pair<int, int> cur = q.front(); q.pop();
        int x = cur.first;
        int y = cur.second;

        if (isFreeCell(x, y)) { mx = (unsigned)x; my = (unsigned)y; return true; }
        
        for(int i = 0; i < 4; i++)
        {
            int xx = x + dx[i], yy = y + dy[i];
            if (xx >=0 && yy >= 0 && xx < W && yy < H) {
                int ii = idx(xx, yy);
                if(!vis[ii]) 
                { 
                    vis[ii] = 1; 
                    q.push(std::make_pair(xx, yy)); 
                }
            }   
        }
    }
    return false;       // If not found, return false for low probability

}



bool AStarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan)
{
    plan.clear();
    if (!initialized_) {
        ROS_ERROR("[AStarPlannerROS] makePlan called but not initialized");    
        return false;
    }
    if (start.header.frame_id != goal.header.frame_id){
        ROS_ERROR_STREAM("[AStarPlannerROS] frame mismatch : " << start.header.frame_id
                            << " vs " << goal.header.frame_id);
        return false;                  
    }
    
    // set frame variable
    // const std::string frame = start.header.frame_id;
    // changed to costmap's global frame (25.09.16)
    const std::string frame = costmap_ros_->getGlobalFrameID();


    // Snapshot costmap ( update thread -> mutex, lock for a second and copy )
    costmap_2d::Costmap2D local_copy;
    {
        costmap_2d::Costmap2D* cm = costmap_ros_->getCostmap();
        std::lock_guard<costmap_2d::Costmap2D::mutex_t> lk(*(cm->getMutex()));
        local_copy = *cm;       // If needed, use copyCostmapWindow(...) and copy only window
    }

    // 여기서 stamp를 만든다
    // const ros::Time stamp = ros::Time::now();
    const ros::Time stamp = ros::Time(0);

    // Start to check time (Walltime)
    // const ros::WallTime t0 = ros::WallTime::now();       // To change WallTime to chrono
    const auto t0 = std::chrono::steady_clock::now();

    // 2D A* call
    // bool ok = searchGridAstar(local_copy, start, goal, plan, frame);
    // stamp 넘겨주기
    bool ok = searchGridAstar(local_copy, start, goal, plan, frame, stamp);

    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0
    ).count();

    // Publish time
    std_msgs::Float64 dt;
    // dt.data = (ros::WallTime::now() - t0).toSec() * 1000.0;      // To change Walltime to chrono
    dt.data = ms;
    plan_time_pub_.publish(dt);

    if (print_timing_) {
        std::ostringstream oss;
        oss << "\033[1;36m"                    // Cyan
            << "[A*] planning_time = "
            << std::fixed << std::setprecision(3) << ms << " ms"
            << "  (poses=" << plan.size() << ")"
            << "\033[0m";
        ROS_INFO_STREAM_NAMED("pongbot_astar", oss.str());
    }

    // ============================================================================
    // Hybrid A* (2D : x, y, yaw ) Main LOGIC
    // TODO: searchHybridAstar(local_copy, start, goal,
    //                         step_size_, yaw_disc_, goal_tol_, plan);
    // ============================================================================
    // For now just simple , (start, goal) two points and check RViz pipe
    // plan.push_back(start);
    // plan.push_back(goal);

    // Publish Path / Time
    if (ok) {
        // plan 내부 stamp/frame도 통일
        for (auto &p : plan) {
            p.header.stamp      = stamp;
            p.header.frame_id   = frame;
        }
        nav_msgs::Path path_msg;
        // path_msg.header.stamp       = ros::Time::now();
        // const ros::Time stamp       = ros::Time::now();

        // path_msg.header.stamp       = ros::Time(0);
        path_msg.header.stamp       = stamp;
        path_msg.header.frame_id    = frame;            // Usually "map"
        path_msg.poses              = plan;
        plan_pub_.publish(path_msg);
    } else
    {
        ROS_WARN("[AStarPlannerROS] no path found");
    }
    return ok;
}


bool AStarPlannerROS::searchGridAstar(const costmap_2d::Costmap2D& cm_copy,
                                      const geometry_msgs::PoseStamped& start,
                                      const geometry_msgs::PoseStamped& goal,
                                      std::vector<geometry_msgs::PoseStamped>& plan,
                                      const std::string& frame,
                                      const ros::Time& stamp) {
                                    //   const std::string& frame) {
    const unsigned int W = cm_copy.getSizeInCellsX();
    const unsigned int H = cm_copy.getSizeInCellsY();
    if (W == 0 || H == 0)
    {
        ROS_ERROR("[AStarPlannerROS] empty costmap");
        return false;
    }

    // world -> map
    unsigned int sx, sy, gx, gy;
    if (!cm_copy.worldToMap(start.pose.position.x, start.pose.position.y, sx, sy)){
        ROS_WARN("[AStarPlannerROS] start outside map");
        return false;
    }

    if (!cm_copy.worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
        ROS_WARN("[AStarPlannerROS] goal outside map");
        return false;
    }

    if (!isCellFree(cm_copy, sx, sy))
    {
        if (!snapToFree(cm_copy, sx, sy)) {
            ROS_WARN("[AStarPlannerROS] start in obstacle and snap failed");
            return false;
        }
    }

    if (!isCellFree(cm_copy, gx, gy))
    {
        if (!snapToFree(cm_copy, gx, gy)) {
            ROS_WARN("[AStarPlannerROS] goal in obstacle and snap failed");
            return false;
        }
    }

    // if (!isCellFree(cm_copy, sx, sy) || !isCellFree(cm_copy, gx, gy)) {
    //     ROS_WARN("[AStarPlannerROS] start or goal is in obstacle");
    //     return false;
    // }

    auto toIndex = [W](unsigned int x, unsigned int y) -> unsigned int { return y * W + x; };
    const unsigned int sidx = toIndex(sx, sy);
    const unsigned int gidx = toIndex(gx, gy);

    struct Node {
        unsigned int idx;
        double f, g;
        bool operator<(const Node& o) const { return f > o.f; } // min - heap effect
    };

    std::priority_queue<Node> open;
    std::vector<double> gscore(W * H, std::numeric_limits<double>::infinity());
    std::vector<int> parent(W * H, -1);

    gscore[sidx] = 0.0;
    {
        double h = heuristic((int)sx, (int)sy, (int)gx, (int)gy);
        open.push({sidx, /*f=*/weight_h_ * h * tie_breaker_, /*g=*/0.0});
    }


    const int dx8[8]        = {+1, -1,  0, 0, +1, +1, -1, -1};
    const int dy8[8]        = { 0,  0, +1, -1, +1, -1, +1, -1};
    const double c8[8]      = {1, 1, 1, 1, std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0)};

    bool found = false;



    while (!open.empty())
    {
        Node cur = open.top();
        open.pop();
        if (cur.g != gscore[cur.idx]) continue;     // lazy skip
        if (cur.idx == gidx) { found = true; break; }

        unsigned int cx = cur.idx % W;
        unsigned int cy = cur.idx / W;


        // Look for 8 direction
        for (int k = 0; k < 8; ++k)
        {
            int nx = (int)cx + dx8[k];
            int ny = (int)cy + dy8[k];
            if (nx < 0 || ny < 0 || nx >= (int)W || ny >= (int)H) continue;


            unsigned int nidx = toIndex((unsigned int)nx, (unsigned int)ny);
            if (!isCellFree(cm_copy, (unsigned int)nx, (unsigned int)ny)) continue;
            
            unsigned char c = cm_copy.getCost((unsigned)nx, (unsigned)ny);  // Use 0 ~ 252
            double norm = std::min(1.0, (double)c / 252.0);                 // Normalized by 0 ~ 1
            double step = c8[k];                                            // grid     
            double w    = 1.0 + cost_weight_alpha_ * norm;                  // alpha = 4 ~ 8?
            double tentative = cur.g + step * w;

            if (tentative < gscore[nidx])
            {
                gscore[nidx] = tentative;
                parent[nidx] = (int)cur.idx;
                double h = heuristic(nx, ny, (int)gx, (int)gy);
                open.push({nidx, tentative + weight_h_ * h * tie_breaker_, tentative});
            }
        }
    }

    if (!found) return false;


    // fallback
    std::vector<unsigned int> rev;
    for (int idx = (int)gidx; idx >= 0; idx = parent[(unsigned int)idx])
    {
        rev.push_back((unsigned int)idx);
        if ((unsigned int)idx == sidx) break;
    }

    std::reverse(rev.begin(), rev.end());

    // map -> world & PoseStamped struct
    plan.reserve(rev.size());
    for (size_t i =0; i < rev.size(); ++i)
    {
        unsigned int idx = rev[i];
        unsigned int mx = idx % W;
        unsigned int my = idx / W;
        double wx, wy;
        cm_copy.mapToWorld(mx, my, wx, wy);

        geometry_msgs::PoseStamped p;
        // p.header.stamp = ros::Time::now();
        // p.header.stamp = ros::Time(0);
        p.header.stamp    = stamp;
        p.header.frame_id = frame;
        p.pose.position.x = wx;
        p.pose.position.y = wy;
        p.pose.position.z = 0.0;

        if (i + 1 < rev.size())
        {
            unsigned int nidx = rev[i + 1];
            unsigned int nmx = nidx % W;
            unsigned int nmy = nidx / W;
            double nwx, nwy;
            cm_copy.mapToWorld(nmx, nmy, nwx, nwy);
            double yaw = yawBetween(wx, wy, nwx, nwy);
            p.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }
        else
        {
            p.pose.orientation = goal.pose.orientation;     // At the end, keep goal yaw
        }
        plan.push_back(p);
    }
    return true;

}


} // namespace pongbot_global_planner


// set plugin at "pluginlib" (This macro will send symbol to .so)
PLUGINLIB_EXPORT_CLASS(pongbot_global_planner::AStarPlannerROS, nav_core::BaseGlobalPlanner)