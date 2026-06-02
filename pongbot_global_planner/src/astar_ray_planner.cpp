#include "pongbot_global_planner/astar_ray_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"


namespace pongbot_global_planner
{

namespace 
{

struct OpenNode
{
    unsigned int index;
    double f;
};

struct OpenNodeCompare
{
    bool operator()(const OpenNode & a, const OpenNode & b) const
    {
        // priority queue is max-heap by default, so reverse for min-heap
        return a.f > b.f;
    }
};

struct RayResult
{
    bool valid{false};
    unsigned int x{0};
    unsigned int y{0};
    double cost{0.0};
    double length{0.0};
};

}   // namespace

void AstarRayPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    auto node = parent.lock();
    if (!node) {
        throw std::runtime_error("Failed to lock lifecycle node in AstarRayPlanner::configure");
    }

    logger_ = node->get_logger();
    name_ = std::move(name);
    tf_ = std::move(tf);
    costmap_ros_ = std::move(costmap_ros);
    global_frame_ = costmap_ros_->getGlobalFrameID();

    auto declare_if_not_declared = 
        [&](const std::string & param_name, const rclcpp::ParameterValue & default_value)
        {
            if (!node->has_parameter(param_name)) {
                node->declare_parameter(param_name, default_value);
            }
        };

    declare_if_not_declared(name_ + ".ray_angle_bins", rclcpp::ParameterValue(16));
    declare_if_not_declared(name_ + ".max_segment_length", rclcpp::ParameterValue(1.5));
    declare_if_not_declared(name_ + ".resample_interval", rclcpp::ParameterValue(0.25));
    declare_if_not_declared(name_ + ".line_cost_threshold", rclcpp::ParameterValue(180));
    declare_if_not_declared(name_ + ".cost_scale", rclcpp::ParameterValue(5.0));

    node->get_parameter(name_ + ".ray_angle_bins", ray_angle_bins_);
    node->get_parameter(name_ + ".max_segment_length", max_segment_length_);
    node->get_parameter(name_ + ".resample_interval", resample_interval_);
    node->get_parameter(name_ + ".line_cost_threshold", line_cost_threshold_);
    node->get_parameter(name_ + ".cost_scale", cost_scale_);

    ray_angle_bins_ = std::max(8, ray_angle_bins_);  // minimum 8 bins for 45 degree resolution
    max_segment_length_ = std::max(0.1, max_segment_length_);
    resample_interval_ = std::max(0.05, resample_interval_);
    line_cost_threshold_ = std::clamp(line_cost_threshold_, 1, 252);
    cost_scale_ = std::max(0.0, cost_scale_);

    RCLCPP_INFO(
        logger_,
        "Configured planner plugin: %s (global_frame: %s, ray_bins=%d, "
        "max_segment=%.2f m, resample=%.2f m, line_cost_threshold=%d, cost_scale=%.2f)",
        name_.c_str(),
        global_frame_.c_str(),
        ray_angle_bins_,
        max_segment_length_,
        resample_interval_,
        line_cost_threshold_,
        cost_scale_);
}

void AstarRayPlanner::cleanup()
{
    RCLCPP_INFO(logger_, "Cleaning up planner plugin: %s", name_.c_str());
    tf_.reset();
    costmap_ros_.reset();
}

void AstarRayPlanner::activate()
{
    RCLCPP_INFO(logger_, "Activating planner plugin: %s", name_.c_str());
}

void AstarRayPlanner::deactivate()
{
    RCLCPP_INFO(logger_, "Deactivating planner plugin: %s", name_.c_str());
}

nav_msgs::msg::Path AstarRayPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
{
    nav_msgs::msg::Path path;
    path.header.stamp = start.header.stamp;
    path.header.frame_id = global_frame_;

    if (!costmap_ros_) {
        RCLCPP_ERROR(logger_, "costmap_ros_ is null (configure not called?)");
        return path;
    }

    auto * costmap = costmap_ros_->getCostmap();
    if (!costmap) {
        RCLCPP_ERROR(logger_, "Costmap pointer is null.");
        return path;
    }

    // =======================================================================
    // 1) Transform start / goal into global frame
    // =======================================================================
    geometry_msgs::msg::PoseStamped start_g = start;
    geometry_msgs::msg::PoseStamped goal_g = goal;

    try {
        if (tf_ && start.header.frame_id != global_frame_) {
            start_g = tf_->transform(start, global_frame_, tf2::durationFromSec(0.1));
        }
        if (tf_ && goal.header.frame_id != global_frame_) {
            goal_g = tf_->transform(goal, global_frame_, tf2::durationFromSec(0.1));
        }
    }
    catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(logger_, "TF transform failed: %s", ex.what());
        return path;
    }

    // =======================================================================
    // 2) Convert world -> map indices
    // =======================================================================
    unsigned int sx, sy, gx, gy;
    if (!costmap->worldToMap(start_g.pose.position.x, start_g.pose.position.y, sx, sy)) {
        RCLCPP_WARN(
            logger_,
            "Start outside costmap bounds: world=(%.3f, %.3f)",
            start_g.pose.position.x,
            start_g.pose.position.y);
        return path;
    }

    if (!costmap->worldToMap(goal_g.pose.position.x, goal_g.pose.position.y, gx, gy)) {
        RCLCPP_WARN(
            logger_,
            "Goal outside costmap bounds: world=(%.3f, %.3f)",
            goal_g.pose.position.x,
            goal_g.pose.position.y);
        return path;
    }

    const unsigned int size_x   = costmap->getSizeInCellsX();
    const unsigned int size_y   = costmap->getSizeInCellsY();
    const unsigned int total    = size_x * size_y;
    const double resolution     = costmap->getResolution(); 

    auto toIndex = [size_x](unsigned int x, unsigned int y) -> unsigned int {
        return y * size_x + x;
    };

    auto toXY = [size_x](unsigned int index, unsigned int & x, unsigned int & y) {
        x = index % size_x;
        y = index / size_x;
    };

    auto sameCell = [](unsigned int ax, unsigned int ay, unsigned int bx, unsigned int by) -> bool {
        return ax == bx && ay == by;
    };

    // =======================================================================
    // 3) Traversability rules
    //
    // isTraversable:
    //   - used for start/goal and graph node validity
    //   - inflated cells are allowed, but penalized
    // 
    // isSafeForRay:
    //   - used for line segment traversal
    //.  - stricter than isTraversable
    //.  - high inflated-cost cells are treated as margin boundary
    // =======================================================================

    auto isTraversable = [costmap](unsigned int x, unsigned int y) -> bool {
        const unsigned char c = costmap->getCost(x, y);

        if (c == nav2_costmap_2d::NO_INFORMATION) {
            return false;
        }

        if (c >= nav2_costmap_2d::LETHAL_OBSTACLE) {
            return false;
        }

        return true;
    };

    auto isSafeForRay = [this, costmap](unsigned int x, unsigned int y) -> bool {
        const unsigned char c = costmap->getCost(x, y);

        if (c == nav2_costmap_2d::NO_INFORMATION) {
            return false;
        }

        if (c >= nav2_costmap_2d::LETHAL_OBSTACLE) {
            return false;
        }

        if (static_cast<int>(c) > line_cost_threshold_) {
            return false;
        }

        return true;
    };

    const unsigned char start_cost = costmap->getCost(sx, sy);
    const unsigned char goal_cost = costmap->getCost(gx, gy);

    RCLCPP_INFO(
        logger_,
        "Ray planning request: start_world=(%.3f, %.3f) goal_world=(%.3f, %.3f)",
        start_g.pose.position.x, start_g.pose.position.y,
        goal_g.pose.position.x, goal_g.pose.position.y);

    RCLCPP_INFO(
        logger_,
        "Ray planning request: start_map=(%u, %u, cost=%u) goal_map=(%u, %u, cost=%u)",
        sx, sy, static_cast<unsigned int>(start_cost),
        gx, gy, static_cast<unsigned int>(goal_cost));

    if (!isTraversable(sx, sy)) {
        RCLCPP_WARN(
            logger_,
            "Start not traversable: map=(%u, %u), cost=%u",
            sx, sy, static_cast<unsigned int>(start_cost));
        return path;
    }

    if (!isTraversable(gx, gy)) {
        RCLCPP_WARN(
            logger_,
            "Goal not traversable: map=(%u, %u), cost=%u",
            gx, gy, static_cast<unsigned int>(goal_cost));
        return path;
    }

    const unsigned int start_i = toIndex(sx, sy);
    const unsigned int goal_i = toIndex(gx, gy);

    if (start_i == goal_i) {
        geometry_msgs::msg::PoseStamped ps = start_g;
        ps.header.frame_id = global_frame_;
        ps.header.stamp = path.header.stamp;
        path.poses.push_back(ps);

        geometry_msgs::msg::PoseStamped pg = goal_g;
        pg.header.frame_id = global_frame_;
        pg.header.stamp = path.header.stamp;
        path.poses.push_back(pg);

        return path;
    }

    // =======================================================================
    // 4) Heuristic in metric units
    // =======================================================================
    auto heuristic = [gx, gy, resolution](unsigned int x, unsigned int y) -> double {
        const double dx = std::abs(static_cast<int>(x) - static_cast<int>(gx));
        const double dy = std::abs(static_cast<int>(y) - static_cast<int>(gy));
        const double D = resolution;  // cost per cell for heuristic (can be tuned)
        const double D2 = std::sqrt(2.0) * resolution;  // diagonal cost for heuristic
        return D * (dx + dy) + (D2 - 2.0 * D) * std::min(dx, dy);
    };

    // =======================================================================
    // 5) Collision / margin checked line segment cost
    //
    // THis samples the segment in world coordinates
    // Each sampled cell must be safe for ray traversal
    // COst penalty is integrated along the segment
    // =======================================================================
    auto lineSegmentCost = 
        [&](unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, double & total_cost)
        -> bool
        {
            double wx0, wy0, wx1, wy1;
            costmap->mapToWorld(x0, y0, wx0, wy0);
            costmap->mapToWorld(x1, y1, wx1, wy1);

            const double dx = wx1 - wx0;
            const double dy = wy1 - wy0;
            const double dist = std::hypot(dx, dy);

            if (dist < 1e-9) {
                total_cost = 0.0;
                return true;
            }

            const double sample_step = std::max(0.5 * resolution, 0.01);
            const int steps = std::max(1, static_cast<int>(std::ceil(dist / sample_step)));
            const double ds = dist / static_cast<double>(steps);

            total_cost = dist;

            for (int s = 0; s <= steps; ++s) {
                const double t = static_cast<double>(s) / static_cast<double>(steps);
                const double wx = wx0 + t * dx;
                const double wy = wy0 + t * dy;

                unsigned int mx, my;
                if (!costmap->worldToMap(wx, wy, mx, my)) {
                    return false;
                }

                if (s == 0) {
                    if (!isTraversable(mx, my)) {
                        return false;
                    }
                }
                else {
                    if (!isSafeForRay(mx, my)) {
                        return false;
                    }
                }

                if (s > 0)
                {
                    const unsigned char c = costmap->getCost(mx, my);
                    total_cost += ds * cost_scale_ * (static_cast<double>(c) / 255.0);
                }
            }

            return true;
        };

        // =======================================================================
        // 6) Ray casting macro-action
        // =======================================================================
        auto castRay =
            [&](unsigned int cx, unsigned int cy, double angle) -> RayResult
            {
                RayResult result;
                
                double cwx, cwy;
                costmap->mapToWorld(cx, cy, cwx, cwy);

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
                    if (!costmap->worldToMap(wx, wy, mx, my)) {
                        break;
                    }

                    // Avoid evaluating the same cell repeatedly when rat_step < resolution
                    if (sameCell(mx, my, last_valid_x, last_valid_y)) {
                        continue;
                    }

                    if (!isSafeForRay(mx, my)) {
                        break;
                    }

                    double seg_cost = 0.0;
                    if (!lineSegmentCost(cx, cy, mx, my, seg_cost)) {
                        break;
                    }

                    last_valid_x = mx;
                    last_valid_y = my;
                    result.cost = seg_cost;
                    result.length = dist;
                    moved = true;
                }

                if (!moved) {
                    return result;  // no valid ray
                }

                result.valid = true;
                result.x = last_valid_x;
                result.y = last_valid_y;
                return result;
            };

        std::vector<double> g_score(total, std::numeric_limits<double>::infinity());
        std::vector<int> came_from(total, -1);
        std::vector<bool> closed(total, false);

        std::priority_queue<OpenNode, std::vector<OpenNode>, OpenNodeCompare> open;
        g_score[start_i] = 0.0;
        open.push(OpenNode{start_i, heuristic(sx, sy)});

        bool found = false;
        std::size_t expanded_nodes = 0;
        constexpr double TWO_PI = 6.28318530717958647692;

        // --------------------------------------------------------------------------
        // 7) Ray-casting A* search
        // -------------------------------------------------------------------------- 
        while (!open.empty()) {
            const auto current = open.top();
            open.pop();

            const unsigned int ci = current.index;
            if (closed[ci]) {
                continue;
            }

            closed[ci] = true;
            ++expanded_nodes;

            if (ci == goal_i) {
                found = true;
                break;
            }

            unsigned int cx, cy;
            toXY(ci, cx, cy);

            // 7.1) Direct goal connection if line-of-sight is valid
            double goal_segment_cost = 0.0;
            if (lineSegmentCost(cx, cy, gx, gy, goal_segment_cost)) {
                const double tentative_g = g_score[ci] + goal_segment_cost;

                if (tentative_g < g_score[goal_i]) {
                    g_score[goal_i] = tentative_g;
                    came_from[goal_i] = static_cast<int>(ci);
                    open.push(OpenNode{goal_i, tentative_g});
                }
            }

            // 7.2) Ray macro-action expansion
            for (int k = 0; k < ray_angle_bins_; ++k) {
                const double angle = TWO_PI * static_cast<double>(k) / 
                    static_cast<double>(ray_angle_bins_);

                const RayResult ray = castRay(cx, cy, angle);
                if (!ray.valid) {
                    continue;
                }

                const unsigned int nx = ray.x;
                const unsigned int ny = ray.y;
                const unsigned int ni = toIndex(nx, ny);

                if (ni == ci || closed[ni]) {
                    continue;
                }

                const double tentative_g = g_score[ci] + ray.cost;

                if (tentative_g < g_score[ni]) {
                    g_score[ni] = tentative_g;
                    came_from[ni] = static_cast<int>(ci);
                    const double f = tentative_g + heuristic(nx, ny);
                    open.push(OpenNode{ni, f});
                }
            }
        }

        if (!found) {
            RCLCPP_WARN(

                logger_,
                "Ray A* failed to find a path. expanded_nodes=%zu start=(%u,%u,cost=%u) "
                "goal=(%u,%u,cost=%u)",
                expanded_nodes,
                sx, sy, static_cast<unsigned int>(start_cost),
                gx, gy, static_cast<unsigned int>(goal_cost));
                return path;
        }

        // --------------------------------------------------------------------------
        // 8) Reconstruct sparse segment path indices
        // --------------------------------------------------------------------------
        std::vector<unsigned int> indices;
        indices.reserve(128);

        unsigned int trace = goal_i;
        indices.push_back(trace);

        while (trace != start_i) {
            const int prev = came_from[trace];
            if (prev < 0) {
                RCLCPP_WARN(logger_, "Ray A* reconstruction failed: broken came_from");
                return nav_msgs::msg::Path{};
            }

            trace = static_cast<unsigned int>(prev);
            indices.push_back(trace);
        }

        std::reverse(indices.begin(), indices.end());

        // --------------------------------------------------------------------------
        // 9) Convert sparse path into controller-friendly resampled path
        // --------------------------------------------------------------------------
        path.poses.clear();
        path.poses.reserve(indices.size() * 4 + 2);
        
        geometry_msgs::msg::PoseStamped start_pose = start_g;
        start_pose.header.frame_id = global_frame_;
        start_pose.header.stamp = path.header.stamp;
        path.poses.push_back(start_pose);

        auto makePose = [&](double wx, double wy) -> geometry_msgs::msg::PoseStamped {
            geometry_msgs::msg::PoseStamped p;
            p.header = path.header;
            p.pose.position.x = wx;
            p.pose.position.y = wy;
            p.pose.position.z = 0.0;
            return p;
        };

        auto appendSegmentToPath =
            [&](const geometry_msgs::msg::PoseStamped & target)
            {
                if (path.poses.empty()) {
                    path.poses.push_back(target);
                    return;
                }

                const auto & prev = path.poses.back().pose.position;
                const auto & next = target.pose.position;

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

                    geometry_msgs::msg::PoseStamped p;
                    p.header = path.header;
                    p.pose.position.x = prev.x + t * dx;
                    p.pose.position.y = prev.y + t * dy;
                    p.pose.position.z = 0.0;
                    path.poses.push_back(p);
                }

                path.poses.push_back(target);
            };

        for (std::size_t i = 1; i < indices.size(); ++i) {
            geometry_msgs::msg::PoseStamped target;

            if (i + 1 == indices.size()) {
                // Keep exact user goal.
                target = goal_g;
                target.header.frame_id = global_frame_;
                target.header.stamp = path.header.stamp;
            } else {
                unsigned int mx, my;
                toXY(indices[i], mx, my);

                double wx, wy;
                costmap->mapToWorld(mx, my, wx, wy);
                target = makePose(wx, wy);
            }

            appendSegmentToPath(target);
        }

        // --------------------------------------------------------------------------
        // 10) Heading orientation
        // --------------------------------------------------------------------------
        if (path.poses.size() >= 2) {
            for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
                const auto & a = path.poses[i].pose.position;
                const auto & b = path.poses[i + 1].pose.position;

                const double dx = b.x - a.x;
                const double dy = b.y - a.y;

                if (std::hypot(dx, dy) < 1e-6) {
                    continue;
                }

                const double yaw = std::atan2(dy, dx);

                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, yaw);
                path.poses[i].pose.orientation = tf2::toMsg(q);
            }

            path.poses.back().pose.orientation =
                path.poses[path.poses.size() - 2].pose.orientation;
        }

        RCLCPP_INFO(
            logger_,
            "Ray A* path found. segment_nodes=%zu final_points=%zu expanded_nodes=%zu "
            "ray_bins=%d max_segment=%.2f resample=%.2f line_cost_threshold=%d",
            indices.size(),
            path.poses.size(),
            expanded_nodes,
            ray_angle_bins_,
            max_segment_length_,
            resample_interval_,
            line_cost_threshold_);

        return path;
}

}   // namespace pongbot_global_planner

PLUGINLIB_EXPORT_CLASS(pongbot_global_planner::AstarRayPlanner, nav2_core::GlobalPlanner)

