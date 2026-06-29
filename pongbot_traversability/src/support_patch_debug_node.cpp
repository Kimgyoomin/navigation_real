#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>

struct LimbEnvelope
{
    std::string name;
    double x_min{0.0};
    double x_max{0.0};
    double y_min{0.0};
    double y_max{0.0};
};

struct Patch
{
    std::string limb_name;
    double cx{0.0};
    double cy{0.0};
    double yaw{0.0};
    double length{0.0};
    double width{0.0};
};

struct PatchRisk
{
    bool valid{false};

    int expected_cells{0};
    int valid_cells{0};

    double unknown_ratio{1.0};
    double height_range{0.0};
    double slope_deg{0.0};
    double roughness{0.0};
    double mean_variance{0.0};

    double total_cost{1.0};

    Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
    Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
};
template <typename T>
T clampValue(const T value, const T low, const T high)
{
    return std::max(low, std::min(value, high));
}

class SupportPatchDebugNode
{
public:
    SupportPatchDebugNode()
        : nh_(),
          pnh_("~"),
          tf_listener_(tf_buffer_)
    {
        loadParams();

        marker_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);

        grid_sub_ =
            nh_.subscribe(
                terrain_topic_,
                1,
                &SupportPatchDebugNode::gridCallback,
                this);

        ROS_INFO("[support_patch_debug] topic=%s frame=%s body=%s",
                 terrain_topic_.c_str(),
                 terrain_frame_.c_str(),
                 body_frame_.c_str());
    }

private:
    void loadParams()
    {
        pnh_.param<std::string>("terrain_topic", terrain_topic_, "/legged_terrain/elevation_grid");
        pnh_.param<std::string>("terrain_frame", terrain_frame_, "odom");
        pnh_.param<std::string>("body_frame", body_frame_, "BODY");

        pnh_.param<std::string>("height_layer", height_layer_, "filter_map");
        pnh_.param<std::string>("valid_layer", valid_layer_, "is_valid");
        pnh_.param<std::string>("variance_layer", variance_layer_, "variance");

        pnh_.param<double>("candidate_stride_x", candidate_stride_x_, 0.05);
        pnh_.param<double>("candidate_stride_y", candidate_stride_y_, 0.05);

        pnh_.param<double>("patch_length", patch_length_, 0.12);
        pnh_.param<double>("patch_width", patch_width_, 0.10);
        pnh_.param<double>("analysis_length", analysis_length_, 0.24);
        pnh_.param<double>("analysis_width", analysis_width_, 0.18);

        pnh_.param<int>("risk/min_valid_cells", min_valid_cells_, 8);
        pnh_.param<double>("risk/max_unknown_ratio", max_unknown_ratio_, 0.60);
        pnh_.param<double>("risk/max_height_range", max_height_range_, 0.08);
        pnh_.param<double>("risk/max_slope_deg", max_slope_deg_, 20.0);
        pnh_.param<double>("risk/max_roughness", max_roughness_, 0.03);
        pnh_.param<double>("risk/max_variance", max_variance_, 0.10);

        pnh_.param<double>("weights/unknown", w_unknown_, 1.0);
        pnh_.param<double>("weights/step", w_step_, 1.0);
        pnh_.param<double>("weights/slope", w_slope_, 1.0);
        pnh_.param<double>("weights/roughness", w_roughness_, 0.7);
        pnh_.param<double>("weights/variance", w_variance_, 0.4);

        pnh_.param<std::string>("visualization/marker_topic",
                                marker_topic_,
                                "/legged_terrain/support_patch_markers");

        loadLimb("FL", {0.18, 0.35, 0.10, 0.22});
        loadLimb("FR", {0.18, 0.35, -0.22, -0.10});
        loadLimb("RL", {-0.35, -0.18, 0.10, 0.22});
        loadLimb("RR", {-0.35, -0.18, -0.22, -0.10});
    }

    void loadLimb(
        const std::string& name,
        const std::vector<double>& defaults)
    {
        std::vector<double> v;
        if (!pnh_.getParam("limbs/" + name, v) || v.size() != 4) {
            v = defaults;
            ROS_WARN("[support_patch_debug] Using default envelope for %s", name.c_str());
        }

        LimbEnvelope limb;
        limb.name = name;
        limb.x_min = v[0];
        limb.x_max = v[1];
        limb.y_min = v[2];
        limb.y_max = v[3];

        limbs_.push_back(limb);
    }

    void gridCallback(const grid_map_msgs::GridMap::ConstPtr& msg)
    {
        grid_map::GridMap map;
        grid_map::GridMapRosConverter::fromMessage(*msg, map);

        if (!checkLayers(map)) {
            return;
        }

        geometry_msgs::TransformStamped tf_msg;
        try {
            tf_msg =
                tf_buffer_.lookupTransform(
                    terrain_frame_,
                    body_frame_,
                    ros::Time(0),
                    ros::Duration(0.05));
        } catch (const tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(
                1.0,
                "[support_patch_debug] TF lookup failed %s <- %s: %s",
                terrain_frame_.c_str(),
                body_frame_.c_str(),
                ex.what());
            return;
        }

        const double body_x = tf_msg.transform.translation.x;
        const double body_y = tf_msg.transform.translation.y;
        const double body_yaw = tf2::getYaw(tf_msg.transform.rotation);

        visualization_msgs::MarkerArray markers;
        int marker_id = 0;

        clearOldMarkers(markers);

        std::vector<PatchRisk> best_risks;

        for (const auto& limb : limbs_) {
            addEnvelopeMarker(
                markers,
                marker_id++,
                limb,
                body_x,
                body_y,
                body_yaw);

            Patch best_patch;
            PatchRisk best_risk;
            evaluateLimb(
                map,
                limb,
                body_x,
                body_y,
                body_yaw,
                best_patch,
                best_risk);

            best_risks.push_back(best_risk);

            addPatchMarker(
                markers,
                marker_id++,
                best_patch,
                best_risk);

            addNormalMarker(
                markers,
                marker_id++,
                best_patch,
                best_risk);
        }

        marker_pub_.publish(markers);
    }

    bool checkLayers(const grid_map::GridMap& map) const
    {
        if (!map.exists(height_layer_)) {
            ROS_WARN_THROTTLE(
                1.0,
                "[support_patch_debug] Missing height layer: %s",
                height_layer_.c_str());
            return false;
        }

        if (!map.exists(valid_layer_)) {
            ROS_WARN_THROTTLE(
                1.0,
                "[support_patch_debug] Missing valid layer: %s",
                valid_layer_.c_str());
            return false;
        }

        return true;
    }

    void evaluateLimb(
        const grid_map::GridMap& map,
        const LimbEnvelope& limb,
        const double body_x,
        const double body_y,
        const double body_yaw,
        Patch& best_patch,
        PatchRisk& best_risk) const
    {
        best_risk.total_cost = std::numeric_limits<double>::infinity();
        best_risk.valid = false;

        for (double bx = limb.x_min; bx <= limb.x_max + 1e-6; bx += candidate_stride_x_) {
            for (double by = limb.y_min; by <= limb.y_max + 1e-6; by += candidate_stride_y_) {
                const std::pair<double, double> p =
                    transformBodyPointToTerrain(body_x, body_y, body_yaw, bx, by);

                const double cx = p.first;
                const double cy = p.second;

                Patch patch;
                patch.limb_name = limb.name;
                patch.cx = cx;
                patch.cy = cy;
                patch.yaw = body_yaw;
                patch.length = patch_length_;
                patch.width = patch_width_;

                Patch analysis_patch = patch;
                analysis_patch.length = analysis_length_;
                analysis_patch.width = analysis_width_;

                PatchRisk risk = evaluatePatch(map, analysis_patch);

                if (risk.total_cost < best_risk.total_cost) {
                    best_patch = patch;
                    best_risk = risk;
                }
            }
        }

        if (!std::isfinite(best_risk.total_cost)) {
            best_risk.total_cost = 1.0;
            best_risk.valid = false;

            best_patch.limb_name = limb.name;
            const auto [cx, cy] =
                transformBodyPointToTerrain(
                    body_x,
                    body_y,
                    body_yaw,
                    0.5 * (limb.x_min + limb.x_max),
                    0.5 * (limb.y_min + limb.y_max));
            best_patch.cx = cx;
            best_patch.cy = cy;
            best_patch.yaw = body_yaw;
            best_patch.length = patch_length_;
            best_patch.width = patch_width_;
        }
    }

    PatchRisk evaluatePatch(
        const grid_map::GridMap& map,
        const Patch& patch) const
    {
        PatchRisk risk;

        std::vector<Eigen::Vector3d> points;
        std::vector<double> variances;

        double z_min = std::numeric_limits<double>::infinity();
        double z_max = -std::numeric_limits<double>::infinity();

        for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
            grid_map::Position pos;
            map.getPosition(*it, pos);

            if (!insideOrientedRect(
                    pos.x(),
                    pos.y(),
                    patch.cx,
                    patch.cy,
                    patch.yaw,
                    patch.length,
                    patch.width)) {
                continue;
            }

            risk.expected_cells++;

            const float valid_value = map.at(valid_layer_, *it);
            const bool is_valid =
                std::isfinite(valid_value) && valid_value > 0.5f;

            if (!is_valid) {
                continue;
            }

            const float z = map.at(height_layer_, *it);
            if (!std::isfinite(z)) {
                continue;
            }

            risk.valid_cells++;

            points.emplace_back(pos.x(), pos.y(), static_cast<double>(z));

            z_min = std::min(z_min, static_cast<double>(z));
            z_max = std::max(z_max, static_cast<double>(z));

            if (map.exists(variance_layer_)) {
                const float var = map.at(variance_layer_, *it);
                if (std::isfinite(var)) {
                    variances.push_back(static_cast<double>(var));
                }
            }
        }

        if (risk.expected_cells <= 0) {
            risk.unknown_ratio = 1.0;
            risk.total_cost = 1.0;
            risk.valid = false;
            return risk;
        }

        risk.unknown_ratio =
            1.0 -
            static_cast<double>(risk.valid_cells) /
            static_cast<double>(risk.expected_cells);

        if (risk.valid_cells < min_valid_cells_ ||
            risk.unknown_ratio > max_unknown_ratio_) {
            risk.total_cost = 1.0;
            risk.valid = false;
            return risk;
        }

        risk.height_range = z_max - z_min;

        computePCA(points, risk);

        if (!variances.empty()) {
            double sum = 0.0;
            for (const auto v : variances) {
                sum += v;
            }
            risk.mean_variance = sum / static_cast<double>(variances.size());
        } else {
            risk.mean_variance = 0.0;
        }

        risk.total_cost = computeRiskCost(risk);
        risk.valid = true;

        return risk;
    }

    void computePCA(
        const std::vector<Eigen::Vector3d>& points,
        PatchRisk& risk) const
    {
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();

        for (const auto& p : points) {
            mean += p;
        }
        mean /= static_cast<double>(points.size());

        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();

        for (const auto& p : points) {
            const Eigen::Vector3d d = p - mean;
            cov += d * d.transpose();
        }
        cov /= static_cast<double>(points.size());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);

        Eigen::Vector3d normal = solver.eigenvectors().col(0);
        if (normal.z() < 0.0) {
            normal = -normal;
        }

        risk.centroid = mean;
        risk.normal = normal;

        const double nz =
            clampValue(normal.z(), -1.0, 1.0);

        risk.slope_deg =
            std::acos(nz) * 180.0 / M_PI;

        risk.roughness =
            std::sqrt(std::max(0.0, solver.eigenvalues()(0)));
    }

    double computeRiskCost(const PatchRisk& risk) const
    {
        const double unknown_cost =
            clampValue(risk.unknown_ratio, 0.0, 1.0);

        const double step_cost =
            clampValue(risk.height_range / max_height_range_, 0.0, 1.0);

        const double slope_cost =
            clampValue(risk.slope_deg / max_slope_deg_, 0.0, 1.0);

        const double roughness_cost =
            clampValue(risk.roughness / max_roughness_, 0.0, 1.0);

        const double variance_cost =
            clampValue(risk.mean_variance / max_variance_, 0.0, 1.0);

        const double weighted =
            w_unknown_ * unknown_cost +
            w_step_ * step_cost +
            w_slope_ * slope_cost +
            w_roughness_ * roughness_cost +
            w_variance_ * variance_cost;

        const double total_weight =
            std::max(
                1e-6,
                w_unknown_ +
                w_step_ +
                w_slope_ +
                w_roughness_ +
                w_variance_);

        return clampValue(weighted / total_weight, 0.0, 1.0);
    }

    std::pair<double, double> transformBodyPointToTerrain(
        const double body_x,
        const double body_y,
        const double body_yaw,
        const double local_x,
        const double local_y) const
    {
        const double c = std::cos(body_yaw);
        const double s = std::sin(body_yaw);

        const double x =
            body_x +
            c * local_x -
            s * local_y;

        const double y =
            body_y +
            s * local_x +
            c * local_y;

        return {x, y};
    }

    bool insideOrientedRect(
        const double px,
        const double py,
        const double cx,
        const double cy,
        const double yaw,
        const double length,
        const double width) const
    {
        const double dx = px - cx;
        const double dy = py - cy;

        const double c = std::cos(yaw);
        const double s = std::sin(yaw);

        const double local_x =  c * dx + s * dy;
        const double local_y = -s * dx + c * dy;

        return std::abs(local_x) <= 0.5 * length &&
               std::abs(local_y) <= 0.5 * width;
    }

    void clearOldMarkers(visualization_msgs::MarkerArray& markers) const
    {
        visualization_msgs::Marker m;
        m.header.frame_id = terrain_frame_;
        m.header.stamp = ros::Time::now();
        m.ns = "support_patch_debug";
        m.id = 0;
        m.action = visualization_msgs::Marker::DELETEALL;

        markers.markers.push_back(m);
    }

    std_msgs::ColorRGBA colorFromRisk(
        const PatchRisk& risk) const
    {
        std_msgs::ColorRGBA c;

        if (!risk.valid) {
            c.r = 0.4;
            c.g = 0.4;
            c.b = 0.4;
            c.a = 0.7;
            return c;
        }

        const double x =
            clampValue(risk.total_cost, 0.0, 1.0);

        c.r = x;
        c.g = 1.0 - x;
        c.b = 0.0;
        c.a = 0.75;

        return c;
    }

    void addEnvelopeMarker(
        visualization_msgs::MarkerArray& markers,
        const int id,
        const LimbEnvelope& limb,
        const double body_x,
        const double body_y,
        const double body_yaw) const
    {
        visualization_msgs::Marker m;
        m.header.frame_id = terrain_frame_;
        m.header.stamp = ros::Time::now();
        m.ns = "support_patch_debug";
        m.id = id;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;

        m.scale.x = 0.01;
        m.color.r = 0.1;
        m.color.g = 0.4;
        m.color.b = 1.0;
        m.color.a = 0.8;

        std::vector<std::pair<double, double>> corners_body = {
            {limb.x_min, limb.y_min},
            {limb.x_max, limb.y_min},
            {limb.x_max, limb.y_max},
            {limb.x_min, limb.y_max},
            {limb.x_min, limb.y_min}
        };

        for (const auto& p_body : corners_body) {
            const auto [x, y] =
                transformBodyPointToTerrain(
                    body_x,
                    body_y,
                    body_yaw,
                    p_body.first,
                    p_body.second);

            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = 0.05;
            m.points.push_back(p);
        }

        markers.markers.push_back(m);
    }

    void addPatchMarker(
        visualization_msgs::MarkerArray& markers,
        const int id,
        const Patch& patch,
        const PatchRisk& risk) const
    {
        visualization_msgs::Marker m;
        m.header.frame_id = terrain_frame_;
        m.header.stamp = ros::Time::now();
        m.ns = "support_patch_debug";
        m.id = id;
        m.type = visualization_msgs::Marker::CUBE;
        m.action = visualization_msgs::Marker::ADD;

        m.pose.position.x = patch.cx;
        m.pose.position.y = patch.cy;
        m.pose.position.z =
            risk.valid ? risk.centroid.z() + 0.02 : 0.05;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, patch.yaw);
        m.pose.orientation = tf2::toMsg(q);

        m.scale.x = patch.length;
        m.scale.y = patch.width;
        m.scale.z = 0.02;

        m.color = colorFromRisk(risk);

        markers.markers.push_back(m);
    }

    void addNormalMarker(
        visualization_msgs::MarkerArray& markers,
        const int id,
        const Patch& patch,
        const PatchRisk& risk) const
    {
        if (!risk.valid) {
            return;
        }

        visualization_msgs::Marker m;
        m.header.frame_id = terrain_frame_;
        m.header.stamp = ros::Time::now();
        m.ns = "support_patch_debug";
        m.id = id;
        m.type = visualization_msgs::Marker::ARROW;
        m.action = visualization_msgs::Marker::ADD;

        m.scale.x = 0.01;
        m.scale.y = 0.02;
        m.scale.z = 0.03;

        m.color.r = 1.0;
        m.color.g = 1.0;
        m.color.b = 1.0;
        m.color.a = 0.9;

        geometry_msgs::Point p0;
        p0.x = risk.centroid.x();
        p0.y = risk.centroid.y();
        p0.z = risk.centroid.z() + 0.03;

        geometry_msgs::Point p1;
        p1.x = risk.centroid.x() + 0.15 * risk.normal.x();
        p1.y = risk.centroid.y() + 0.15 * risk.normal.y();
        p1.z = risk.centroid.z() + 0.03 + 0.15 * risk.normal.z();

        m.points.push_back(p0);
        m.points.push_back(p1);

        markers.markers.push_back(m);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber grid_sub_;
    ros::Publisher marker_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string terrain_topic_;
    std::string terrain_frame_;
    std::string body_frame_;

    std::string height_layer_;
    std::string valid_layer_;
    std::string variance_layer_;

    std::string marker_topic_;

    std::vector<LimbEnvelope> limbs_;

    double candidate_stride_x_{0.05};
    double candidate_stride_y_{0.05};

    double patch_length_{0.12};
    double patch_width_{0.10};

    double analysis_length_{0.24};
    double analysis_width_{0.18};

    int min_valid_cells_{8};

    double max_unknown_ratio_{0.60};
    double max_height_range_{0.08};
    double max_slope_deg_{20.0};
    double max_roughness_{0.03};
    double max_variance_{0.10};

    double w_unknown_{1.0};
    double w_step_{1.0};
    double w_slope_{1.0};
    double w_roughness_{0.7};
    double w_variance_{0.4};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "support_patch_debug_node");

    SupportPatchDebugNode node;

    ros::spin();

    return 0;
}