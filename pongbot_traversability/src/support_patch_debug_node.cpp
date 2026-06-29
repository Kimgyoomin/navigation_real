#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>
#include <utility>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>

#include "pongbot_traversability/support_patch_types.hpp"
#include "pongbot_traversability/support_patch_evaluator.hpp"

namespace pt = pongbot_traversability;

class SupportPatchDebugNode
{
public:
    SupportPatchDebugNode()
        : nh_(),
          pnh_("~"),
          tf_listener_(tf_buffer_)
    {
        loadParams();

        evaluator_.setLayers(height_layer_, valid_layer_, variance_layer_);
        evaluator_.setPatchSize(patch_length_, patch_width_);
        evaluator_.setAnalysisWindow(analysis_length_, analysis_width_);
        evaluator_.setCandidateStride(candidate_stride_x_, candidate_stride_y_);
        evaluator_.setRiskLimits(risk_limits_);
        evaluator_.setRiskWeights(risk_weights_);

        marker_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);

        grid_sub_ =
            nh_.subscribe(
                terrain_topic_,
                1,
                &SupportPatchDebugNode::gridCallback,
                this);

        ROS_INFO("[support_patch_debug] terrain_topic=%s terrain_frame=%s body_frame=%s marker_topic=%s",
                 terrain_topic_.c_str(),
                 terrain_frame_.c_str(),
                 body_frame_.c_str(),
                 marker_topic_.c_str());
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

        pnh_.param<int>("risk/min_valid_cells", risk_limits_.min_valid_cells, 8);
        pnh_.param<double>("risk/max_unknown_ratio", risk_limits_.max_unknown_ratio, 0.60);
        pnh_.param<double>("risk/max_height_range", risk_limits_.max_height_range, 0.08);
        pnh_.param<double>("risk/max_slope_deg", risk_limits_.max_slope_deg, 20.0);
        pnh_.param<double>("risk/max_roughness", risk_limits_.max_roughness, 0.03);
        pnh_.param<double>("risk/max_variance", risk_limits_.max_variance, 0.10);

        pnh_.param<double>("weights/unknown", risk_weights_.unknown, 1.0);
        pnh_.param<double>("weights/step", risk_weights_.step, 1.0);
        pnh_.param<double>("weights/slope", risk_weights_.slope, 1.0);
        pnh_.param<double>("weights/roughness", risk_weights_.roughness, 0.7);
        pnh_.param<double>("weights/variance", risk_weights_.variance, 0.4);

        pnh_.param<std::string>("visualization/marker_topic",
                                marker_topic_,
                                "/legged_terrain/support_patch_markers");

        loadLimb("FL", 0.18, 0.35, 0.10, 0.22);
        loadLimb("FR", 0.18, 0.35, -0.22, -0.10);
        loadLimb("RL", -0.35, -0.18, 0.10, 0.22);
        loadLimb("RR", -0.35, -0.18, -0.22, -0.10);
    }

    void loadLimb(
        const std::string& name,
        const double default_x_min,
        const double default_x_max,
        const double default_y_min,
        const double default_y_max)
    {
        std::vector<double> v;
        if (!pnh_.getParam("limbs/" + name, v) || v.size() != 4) {
            v = {default_x_min, default_x_max, default_y_min, default_y_max};
            ROS_WARN("[support_patch_debug] Using default envelope for %s", name.c_str());
        }

        pt::LimbEnvelope limb;
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
        const std::string grid_frame =
            map.getFrameId().empty() ? terrain_frame_ : map.getFrameId();

        if (!evaluator_.checkLayers(map)) {
            ROS_WARN_THROTTLE(
                1.0,
                "[support_patch_debug] Missing required layers. height=%s valid=%s",
                height_layer_.c_str(),
                valid_layer_.c_str());
            return;
        }

        geometry_msgs::TransformStamped tf_msg;
        try {
            tf_msg =
                tf_buffer_.lookupTransform(
                    grid_frame,
                    body_frame_,
                    ros::Time(0),
                    ros::Duration(0.05));
        } catch (const tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(
                1.0,
                "[support_patch_debug] TF lookup failed %s <- %s: %s",
                grid_frame.c_str(),
                body_frame_.c_str(),
                ex.what());
            return;
        }

        const double body_x = tf_msg.transform.translation.x;
        const double body_y = tf_msg.transform.translation.y;
        const double body_yaw = tf2::getYaw(tf_msg.transform.rotation);

        visualization_msgs::MarkerArray markers;
        int marker_id = 1;

        addDeleteAllMarker(markers, grid_frame);

        double body_risk = 0.0;

        for (const auto& limb : limbs_) {
            addEnvelopeMarker(
                markers,
                marker_id++,
                grid_frame,
                limb,
                body_x,
                body_y,
                body_yaw);

            pt::Patch best_patch;
            pt::PatchRisk best_risk;

            evaluator_.evaluateLimb(
                map,
                limb,
                body_x,
                body_y,
                body_yaw,
                best_patch,
                best_risk);

            body_risk = std::max(body_risk, best_risk.total_cost);

            addPatchMarker(
                markers,
                marker_id++,
                grid_frame,
                best_patch,
                best_risk);

            addNormalMarker(
                markers,
                marker_id++,
                grid_frame,
                best_risk);

            addTextMarker(
                markers,
                marker_id++,
                grid_frame,
                best_patch,
                best_risk);
        }

        ROS_INFO_THROTTLE(
            1.0,
            "[support_patch_debug] frame=%s body=(%.2f, %.2f, %.2f deg) body_risk=%.3f",
            grid_frame.c_str(),
            body_x,
            body_y,
            body_yaw * 180.0 / M_PI,
            body_risk);

        marker_pub_.publish(markers);
    }

    void addDeleteAllMarker(
        visualization_msgs::MarkerArray& markers,
        const std::string& frame_id) const
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "support_patch_debug";
        marker.id = 0;
        marker.action = visualization_msgs::Marker::DELETEALL;

        markers.markers.push_back(marker);
    }


    std_msgs::ColorRGBA colorFromRisk(const pt::PatchRisk& risk) const
    {
        std_msgs::ColorRGBA color;

        if (!risk.valid) {
            color.r = 0.45;
            color.g = 0.45;
            color.b = 0.45;
            color.a = 0.75;
            return color;
        }

        const double x = clampValue(risk.total_cost, 0.0, 1.0);

        color.r = x;
        color.g = 1.0 - x;
        color.b = 0.0;
        color.a = 0.75;

        return color;
    }

    void addEnvelopeMarker(
        visualization_msgs::MarkerArray& markers,
        const int id,
        const std::string& frame_id,
        const pt::LimbEnvelope& limb,
        const double body_x,
        const double body_y,
        const double body_yaw) const
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "support_patch_debug";
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.01;
        marker.color.r = 0.1;
        marker.color.g = 0.4;
        marker.color.b = 1.0;
        marker.color.a = 0.8;

        const std::vector<std::pair<double, double>> corners_body = {
            {limb.x_min, limb.y_min},
            {limb.x_max, limb.y_min},
            {limb.x_max, limb.y_max},
            {limb.x_min, limb.y_max},
            {limb.x_min, limb.y_min}
        };

        for (const auto& p_body : corners_body) {
            const std::pair<double, double> p =
                evaluator_.transformBodyPointToTerrain(
                    body_x,
                    body_y,
                    body_yaw,
                    p_body.first,
                    p_body.second);

            geometry_msgs::Point point;
            point.x = p.first;
            point.y = p.second;
            point.z = 0.05;

            marker.points.push_back(point);
        }

        markers.markers.push_back(marker);
    }

    void addPatchMarker(
        visualization_msgs::MarkerArray& markers,
        const int id,
        const std::string& frame_id,
        const pt::Patch& patch,
        const pt::PatchRisk& risk) const
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "support_patch_debug";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = patch.cx;
        marker.pose.position.y = patch.cy;
        marker.pose.position.z =
            risk.valid ? risk.centroid.z() + 0.02 : 0.05;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, patch.yaw);
        marker.pose.orientation = tf2::toMsg(q);

        marker.scale.x = patch.length;
        marker.scale.y = patch.width;
        marker.scale.z = 0.02;

        marker.color = colorFromRisk(risk);

        markers.markers.push_back(marker);
    }

    void addNormalMarker(
        visualization_msgs::MarkerArray& markers,
        const int id,
        const std::string& frame_id,
        const pt::PatchRisk& risk) const
    {
        if (!risk.valid) {
            return;
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "support_patch_debug";
        marker.id = id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.01;
        marker.scale.y = 0.02;
        marker.scale.z = 0.03;

        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 0.9;

        geometry_msgs::Point p0;
        p0.x = risk.centroid.x();
        p0.y = risk.centroid.y();
        p0.z = risk.centroid.z() + 0.03;

        geometry_msgs::Point p1;
        p1.x = risk.centroid.x() + 0.15 * risk.normal.x();
        p1.y = risk.centroid.y() + 0.15 * risk.normal.y();
        p1.z = risk.centroid.z() + 0.03 + 0.15 * risk.normal.z();

        marker.points.push_back(p0);
        marker.points.push_back(p1);

        markers.markers.push_back(marker);
    }

    void addTextMarker(
        visualization_msgs::MarkerArray& markers,
        const int id,
        const std::string& frame_id,
        const pt::Patch& patch,
        const pt::PatchRisk& risk) const
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "support_patch_debug";
        marker.id = id;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = patch.cx;
        marker.pose.position.y = patch.cy;
        marker.pose.position.z =
            risk.valid ? risk.centroid.z() + 0.12 : 0.15;

        marker.scale.z = 0.06;

        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 0.9;

        char buffer[256];
        std::snprintf(
            buffer,
            sizeof(buffer),
            "%s\nc=%.2f\nv=%d/%d\ns=%.1f",
            patch.limb_name.c_str(),
            risk.total_cost,
            risk.valid_cells,
            risk.expected_cells,
            risk.slope_deg);

        marker.text = buffer;

        markers.markers.push_back(marker);
    }

    template <typename T>
    static T clampValue(const T value, const T low, const T high)
    {
        return std::max(low, std::min(value, high));
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber grid_sub_;
    ros::Publisher marker_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    pt::SupportPatchEvaluator evaluator_;

    std::string terrain_topic_;
    std::string terrain_frame_;
    std::string body_frame_;

    std::string height_layer_;
    std::string valid_layer_;
    std::string variance_layer_;

    std::string marker_topic_;

    std::vector<pt::LimbEnvelope> limbs_;

    double candidate_stride_x_{0.05};
    double candidate_stride_y_{0.05};

    double patch_length_{0.12};
    double patch_width_{0.10};

    double analysis_length_{0.24};
    double analysis_width_{0.18};

    pt::PatchRiskLimits risk_limits_;
    pt::PatchRiskWeights risk_weights_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "support_patch_debug_node");

    SupportPatchDebugNode node;

    ros::spin();

    return 0;
}