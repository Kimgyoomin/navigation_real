#pragma once

#include <Eigen/Dense>
#include <string>

namespace pongbot_traversability
{

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

struct PatchRiskWeights
{
    double unknown{1.0};
    double step{1.0};
    double slope{1.0};
    double roughness{0.7};
    double variance{0.4};
};

struct PatchRiskLimits
{
    int min_valid_cells{8};

    double max_unknown_ratio{0.60};
    double max_height_range{0.08};
    double max_slope_deg{20.0};
    double max_roughness{0.03};
    double max_variance{0.10};
};

}  // namespace pongbot_traversability