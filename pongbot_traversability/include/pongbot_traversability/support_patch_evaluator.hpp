#pragma once

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>

#include "pongbot_traversability/support_patch_types.hpp"

namespace pongbot_traversability
{

class SupportPatchEvaluator
{
public:
    SupportPatchEvaluator() = default;

    void setLayers(const std::string& height_layer,
                   const std::string& valid_layer,
                   const std::string& variance_layer);

    void setPatchSize(double patch_length, double patch_width);
    void setAnalysisWindow(double analysis_length, double analysis_width);

    void setCandidateStride(double stride_x, double stride_y);

    void setRiskLimits(const PatchRiskLimits& limits);
    void setRiskWeights(const PatchRiskWeights& weights);

    bool checkLayers(const grid_map::GridMap& map) const;

    void evaluateLimb(const grid_map::GridMap& map,
                      const LimbEnvelope& limb,
                      double body_x,
                      double body_y,
                      double body_yaw,
                      Patch& best_patch,
                      PatchRisk& best_risk) const;

    PatchRisk evaluatePatch(const grid_map::GridMap& map,
                            const Patch& patch) const;

    std::pair<double, double> transformBodyPointToTerrain(
        double body_x,
        double body_y,
        double body_yaw,
        double local_x,
        double local_y) const;

private:
    bool insideOrientedRect(double px,
                            double py,
                            double cx,
                            double cy,
                            double yaw,
                            double length,
                            double width) const;

    void computePCA(const std::vector<Eigen::Vector3d>& points,
                    PatchRisk& risk) const;

    double computeRiskCost(const PatchRisk& risk) const;

    template <typename T>
    static T clampValue(T value, T low, T high)
    {
        return std::max(low, std::min(value, high));
    }

private:
    std::string height_layer_{"filter_map"};
    std::string valid_layer_{"is_valid"};
    std::string variance_layer_{"variance"};

    double candidate_stride_x_{0.05};
    double candidate_stride_y_{0.05};

    double patch_length_{0.12};
    double patch_width_{0.10};

    double analysis_length_{0.24};
    double analysis_width_{0.18};

    PatchRiskLimits limits_;
    PatchRiskWeights weights_;
};

}  // namespace pongbot_traversability