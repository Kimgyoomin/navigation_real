#include "pongbot_traversability/support_patch_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <grid_map_core/iterators/GridMapIterator.hpp>

namespace pongbot_traversability
{

void SupportPatchEvaluator::setLayers(
    const std::string& height_layer,
    const std::string& valid_layer,
    const std::string& variance_layer)
{
    height_layer_ = height_layer;
    valid_layer_ = valid_layer;
    variance_layer_ = variance_layer;
}

void SupportPatchEvaluator::setPatchSize(
    const double patch_length,
    const double patch_width)
{
    patch_length_ = patch_length;
    patch_width_ = patch_width;
}

void SupportPatchEvaluator::setAnalysisWindow(
    const double analysis_length,
    const double analysis_width)
{
    analysis_length_ = analysis_length;
    analysis_width_ = analysis_width;
}

void SupportPatchEvaluator::setCandidateStride(
    const double stride_x,
    const double stride_y)
{
    candidate_stride_x_ = stride_x;
    candidate_stride_y_ = stride_y;
}

void SupportPatchEvaluator::setRiskLimits(const PatchRiskLimits& limits)
{
    limits_ = limits;
}

void SupportPatchEvaluator::setRiskWeights(const PatchRiskWeights& weights)
{
    weights_ = weights;
}

bool SupportPatchEvaluator::checkLayers(const grid_map::GridMap& map) const
{
    return map.exists(height_layer_) && map.exists(valid_layer_);
}

void SupportPatchEvaluator::evaluateLimb(
    const grid_map::GridMap& map,
    const LimbEnvelope& limb,
    const double body_x,
    const double body_y,
    const double body_yaw,
    Patch& best_patch,
    PatchRisk& best_risk) const
{
    bool found_valid_patch = false;

    best_risk.total_cost = std::numeric_limits<double>::infinity();
    best_risk.valid = false;

    for (double bx = limb.x_min; bx <= limb.x_max + 1e-6; bx += candidate_stride_x_) {
        for (double by = limb.y_min; by <= limb.y_max + 1e-6; by += candidate_stride_y_) {
            const std::pair<double, double> p =
                transformBodyPointToTerrain(body_x, body_y, body_yaw, bx, by);

            Patch patch;
            patch.limb_name = limb.name;
            patch.cx = p.first;
            patch.cy = p.second;
            patch.yaw = body_yaw;
            patch.length = patch_length_;
            patch.width = patch_width_;

            Patch analysis_patch = patch;
            analysis_patch.length = analysis_length_;
            analysis_patch.width = analysis_width_;

            const PatchRisk risk = evaluatePatch(map, analysis_patch);

            if (!risk.valid) {
                continue;
            }

            if (!found_valid_patch || risk.total_cost < best_risk.total_cost) {
                found_valid_patch = true;
                best_patch = patch;
                best_risk = risk;
            }
        }
    }

    if (!found_valid_patch) {
        const std::pair<double, double> p =
            transformBodyPointToTerrain(
                body_x,
                body_y,
                body_yaw,
                0.5 * (limb.x_min + limb.x_max),
                0.5 * (limb.y_min + limb.y_max));

        best_patch.limb_name = limb.name;
        best_patch.cx = p.first;
        best_patch.cy = p.second;
        best_patch.yaw = body_yaw;
        best_patch.length = patch_length_;
        best_patch.width = patch_width_;

        Patch analysis_patch = best_patch;
        analysis_patch.length = analysis_length_;
        analysis_patch.width = analysis_width_;

        best_risk = evaluatePatch(map, analysis_patch);
        best_risk.valid = false;
        best_risk.total_cost = 1.0;
    }
}

PatchRisk SupportPatchEvaluator::evaluatePatch(
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

    if (risk.valid_cells < limits_.min_valid_cells ||
        risk.unknown_ratio > limits_.max_unknown_ratio) {
        risk.total_cost = 1.0;
        risk.valid = false;
        return risk;
    }

    risk.height_range = z_max - z_min;

    computePCA(points, risk);

    if (!variances.empty()) {
        double sum = 0.0;
        for (const double v : variances) {
            sum += v;
        }
        risk.mean_variance = sum / static_cast<double>(variances.size());
    }

    risk.total_cost = computeRiskCost(risk);
    risk.valid = true;

    return risk;
}

void SupportPatchEvaluator::computePCA(
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

double SupportPatchEvaluator::computeRiskCost(
    const PatchRisk& risk) const
{
    const double unknown_cost =
        clampValue(risk.unknown_ratio, 0.0, 1.0);

    const double step_cost =
        clampValue(risk.height_range / limits_.max_height_range, 0.0, 1.0);

    const double slope_cost =
        clampValue(risk.slope_deg / limits_.max_slope_deg, 0.0, 1.0);

    const double roughness_cost =
        clampValue(risk.roughness / limits_.max_roughness, 0.0, 1.0);

    const double variance_cost =
        clampValue(risk.mean_variance / limits_.max_variance, 0.0, 1.0);

    const double weighted =
        weights_.unknown * unknown_cost +
        weights_.step * step_cost +
        weights_.slope * slope_cost +
        weights_.roughness * roughness_cost +
        weights_.variance * variance_cost;

    const double total_weight =
        std::max(
            1e-6,
            weights_.unknown +
            weights_.step +
            weights_.slope +
            weights_.roughness +
            weights_.variance);

    return clampValue(weighted / total_weight, 0.0, 1.0);
}

std::pair<double, double> SupportPatchEvaluator::transformBodyPointToTerrain(
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

    return std::make_pair(x, y);
}

bool SupportPatchEvaluator::insideOrientedRect(
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

}  // namespace pongbot_traversability