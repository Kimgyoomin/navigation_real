#include <limits>
#include <cmath>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include "pongbot_traversability_filters/morph_drop_filter.hpp"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

using grid_map::GridMap;
using grid_map::GridMapCvConverter;

namespace pongbot_traversability_filters {


int MorphDropFilter::clampOdd(int k) {
    if (k < 1) k = 1;
    if ((k % 2) == 0) ++k;
    return k;
}

bool MorphDropFilter::configure() {
    // Load Chain Paramter (if None, use basic value)
    this->getParam("elevation_layer",       elev_);
    this->getParam("step_layer",            step_layer_);
    this->getParam("drop_layer",            drop_layer_);
    this->getParam("window_radius",         window_radius_m_);
    this->getParam("max_step_height",       h_max_step_m_);
    this->getParam("max_drop_height",       h_max_drop_m_);
    this->getParam("use_square_kernel",     use_square_kernel_);

    this->getParam("step_h_layer",          step_h_layer_);
    this->getParam("drop_h_layer",          drop_h_layer_);
    this->getParam("step_over_layer",       step_over_layer_);
    this->getParam("drop_over_layer",       drop_over_layer_);
    this->getParam("output_metric_layers",  output_metric_layers_);
    this->getParam("output_over_mask",      output_over_mask_);
    this->getParam("output_cm_bins",        output_cm_bins_);
    this->getParam("output_normalized",     output_normalized_);
    this->getParam("cm_cap",                cm_cap_);
    if (cm_cap_ < 0) cm_cap_ = 0;

    // stablity guard(under)
    if (window_radius_m_ <= 1e-6) { ROS_WARN("MorphDropFilter : window radius is too small , clamp to 0.05[m]");
                                    window_radius_m_ = 0.05; }
    if (h_max_step_m_    <= 1e-6) { ROS_WARN("MorphDropFilter : max step height is too small , clamp to 0.05[m]");
                                    h_max_step_m_ = 0.05; }
    if (h_max_drop_m_    <= 1e-6) { ROS_WARN("MorphDropFilter : max drop height is too small , clamp to 0.05[m]");
                                    h_max_drop_m_ = 0.05; }
    return true;
}


bool MorphDropFilter::update(const GridMap& in, GridMap& out) {
    out = in;       // copy input (keep Multilayer)

    if (!out.exists(elev_)) {
        ROS_WARN_THROTTLE(2.0, "MorphDropFilter: missing input layer '%s' -> pass through", elev_.c_str());
        if (output_normalized_) {
            if (!out.exists(step_layer_)) out.add(step_layer_, std::numeric_limits<float>::quiet_NaN());
            if (!out.exists(drop_layer_)) out.add(drop_layer_, std::numeric_limits<float>::quiet_NaN());
        }
        // prevent from chain cutter : just add output layer , fillout with 0 / NaN and return true
        // if (!out.exists(step_layer_)) out.add(step_layer_, NAN);
        // if (!out.exists(drop_layer_)) out.add(drop_layer_, NAN);
        if (output_metric_layers_) {
            if (!out.exists(step_h_layer_)) out.add(step_h_layer_, std::numeric_limits<float>::quiet_NaN());
            if (!out.exists(drop_h_layer_)) out.add(drop_h_layer_, std::numeric_limits<float>::quiet_NaN());
        }
        if (output_over_mask_) {
            if (!out.exists(step_over_layer_)) out.add(step_over_layer_, 0.0);
            if (!out.exists(drop_over_layer_)) out.add(drop_over_layer_, 0.0);
        }

        if (output_cm_bins_) {
            if (!out.exists("step_up_cm")) out.add("step_up_cm", 0.0);
            if (!out.exists("drop_cm")) out.add("drop_cm",       0.0);
        }
        return true;
    }

    // -----------------------------------------------
    // 1) GridMap -> cv::Mat (RAW meters for absolute / over/ bin)
    // -----------------------------------------------
    const auto size = out.getSize();                // (cols, rows)
    const int cols = size(0), rows = size(1);
    cv::Mat elev_raw(rows, cols, CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
    for (grid_map::GridMapIterator it(out); !it.isPastEnd(); ++it) {
        const grid_map::Index idx = *it;            // idx(0) = col, idx(1) = row
        elev_raw.at<float>(idx(1), idx(0)) = out.at(elev_, idx);        // [m]
    }

    // ---------------------------------------------------
    // 2) Kernel
    // ---------------------------------------------------
    const double res = out.getResolution();
    const int pxRadius = std::max(1, static_cast<int>(std::ceil(window_radius_m_ / std::max(1e-9, res))));
    const int k = clampOdd(2 * pxRadius + 1);
    cv::Mat kernel = cv::getStructuringElement(
        use_square_kernel_ ? cv::MORPH_RECT : cv::MORPH_ELLIPSE, cv::Size(k, k)
    );


    // ------------------------------------------------------
    // 3) Morphology on RAW (NaN - safe)
    // ------------------------------------------------------
    cv::Mat elev_for_max = elev_raw.clone(), elev_for_min = elev_raw.clone();
    for (int r = 0; r < rows; ++r) {
        float* pmax = elev_for_max.ptr<float>(r);
        float* pmin = elev_for_min.ptr<float>(r);
        for (int c = 0; c < cols; ++c) {
            if (!std::isfinite(pmax[c])) pmax[c] = -std::numeric_limits<float>::infinity();     // for dilation
            if (!std::isfinite(pmin[c])) pmin[c] = std::numeric_limits<float>::infinity();      // for erosion
        }
    }
    cv::Mat zmax_raw, zmin_raw;
    cv::dilate(
        elev_for_max,
        zmax_raw,
        kernel,
        cv::Point(-1, -1),
        1,
        cv::BORDER_CONSTANT,
        cv::Scalar(-std::numeric_limits<float>::infinity())
    );
    cv::erode(
        elev_for_min,
        zmin_raw,
        kernel,
        cv::Point(-1, -1),
        1,
        cv::BORDER_CONSTANT,
        cv::Scalar(std::numeric_limits<float>::infinity())
    );


    // -------------------------------------------------------
    // 4) For absolute height
    // -------------------------------------------------------
    cv::Mat step_h_raw = zmax_raw - elev_raw;
    cv::Mat drop_h_raw = elev_raw - zmin_raw;
    cv::threshold(step_h_raw, step_h_raw, 0.0, 0.0, cv::THRESH_TOZERO);
    cv::threshold(drop_h_raw, drop_h_raw, 0.0, 0.0, cv::THRESH_TOZERO);

    cv::Mat step_h = step_h_raw.clone();
    cv::Mat drop_h = drop_h_raw.clone();
    cv::threshold(step_h, step_h, static_cast<float>(h_max_step_m_), static_cast<float>(h_max_step_m_), cv::THRESH_TRUNC);
    cv::threshold(drop_h, drop_h, static_cast<float>(h_max_drop_m_), static_cast<float>(h_max_drop_m_), cv::THRESH_TRUNC);

    cv::Mat valid = elev_raw == elev_raw;  // NaN은 false
    step_h_raw.setTo(std::numeric_limits<float>::quiet_NaN(), ~valid);
    drop_h_raw.setTo(std::numeric_limits<float>::quiet_NaN(), ~valid);
    step_h.setTo(std::numeric_limits<float>::quiet_NaN(), ~valid);
    drop_h.setTo(std::numeric_limits<float>::quiet_NaN(), ~valid);

    const bool need_metric_layers = output_metric_layers_ || output_cm_bins_;
    if (need_metric_layers) {
        if (!out.exists(step_h_layer_)) out.add(step_h_layer_, std::numeric_limits<float>::quiet_NaN());
        if (!out.exists(drop_h_layer_)) out.add(drop_h_layer_, std::numeric_limits<float>::quiet_NaN());
    }

    if (output_over_mask_) {
        if (!out.exists(step_over_layer_)) out.add(step_over_layer_, 0.0);
        if (!out.exists(drop_over_layer_)) out.add(drop_over_layer_, 0.0);
    }

    if (output_cm_bins_) {
        if (!out.exists("step_up_cm")) out.add("step_up_cm", std::numeric_limits<float>::quiet_NaN());
        if (!out.exists("drop_cm")) out.add("drop_cm", std::numeric_limits<float>::quiet_NaN());
    }

    if (output_normalized_) {
        if (!out.exists(step_layer_)) out.add(step_layer_, std::numeric_limits<float>::quiet_NaN());
        if (!out.exists(drop_layer_)) out.add(drop_layer_, std::numeric_limits<float>::quiet_NaN());
    }

    // ------------------------------------------------------
    // 5) over limit mask (optional)
    // ------------------------------------------------------
    cv::Mat step_over, drop_over;
    if (output_over_mask_) {
        const float eps = 1e-6f;
        cv::compare(step_h, static_cast<float>(h_max_step_m_) - eps, step_over, cv::CMP_GE);
        cv::compare(drop_h, static_cast<float>(h_max_drop_m_) - eps, drop_over, cv::CMP_GE);
        step_over.convertTo(step_over, CV_32FC1, 1.0f / 255.0f);
        drop_over.convertTo(drop_over, CV_32FC1, 1.0f / 255.0f);
    }

    // ------------------------------------------------------
    // 6) 1cm binning (optional)
    // ------------------------------------------------------
    cv::Mat step_cm, drop_cm;
    if (output_cm_bins_) {
        step_h_raw.convertTo(step_cm, CV_32FC1, 100.0f);
        drop_h_raw.convertTo(drop_cm, CV_32FC1, 100.0f);

        for (int r = 0; r < step_cm.rows; ++r) {
            float* ps = step_cm.ptr<float>(r);
            float* pd = drop_cm.ptr<float>(r);
            for (int c = 0; c < step_cm.cols; ++c) {
                if (std::isfinite(ps[c])) ps[c] = std::floor(ps[c] + 1e-6f);
                if (std::isfinite(pd[c])) pd[c] = std::floor(pd[c] + 1e-6f);
            }
        }

        const float step_cm_upper = (cm_cap_ > 0)
                                        ? static_cast<float>(cm_cap_)
                                        : static_cast<float>(100.0 * h_max_step_m_);
        const float drop_cm_upper = (cm_cap_ > 0)
                                        ? static_cast<float>(cm_cap_)
                                        : static_cast<float>(100.0 * h_max_drop_m_);

        cv::threshold(step_cm, step_cm, step_cm_upper, step_cm_upper, cv::THRESH_TRUNC);
        cv::threshold(drop_cm, drop_cm, drop_cm_upper, drop_cm_upper, cv::THRESH_TRUNC);

        step_cm.setTo(std::numeric_limits<float>::quiet_NaN(), ~valid);
        drop_cm.setTo(std::numeric_limits<float>::quiet_NaN(), ~valid);
    }

    // ------------------------------------------------------
    // 7) (Optional) 0 ~ 1 Regularized Layer
    // ------------------------------------------------------
    cv::Mat step01, drop01;
    if (output_normalized_) {
        step01 = step_h / static_cast<float>(h_max_step_m_);
        drop01 = drop_h / static_cast<float>(h_max_drop_m_);
        cv::min(step01, 1.0, step01);
        cv::min(drop01, 1.0, drop01);
        step01.setTo(std::numeric_limits<float>::quiet_NaN(), ~valid);
        drop01.setTo(std::numeric_limits<float>::quiet_NaN(), ~valid);
    }

    // ------------------------------------------------------
    // 8) Copy back into GridMap layers
    // ------------------------------------------------------
    for (grid_map::GridMapIterator it(out); !it.isPastEnd(); ++it) {
        const grid_map::Index idx = *it;
        const int r = idx(1);
        const int c = idx(0);

        const float step_val = step_h.at<float>(r, c);
        const float drop_val = drop_h.at<float>(r, c);

        if (need_metric_layers) {
            out.at(step_h_layer_, idx) = step_val;
            out.at(drop_h_layer_, idx) = drop_val;
        }

        if (output_over_mask_) {
            out.at(step_over_layer_, idx) = step_over.at<float>(r, c);
            out.at(drop_over_layer_, idx) = drop_over.at<float>(r, c);
        }

        if (output_cm_bins_) {
            out.at("step_up_cm", idx) = step_cm.at<float>(r, c);
            out.at("drop_cm", idx) = drop_cm.at<float>(r, c);
        }

        if (output_normalized_) {
            out.at(step_layer_, idx) = step01.at<float>(r, c);
            out.at(drop_layer_, idx) = drop01.at<float>(r, c);
        }
    }

    return true;
}

} // namespace pongbot_traversability_filters

// Register pluginlib
PLUGINLIB_EXPORT_CLASS(pongbot_traversability_filters::MorphDropFilter,
                        filters::FilterBase<grid_map::GridMap>)
