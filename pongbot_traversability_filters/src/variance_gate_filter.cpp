#include "pongbot_traversability_filters/variance_gate_filter.hpp"
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <cmath>

using grid_map::GridMap;

namespace pongbot_traversability_filters {

bool VarianceGateFilter::configure() {
    this->getParam("variance_layer",        var_layer_);
    this->getParam("out_layer",             out_layer_);
    this->getParam("var_max",               var_max_);
    if (var_max_ <= 1e-9) var_max_ = 1e-3;
    return true;
}

bool VarianceGateFilter::update(const grid_map::GridMap& in, grid_map::GridMap& out) {
    out = in;


    if(!out.exists(var_layer_)) {
        ROS_WARN_THROTTLE(2.0, "VarianceGateFilter : missing layer '%s' -> default gate = 1", var_layer_.c_str());
        if (!out.exists(out_layer_)) out.add(out_layer_, 1.0);
        return true;
    }

    out.add(out_layer_, 0.0);
    for(grid_map::GridMapIterator it(out); !it.isPastEnd(); ++it) {
        const float v = out.at(var_layer_, *it);
        const double gate = (std::isnan(v) ? 0.0 : std::max(0.0, 1.0 - (double)v / var_max_));
        out.at(out_layer_, *it) = gate;     // 1 : convincable , 0: unconvincable
    }
    return true;
}
}   // namespace pongbot_traversability_filters

PLUGINLIB_EXPORT_CLASS(pongbot_traversability_filters::VarianceGateFilter, 
                        filters::FilterBase<grid_map::GridMap>) 