#pragma once
#include <filters/filter_base.hpp>
#include <grid_map_core/GridMap.hpp>
#include <string>

namespace pongbot_traversability_filters{

class VarianceGateFilter : public filters::FilterBase<grid_map::GridMap> {
public:
    bool configure() override;
    bool update(const grid_map::GridMap& in, grid_map::GridMap& out) override;

private:
    std::string var_layer_      = "variance";   // elevation mapping will predict
    std::string out_layer_      = "var_gate";
    double var_max_             = 0.05;         // [m^2]
};
}   // namespace pongbot_traversability_filters