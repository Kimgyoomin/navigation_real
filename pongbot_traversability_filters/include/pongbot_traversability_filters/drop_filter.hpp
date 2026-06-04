// include/pongbot_traversability_filters/drop_filter.hpp
#pragma once
#include <filters/filter_base.hpp>
#include <grid_map_core/GridMap.hpp>
// #include <grid_map_core/iterators/CircleIterator.hpp>
#include <string>

namespace pongbot_traversability_filters {

class DropFilter : public filters::FilterBase<grid_map::GridMap> {
public:
    bool configure() override;
    bool update(const grid_map::GridMap& in, grid_map::GridMap& out) override;

private:
    std::string elev_           = "elevation";
    std::string step_layer_     = "step_up";
    std::string drop_layer_     = "drop";
    double radius_              = 0.25;         // [m] surround check(based on footprint, gotta change later if needed)
    double h_max_step_          = 0.1;          // [m] max step height ; now for 10cm
    double h_max_drop_          = 0.2;          // [m] max drop height ; now for 20cm
};
}   // namespace pongbot_traversability_filters