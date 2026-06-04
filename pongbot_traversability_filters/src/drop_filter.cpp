#include "pongbot_traversability_filters/drop_filter.hpp"
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <limits>
#include <cmath>


using grid_map::GridMap; 
using grid_map::Position;

// Based on Center, For circular window -> search for max/min altitude
// Based on Center altitude, + ; step_up, - ; drop
// For traversability setup
namespace pongbot_traversability_filters{

bool DropFilter::configure(){
    // ros::NodeHandle nh("~");
    this->getParam("elevation_layer",       elev_);
    this->getParam("step_layer",            step_layer_);
    this->getParam("drop_layer",            drop_layer_);
    this->getParam("window_radius",         radius_);
    this->getParam("max_step_height",       h_max_step_);
    this->getParam("max_drop_height",       h_max_drop_);

    // Under Guard
    if (radius_         <= 1e-6) radius_        = 0.05;
    if (h_max_step_     <= 1e-6) h_max_step_    = 0.01;
    if (h_max_drop_     <= 1e-6) h_max_drop_    = 0.01;
    return true;
}


bool DropFilter::update(const GridMap& in, GridMap& out)
{
    out = in;           // copy (keep layer)


    if(!out.exists(elev_)) {
        ROS_WARN_THROTTLE(2.0, "DropFilter: missing layer '%s' -> pass-through", elev_.c_str());
        if (!out.exists(step_layer_)) out.add(step_layer_, NAN);
        if (!out.exists(drop_layer_)) out.add(drop_layer_, NAN);
        return true;
    }

    out.add(step_layer_, 0.0);
    out.add(drop_layer_, 0.0);

    for(grid_map::GridMapIterator it(out); !it.isPastEnd(); ++it) {
        const auto idx = *it;
        float zc = out.at(elev_, idx);
        if(std::isnan(zc)) { out.at(step_layer_, idx)=NAN; out.at(drop_layer_, idx)=NAN; continue; }

        Position p;
        out.getPosition(idx, p);
        double zmin = std::numeric_limits<double>::infinity();
        double zmax = -std::numeric_limits<double>::infinity();

        for (grid_map::CircleIterator cit(out, p, radius_); !cit.isPastEnd(); ++cit) {
            float zn = out.at(elev_, *cit);
            if(std::isnan(zn)) continue;
            zmin = std::min(zmin, (double)zn);
            zmax = std::max(zmax, (double)zn);
        }

        if(!std::isfinite(zmin) || !std::isfinite(zmax)) { 
            out.at(step_layer_, idx)=NAN; out.at(drop_layer_, idx)=NAN; continue; 
        }

        const double step       = std::max(0.0, zmax - zc);         // front is higher, you've got to go up
        const double drop       = std::max(0.0, zc - zmin);         // front is lower , you've got to aware of drop

        // 0 ~ 1 Regularization(clamp)
        out.at(step_layer_, idx)    = std::min(1.0, step / h_max_step_);
        out.at(drop_layer_, idx)    = std::min(1.0, drop / h_max_drop_);
    }

    return true;
}

}   // ns
PLUGINLIB_EXPORT_CLASS(pongbot_traversability_filters::DropFilter, 
                        filters::FilterBase<grid_map::GridMap>)