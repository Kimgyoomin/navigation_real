#pragma once
#include <filters/filter_base.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <algorithm>
#include <string>
#include <vector>

namespace pongbot_traversability_filters {

/**
 * MorphDropFIlter
 * - Convert elevation layer to OpenCV image
 * - Kernel(Window) Size = radius / map_resolution , erosion/delation process
 * - zmax,zmin approximation , step_up / drop calculation (0 ~ 1 Regularization) 
 * - Output to new Layer
 * 
 * - Output :
 *      - step_up : 0 ~ 1(bigger, "Needs bigger step" , harder)
 *      - drop    : 0 ~ 1(bigger, "Drop danger bigger", harder)
 * 
 * absolute_height_layer [m] (step_up_h, drop_h)
 * 
 */

class MorphDropFilter : public filters::FilterBase<grid_map::GridMap> {
public:
    MorphDropFilter() = default;
    ~MorphDropFilter() override = default;

    bool configure() override;
    bool update(const grid_map::GridMap& in, grid_map::GridMap& out) override;


private:
    // Params
    std::string elev_               = "elevation";
    std::string step_layer_         = "step_up";
    std::string drop_layer_         = "drop";

    std::string step_h_layer_       = "step_up_h";  // [m]
    std::string drop_h_layer_       = "drop_h";     // [m]
    std::string step_over_layer_    = "step_over";  // {0, 1}
    std::string drop_over_layer_    = "drop_over"; // {0, 1}
    
    // Output Switches
    bool output_metric_layers_      = true;     // step_hp_h / drop_h
    bool output_over_mask_          = true;     // step_over / drop_over
    bool output_cm_bins_            = true;     // *_cm
    bool output_normalized_         = false;    // step_up/drop (0..1)
    
    int cm_cap_                     = 0;        // 1cm binning upper [cm], 0: no use

    double window_radius_m_         = 0.50;     // [m] Window Radius 
    double h_max_step_m_            = 0.10;     // [m] step allowance
    double h_max_drop_m_            = 0.15;     // [m] drop allowance
    bool use_square_kernel_         = true;     // true: rectangular, false: circular


    // Inside Util
    static int clampOdd(int K);                 // 3, 5, 7
};

} // namespace pongbot_traversability_filters