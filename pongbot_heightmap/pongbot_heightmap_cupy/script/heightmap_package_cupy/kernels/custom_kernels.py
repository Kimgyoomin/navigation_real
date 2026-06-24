import cupy as cp
import string

# __device__ : CUDA device

def map_utils(
    resolution,
    width,
    height,
    sensor_noise_weight,
    min_valid_distance,
    max_region_of_interest_z,
    min_region_of_interest_xy,
):
    
    util_preamble = string.Template(
        """
        __device__ float16 clamp(float16 x, float16 min_x, float16 max_x) 
        {
            return max(min(x, max_x), min_x);
        }

        __device__ int get_x_idx(float16 x, float16 center) 
        {
            int i       = (x - center) / ${resolution} + 0.5 * ${width};
        
            return i;
        }

        __device__ int get_y_idx(float16 y, float16 center) 
        {
            int i       = (y - center) / ${resolution} + 0.5 * ${height};
            return i;
        }

        __device__ bool is_inside(int idx) 
        {

            int idx_x = idx / ${width};
            int idx_y = idx % ${width};

            if (idx_x == 0 || idx_x == ${width} - 1) 
            {
                return false;
            }

            if (idx_y == 0 || idx_y == ${height} - 1) 
            {
                return false;
            }
            return true;
        }

        __device__ int get_idx(float16 x, float16 y, float16 center_x, float16 center_y) 
        {
            int idx_x       = clamp(get_x_idx(x, center_x), 0, ${width} - 1);
            int idx_y       = clamp(get_y_idx(y, center_y), 0, ${height} - 1);
        
            return ${width} * idx_x + idx_y;
        }

        __device__ int get_map_idx(int idx, int layer_n) 
        {
            const int layer = ${width} * ${height};
        
            return layer * layer_n + idx;
        }

        __device__ float transform(float16 x, float16 y, float16 z,
                                   float16 r0, float16 r1, float16 r2, float16 t) 
        {
            return r0 * x + r1 * y + r2 * z + t;
        }

        __device__ float sensor_noise_z(float16 z)
        {
            return ${sensor_noise_weight} * z * z;
        }

        __device__ float point_distance(float16 x, float16 y, float16 z,
                                        float16 frame_x, float16 frame_y, float16 frame_z) 
        {
            float d  = (x - frame_x) * (x - frame_x) + (y - frame_y) * (y - frame_y) + (z - frame_z) * (z - frame_z);

            return d;
        }

        __device__ float inner_product(float16 x1, float16 y1, float16 z1,
                                       float16 x2, float16 y2, float16 z2) 
        {
            float product   = (x1 * x2 + y1 * y2 + z1 * z2);

            return product;
        }

        __device__ bool is_valid(float16 x, float16 y, float16 z,
                                 float16 frame_x, float16 frame_y, float16 frame_z)
        {
            float d         = point_distance(x, y, z, frame_x, frame_y, frame_z);
            
            // TEST
            float dxy       = max(0.0, sqrt(x * x + y * y));
            float dz       = max(0.0, sqrt((z-frame_z)*(z-frame_z)));
            
            if (d < ${min_valid_distance}) 
            {
                return false;
            }
            else if (dxy < ${min_region_of_interest_xy})  // TEST 
            {
                return false;
            }
            else if (dz > ${max_region_of_interest_z}) 
            {
                return false;
            }
            else 
            {
                return true;
            }
        }
       """

    ).substitute(
        resolution                  = resolution,
        width                       = width,
        height                      = height,
        sensor_noise_weight         = sensor_noise_weight,
        min_valid_distance          = min_valid_distance,
        max_region_of_interest_z    = max_region_of_interest_z,
        min_region_of_interest_xy   = min_region_of_interest_xy,
    )
    return util_preamble

def add_points_kernel(
    resolution,
    width,
    height,
    sensor_noise_weight,
    mahalanobis_threshold,
    outlier_variance,
    min_valid_distance,
    max_region_of_interest_z,
    min_region_of_interest_xy,
):
    add_points_kernel = cp.ElementwiseKernel(
        in_params="raw U center_x, raw U center_y, raw U R, raw U t, raw U norm_map",
        out_params="raw U p, raw U map, raw U newmap, raw U update",
        preamble=map_utils(
            resolution,
            width,
            height,
            sensor_noise_weight,
            min_valid_distance,
            max_region_of_interest_z,
            min_region_of_interest_xy,
        ),
        operation=string.Template(
            """
            U rx                                = p[i * 3];
            U ry                                = p[i * 3 + 1];
            U rz                                = p[i * 3 + 2];

            U x                                 = transform(rx, ry, rz, R[0], R[1], R[2], t[0]);
            U y                                 = transform(rx, ry, rz, R[3], R[4], R[5], t[1]);
            U z                                 = transform(rx, ry, rz, R[6], R[7], R[8], t[2]);
            U point_variance                    = sensor_noise_z(rz);            

            int idx                             = get_idx(x, y, center_x[0], center_y[0]);
        
            if (is_valid(x, y, z, t[0], t[1], t[2])) 
            {
                if (is_inside(idx)) 
                {
                    U map_height                    = map[get_map_idx(idx, 0)];
                    U map_variance                  = map[get_map_idx(idx, 1)];
                    
                    U num_points                    = newmap[get_map_idx(idx, 4)];                    

                    // if (abs(map_height - z) > (map_variance * ${mahalanobis_threshold})) 
                    if (abs(map_height - z)/sqrt(map_variance) > ${mahalanobis_threshold}) 
                    {
                        atomicAdd(&map[get_map_idx(idx, 1)], ${outlier_variance});
                    }
                    else 
                    {
                        // if ((z < map_height - map_variance * ${mahalanobis_threshold} / num_points)) {

                        U new_map_height                = (point_variance * map_height + map_variance * z) / (map_variance + point_variance);
                        U new_map_variance              = (map_variance * point_variance) / (map_variance + point_variance);

                        atomicAdd(&newmap[get_map_idx(idx, 0)], new_map_height);
                        atomicAdd(&newmap[get_map_idx(idx, 1)], new_map_variance);
                        atomicAdd(&newmap[get_map_idx(idx, 2)], 1.0);               // add point number        

                        // is Valid
                        map[get_map_idx(idx, 2)] = 1;

                        // Update
                        update[get_map_idx(idx, 0)] = 1;
                    }
                }
            }
            """
        ).substitute(
            mahalanobis_threshold=mahalanobis_threshold,
            outlier_variance=outlier_variance,
        ),
        name="add_points_kernel",
    )
    return add_points_kernel

def error_counting_kernel(
    resolution,
    width,
    height,
    mahalanobis_threshold,
    traversability_inlier,
    min_valid_distance,
    max_region_of_interest_z,
    min_region_of_interest_xy,
):
    error_counting_kernel = cp.ElementwiseKernel(
        in_params="raw U map, raw U p, raw U center_x, raw U center_y, raw U R, raw U t",
        out_params="raw U newmap, raw T error, raw T error_cnt",
        preamble=map_utils(
            resolution,
            width,
            height,
            mahalanobis_threshold,
            min_valid_distance,
            max_region_of_interest_z,
            min_region_of_interest_xy,
        ),
        operation=string.Template(
            """
            U rx                                = p[i * 3];
            U ry                                = p[i * 3 + 1];
            U rz                                = p[i * 3 + 2];

            U x                                 = transform(rx, ry, rz, R[0], R[1], R[2], t[0]);
            U y                                 = transform(rx, ry, rz, R[3], R[4], R[5], t[1]);
            U z                                 = transform(rx, ry, rz, R[6], R[7], R[8], t[2]);
            U point_variance                    = sensor_noise_z(rz);            

            if (!is_valid(x, y, z, t[0], t[1], t[2])) 
            {
                return;
            }

            int idx                             = get_idx(x, y, center_x[0], center_y[0]);

            if(!is_inside(idx))
            {
                return;
            }

            U map_height                        = map[get_map_idx(idx, 0)];
            U map_variance                      = map[get_map_idx(idx, 1)];
            U valid                             = map[get_map_idx(idx, 2)];
            U traversability                    = map[get_map_idx(idx, 3)];

            if ((valid > 0.5) &&
                (abs(map_height - z) / sqrt(map_variance) < ${mahalanobis_threshold}) &&
                (traversability < ${traversability_inlier}))
            {
                T e = z - map_height;
                atomicAdd(&error[0], e);
                atomicAdd(&error_cnt[0], 1);
            }

            // (traversability < ${traversability_inlier}))
            """
        ).substitute(
            mahalanobis_threshold=mahalanobis_threshold,
            traversability_inlier=traversability_inlier,
        ),
        name="error_counting_kernel",
    )
    return error_counting_kernel


def average_map_kernel(width, height, max_variance, initial_variance):
    average_map_kernel = cp.ElementwiseKernel(
        in_params   ="raw U newmap",
        out_params  ="raw U map",
        preamble    =string.Template(
            """
            __device__ int get_map_idx(int idx, int layer_n) 
            {
                const int layer = ${width} * ${height};
                return layer * layer_n + idx;
            }
            """
        ).substitute(width=width, height=height),
        operation=string.Template(
            """
            U new_map_height                    = newmap[get_map_idx(i, 0)];
            U new_map_variance                  = newmap[get_map_idx(i, 1)];
            U new_map_point_cnt                 = newmap[get_map_idx(i, 2)];

            if(new_map_point_cnt > 0)   
            {
                if((new_map_variance / new_map_point_cnt) > ${max_variance})
                {
                    map[get_map_idx(i, 0)]    = 0;
                    map[get_map_idx(i, 1)]    = ${initial_variance};
                    map[get_map_idx(i, 2)]    = 0;
                }
                else
                {
                    map[get_map_idx(i, 0)]    = new_map_height   / new_map_point_cnt;
                    map[get_map_idx(i, 1)]    = new_map_variance / new_map_point_cnt;
                    map[get_map_idx(i, 2)]    = 1;
                }   
            }


            """
        ).substitute(max_variance=max_variance, initial_variance=initial_variance),
        name="average_map_kernel",
    )                                                                                                                   
    return average_map_kernel

def normal_estimation_kernel(width, height, resolution):
    normal_estimation_kernel = cp.ElementwiseKernel(
        in_params   ="raw U map, raw U update, raw U mask",
        out_params  ="raw U newmap",
        preamble    =string.Template(
            """
            __device__ int get_map_idx(int idx, int layer_n) 
            {
                const int layer             = ${width} * ${height};
                
                return layer * layer_n + idx;
            }

            __device__ int get_neighbor_map_idx(int idx, int delta_x, int delta_y, int layer_n) 
            {
                const int layer             = ${width} * ${height};
                const int neighbor_idx      = idx + ${width} * delta_y + delta_x;

                return layer * layer_n + neighbor_idx;
            }
            
            __device__ bool is_inside(int idx) 
            {
                int idx_x = idx / ${width};
                int idx_y = idx % ${width};

                if(idx_x <= 0 || idx_x >= ${width} - 1)
                {
                    return false;
                }

                if(idx_y <= 0 || idx_y >= ${height} - 1)
                {
                    return false;
                }
                
                return true;
            }

            __device__ float get_resolution() 
            {
                return ${resolution};
            }

            """
        ).substitute(
        width                               = width,
        height                              = height,                                                                 
        resolution                          = resolution,
        ),
        operation=string.Template(
            """
            U map_height                    = map[get_map_idx(i, 0)];
            U valid                         = mask[get_map_idx(i, 0)];
            
            U map_update                    = update[get_map_idx(i, 0)];

            if((valid > 0.5) &&  (map_update > 0))
            {
                int idx_x                   = get_neighbor_map_idx(i, 1, 0, 0);
                int idx_y                   = get_neighbor_map_idx(i, 0, 1, 0);

                if(is_inside(idx_x) == false || is_inside(idx_y) == false)
                {
                    return;
                }

                float dzdx                  = map[idx_x] - map_height;
                float dzdy                  = map[idx_y] - map_height;

                float nx                    = -dzdy / get_resolution();
                float ny                    = -dzdx / get_resolution();
                float nz                    = 1;

                float norm                  = sqrt((nx * nx) + (ny * ny) + 1);
                newmap[get_map_idx(i, 0)]   = nx / norm;
                newmap[get_map_idx(i, 1)]   = ny / norm;
                newmap[get_map_idx(i, 2)]   = nz / norm;

            }
            """
        ).substitute(),
        name="normal_estimation_kernel",
    )
    return normal_estimation_kernel


def erosion_filter_kernel(width, height, erosion_size):
    erosion_kernel = cp.ElementwiseKernel(
        in_params="raw U map, raw U mask",
        out_params="raw U newmap, raw U newmask",
        preamble=string.Template(
            """
            __device__ int get_map_idx(int idx, int layer_n) 
            {
                const int layer         = ${width} * ${height};
                return layer * layer_n + idx;
            }

            __device__ int get_neighbor_map_idx(int idx, int delta_x, int delta_y, int layer_n) 
            {
                const int layer         = ${width} * ${height};
                const int neighbor_idx  = idx + ${width} * delta_y + delta_x;
                return layer * layer_n + neighbor_idx;
            }

            __device__ bool is_inside(int idx) 
            {
                int idx_x               = idx / ${width};
                int idx_y               = idx % ${width};

                if (idx_x <= 0 || idx_x >= ${width} - 1)
                {
                    return false;
                }

                if (idx_y <= 0 || idx_y >= ${height} - 1)
                {
                    return false;
                } 
                return true;
            }
            """
        ).substitute(
        width                                   = width,
        height                                  = height, 
        ),
        operation=string.Template(
            """
            U h                                 = map[get_map_idx(i, 0)];
            U valid                             = mask[get_map_idx(i, 0)];

            newmap[get_map_idx(i, 0)]           = h;
            newmask[get_map_idx(i, 0)]          = valid;
            
            if (valid > 0.5) 
            {
                U min_value                    = 1000;

                for (int dy = -${erosion_size}; dy <= ${erosion_size}; dy++) 
                {
                    for (int dx = -${erosion_size}; dx <= ${erosion_size}; dx++) 
                    {
                        if (dx == -${erosion_size} && dy == -${erosion_size}) continue;
                        else if (dx == -${erosion_size} && dy ==  ${erosion_size}) continue;
                        else if (dx ==  ${erosion_size} && dy == -${erosion_size}) continue;
                        else if (dx ==  ${erosion_size} && dy ==  ${erosion_size}) continue;

                        int neighbor_idx        = get_neighbor_map_idx(i, dx, dy, 0);

                        if (!is_inside(neighbor_idx)) continue;

                        U neighbor_valid    = mask[neighbor_idx];

                        if (neighbor_valid > 0.5) 
                        {
                            U value         = map[neighbor_idx];

                            if (value < min_value) 
                            {
                                min_value   = value;
                            }
                        }
                        else
                        {
                            newmask[get_map_idx(i, 0)]  = 0.0;
                        }
                    }
                }
                newmap[get_map_idx(i, 0)]   = min_value;
                
            }
            """
        ).substitute(erosion_size=erosion_size),
        name="erosion_filter_kernel",
    )
    return erosion_kernel

def traversability_kernel(width, height):
    traversability_kernel = cp.ElementwiseKernel(
        in_params="raw U gradient_x, raw U gradient_y, raw U mask",
        out_params="raw U newmap",
        preamble=string.Template(
            """
            __device__ int get_map_idx(int idx, int layer_n) 
            {
                const int layer         = ${width} * ${height};

                return layer * layer_n + idx;
            }

            __device__ float get_traversability(float16 gx, float16 gy) 
            {
                float traversability    = abs(atan(sqrt(gx * gx + gy * gy)));

                return traversability;
            }

            __device__ bool is_inside(int idx) 
            {
                int idx_x               = idx / ${width};
                int idx_y               = idx % ${width};

                if (idx_x <= 0 || idx_x >= ${width} - 1)
                {
                    return false;
                }

                if (idx_y <= 0 || idx_y >= ${height} - 1)
                {
                    return false;
                } 

                return true;
            }
            """
        ).substitute(
        width                                   = width,
        height                                  = height, 
        ),
        operation=string.Template(
            """
            U gx                                = gradient_x[get_map_idx(i, 0)];
            U gy                                = gradient_y[get_map_idx(i, 0)];

            U valid                             = mask[get_map_idx(i, 0)];

            if (valid > 0.5) 
            {
                U traversability                = get_traversability(gx, gy);

                newmap[get_map_idx(i, 0)]       = traversability;
            }
            """
        ).substitute(),
        name="traversability_kernel",
    )
    return traversability_kernel

def feasibility_kernel(width, height, feasibility_threshold, edge_indicator):
    feasibility_kernel = cp.ElementwiseKernel(
        in_params="raw U map, raw U mask",
        out_params="raw U newmap",
        preamble=string.Template(
            """
            __device__ int get_map_idx(int idx, int layer_n) 
            {
                const int layer         = ${width} * ${height};

                return layer * layer_n + idx;
            }
            """
        ).substitute(
        width                                   = width,
        height                                  = height, 
        ),
        operation=string.Template(
            """
            U traversability                    = map[get_map_idx(i, 0)];
            U edge                              = mask[get_map_idx(i, 0)];

            if ((traversability < ${feasibility_threshold}) && (edge >= ${edge_indicator})) 
            {
                newmap[get_map_idx(i, 0)]       = 1.0;
            }
            else
            {
                newmap[get_map_idx(i, 0)]       = 0.0;
            }
            """
        ).substitute(feasibility_threshold=feasibility_threshold,
                     edge_indicator = edge_indicator
                     ),
        name="feasibility_kernel",
    )
    return feasibility_kernel

def edge_filter_kernel(width, height, edge_threshold):
    edge_filter_kernel = cp.ElementwiseKernel(
        in_params="raw U map",
        out_params="raw U newmask",
        preamble=string.Template(
            """
            __device__ int get_map_idx(int idx, int layer_n) 
            {
                const int layer         = ${width} * ${height};

                return layer * layer_n + idx;
            }
            """
        ).substitute(
        width                                   = width,
        height                                  = height, 
        ),
        operation=string.Template(
            """
            U traversability                    = map[get_map_idx(i, 0)];

            if (traversability > ${edge_threshold}) 
            {
                newmask[get_map_idx(i, 0)]       = 1.0;
            }
            else
            {
                newmask[get_map_idx(i, 0)]       = 0.0;
            }
            """
        ).substitute(edge_threshold=edge_threshold),
        name="edge_filter_kernel",
    )
    return edge_filter_kernel

def esdf_kernel(resolution, width, height, esdf_size, indicator):
    esdf_kernel = cp.ElementwiseKernel(
        in_params   ="raw U map, raw U mask",
        out_params  ="raw U newmap",
        preamble    =string.Template(
            """
            __device__ int get_map_idx(int idx, int layer_n) 
            {
                const int layer             = ${width} * ${height};
                
                return layer * layer_n + idx;
            }

            __device__ int get_neighbor_map_idx(int idx, int delta_x, int delta_y, int layer_n) 
            {
                const int layer             = ${width} * ${height};
                const int neighbor_idx      = idx + ${width} * delta_y + delta_x;

                return layer * layer_n + neighbor_idx;
            }
            
            __device__ bool is_inside(int idx) 
            {
                int idx_x = idx / ${width};
                int idx_y = idx % ${width};

                if(idx_x <= 0 || idx_x >= ${width} - 1)
                {
                    return false;
                }

                if(idx_y <= 0 || idx_y >= ${height} - 1)
                {
                    return false;
                }
                
                return true;
            }

            __device__ float get_resolution() 
            {
                return ${resolution};
            }

            """
        ).substitute(
        width                               = width,
        height                              = height,                                                                 
        resolution                          = resolution,
        ),
        operation=string.Template(
            """
            U map_height                    = map[get_map_idx(i, 0)];
            U edge                          = mask[get_map_idx(i, 0)];

            float min_value                 = ${indicator};

            for (int dy = -${esdf_size}; dy <= ${esdf_size}; ++dy) 
            {
                for (int dx = -${esdf_size}; dx <= ${esdf_size}; ++dx) 
                {
                    int neighbor_idx        = get_neighbor_map_idx(i, dx, dy, 0);

                    if (!is_inside(neighbor_idx)) continue;

                    float delta_x       = abs(dx) * get_resolution();
                    float delta_y       = abs(dy) * get_resolution();
                    float delta_z       = map_height - map[get_map_idx(neighbor_idx, 0)];
                    
                    float d = delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;

                    if(mask[get_map_idx(neighbor_idx, 0)] < 1)
                    {
                        d = d + ${indicator};
                    }

                    if (d < min_value) 
                    {
                        min_value      = d;
                    }
                }
            }
            

            newmap[get_map_idx(i, 0)]   = min_value;
            """
        ).substitute(
            esdf_size       = esdf_size,
            indicator       = indicator,
            ),
        name="esdf_kernel",
    )
    return esdf_kernel

if __name__ == "__main__":
    print("kernel")
    