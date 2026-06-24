import cupy as cp
import string

def cluster_kernel(
    width, 
    height,
    cos_thresh, 
    dist_thresh
):
    cluster_kernel = cp.ElementwiseKernel(
        in_params="raw U map, raw U update, raw U n",
        out_params="raw U label",
        preamble=string.Template(
            """
            __device__ int get_map_idx(int idx, int layer_n) 
            {
                const int layer         = ${width} * ${height};

                return layer * layer_n + idx;
            }

            __device__ float inner_product(float x1, float y1, float z1,
                                           float x2, float y2, float z2)
            {
                float product       = (x1 * x2 + y1 * y2 + z1 * z2);

                return product;
            }
            
            __device__ float cosine_similarity(float x1, float y1, float z1,
                                               float x2, float y2, float z2)
            {
                float product       = inner_product(x1, y1, z1, x2, y2, z2);

                float norm1         = sqrt(x1 * x1 + y1 * y1 + z1 * z1);
                float norm2         = sqrt(x2 * x2 + y2 * y2 + z2 * z2);            

                if((norm1 == 0)||(norm2 == 0))
                {
                    return 0;
                }
                else
                {
                    return product / (norm1 * norm2);
                }                
            }
            """
        ).substitute(
        width                                       = width,
        height                                      = height, 
        ),
        operation = string.Template(
            """
            U h                                     = map[get_map_idx(i, 0)];
            U map_update                            = update[get_map_idx(i, 0)];

            float nx                                = n[get_map_idx(i, 0)];
            float ny                                = n[get_map_idx(i, 1)];
            float nz                                = n[get_map_idx(i, 2)]; 

            if (map_update > 0) 
            {
                float value                         = fabs(cosine_similarity(nx, ny, nz, 0.0, 0.0, 1.0));

                if(value > ${cos_thresh})
                {
                    label[get_map_idx(i, 0)]        = 1.0;
                }
            }            
            """
        ).substitute(cos_thresh = cos_thresh, dist_thresh = dist_thresh),
        name="cluster_kernel",
    )
    return cluster_kernel

def inlier_mask_kernel(
    cos_thresh, 
    dist_thresh
):
    inlier_mask_kernel = cp.ElementwiseKernel(
        in_params="raw U p, raw U n, raw U a, raw U b, raw U c, raw U d",
        out_params="raw bool inlier_mask",
        preamble=string.Template(
            """
            __device__ float point_to_plane_distance(float x, float y, float z,
                                                     float a, float b, float c, float d)
            {
                float value       = sqrt(a * a + b * b + c * c);
                float dist        = fabs(a * x + b * y + c * z + d) / value;

                if(value == 0)
                {
                    dist            = 0;
                }

                return dist;
            }

            __device__ float inner_product(float x1, float y1, float z1,
                                           float x2, float y2, float z2)
            {
                float product       = (x1 * x2 + y1 * y2 + z1 * z2);

                return product;
            }

            __device__ float cosine_similarity(float x1, float y1, float z1,
                                               float x2, float y2, float z2)
            {
                float product       = inner_product(x1, y1, z1, x2, y2, z2);

                float norm1         = sqrt(x1 * x1 + y1 * y1 + z1 * z1);
                float norm2         = sqrt(x2 * x2 + y2 * y2 + z2 * z2);            

                if((norm1 == 0)||(norm2 == 0))
                {
                    return 0;
                }
                else
                {
                    return product / (norm1 * norm2);
                }

                
            }
            """
        ).substitute(),
        operation = string.Template(
            """
            float x         = p[i * 3 + 0];
            float y         = p[i * 3 + 1];
            float z         = p[i * 3 + 2];

            float nx        = n[i * 3 + 0];
            float ny        = n[i * 3 + 1];
            float nz        = n[i * 3 + 2];

            float dist      = point_to_plane_distance(x, y, z, a[i], b[i], c[i], d[i]);
            float value     = cosine_similarity(nx, ny, nz, a[i], b[i], c[i]);
            
            inlier_mask[i]  = (dist < ${dist_thresh}) && (fabs(value) > ${cos_thresh});
            """
        ).substitute(cos_thresh = cos_thresh, dist_thresh = dist_thresh),
        name="inlier_mask_kernel",
    )
    return inlier_mask_kernel
        
    