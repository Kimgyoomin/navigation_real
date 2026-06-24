# heightmap_package_cupy/plane_extraction.py

import cupy as cp
import numpy as np

class PlaneSegment:
    """Plane segmentation."""
    
    def set_parameter(self, iterations, distance_thresh, normal_thresh, min_pointcnt, cell_n, resolution, data_type):
        self.iterations             = iterations
        self.min_pointcnt           = min_pointcnt
        self.normal_thresh          = normal_thresh
        self.distance_thresh        = distance_thresh
        self.cell_n                 = cell_n
        self.resolution             = resolution
        self.data_type              = data_type

    def ransac_plane(self, point_map, normal_map):
        best_inlier_cnt             = 0
        best_model                  = None
        N                           = point_map.shape[0]

        self.sample_indices         = cp.asarray([
            np.random.choice(N, 3, replace=False) for _ in range(self.iterations)
        ])
        
        for i in range(self.iterations):
            idx                     = self.sample_indices[i]
            p1, p2, p3              = point_map[idx]

            v1, v2                  = p2 - p1, p3 - p1

            x1, y1, z1              = v1
            x2, y2, z2              = v2

            cross_product           = cp.array([
                                        y1 * z2 - z1 * y2,
                                        z1 * x2 - x1 * z2,
                                        x1 * y2 - y1 * x2
                                                ], dtype=cp.float32)

            norm                    = cp.sqrt(cross_product[0] ** 2 + cross_product[1] ** 2 + cross_product[2] ** 2)

            if norm < 1e-6:
                continue
                
            normal                  = cross_product / norm
            
            a, b, c                 = normal
            d                       = -cp.dot(normal, p1)
            
            
            inlier_mask             = cp.zeros(N, dtype=cp.bool_)

            a_array                 = cp.full(N, a, dtype=cp.float32)
            b_array                 = cp.full(N, b, dtype=cp.float32)
            c_array                 = cp.full(N, c, dtype=cp.float32)
            d_array                 = cp.full(N, d, dtype=cp.float32)
            
            self.inlier_mask_kernel(
                point_map.ravel(), 
                normal_map.ravel(),
                a_array, 
                b_array, 
                c_array,
                d_array,
                inlier_mask,
                size=N
            )

            inlier_count            = cp.count_nonzero(inlier_mask) 

            if inlier_count > best_inlier_cnt:
                best_inlier_cnt     = inlier_count
                best_model          = (normal.get(), inlier_mask)

        return best_model



    def extract_planes(self, center, height_map, update, normal_map, label_map):
        
        self.cluster_kernel(
            height_map, 
            update,
            normal_map, 
            label_map,
            size=(self.cell_n * self.cell_n)
        )
     
        # Plane Segmenation
        plane_valid_indices         = cp.argwhere(label_map > 0.5) 
        
        i_idx                       = plane_valid_indices[:, 0]
        j_idx                       = plane_valid_indices[:, 1]

        x                           = ((i_idx - self.cell_n // 2) * self.resolution + center[0]).astype(self.data_type)
        y                           = ((j_idx - self.cell_n // 2) * self.resolution + center[1]).astype(self.data_type)
        z                           = height_map[i_idx, j_idx]

        plane_point_map             = cp.stack([x, y, z], axis=1)
        nx                          = normal_map[0, i_idx, j_idx]
        ny                          = normal_map[1, i_idx, j_idx]
        nz                          = normal_map[2, i_idx, j_idx]
        plane_normal_map            = cp.stack([nx, ny, nz], axis=1)

        planes                      = []
        total_point                 = plane_point_map.shape[0]
        
        remain_mask                 = cp.ones(plane_point_map.shape[0], dtype=cp.bool_)

        while cp.count_nonzero(remain_mask) >= total_point * 0.1:
            current_points          = plane_point_map[remain_mask]
            current_normals         = plane_normal_map[remain_mask]

            model                   = self.ransac_plane(current_points, current_normals)

            if model is None:
                break
   
            normal, inlier_mask     = model

            inlier_count            = cp.count_nonzero(inlier_mask)


            if inlier_count         < self.min_pointcnt:
                break
            
            remain_idx              = cp.where(remain_mask)[0] 
            inlier_idx              = remain_idx[inlier_mask]

            planes.append(
                {
                "normal": normal,
                "inliers": plane_point_map[inlier_idx]
                }
            )
            remain_mask[inlier_idx] = False
 
        return planes


    # def extract_planes(self, point_map, normal_map):
    #     planes                      = []
    #     remaining_point             = point_map
    #     remaining_normal            = normal_map
    #     total_point                 = point_map.shape[0]

    #     while remaining_point.shape[0] >= total_point * 0.1:
    #         model                   = self.ransac_plane(remaining_point, remaining_normal)

    #         if model is None:
    #             break

    #         normal, inlier_mask     = model
    #         N                       = remaining_point.shape[0]

    #         inlier_count            = cp.sum(inlier_mask).get()

    #         if inlier_count         < self.min_pointcnt:
    #             break

    #         planes.append(
    #             {
    #             "normal": normal,
    #             "inliers": remaining_point[inlier_mask]
    #             }
    #         )

    #         remaining_point            = remaining_point[~inlier_mask]
    #         remaining_normal           = remaining_normal[~inlier_mask]

    #     return planes
