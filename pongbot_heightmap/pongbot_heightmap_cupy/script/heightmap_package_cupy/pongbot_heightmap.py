import os
from typing import List, Any, Tuple, Union

import numpy as np
import threading
import subprocess               # system command
import time

from heightmap_package_cupy.parameter import Parameter

from cupyx.scipy.ndimage import convolve
from cupyx.scipy.ndimage import grey_opening, grey_erosion

from heightmap_package_cupy.kernels import (
    add_points_kernel,
    error_counting_kernel,
    average_map_kernel,
    normal_estimation_kernel,
    erosion_filter_kernel,
    traversability_kernel,
    feasibility_kernel,
    edge_filter_kernel,
    esdf_kernel,
)

import cupy as cp



# print(np.__version__)
# if DownGrade (numpy), pip install numpy==1.22.4

xp                              = cp
pool                            = cp.cuda.MemoryPool(cp.cuda.malloc_managed)
cp.cuda.set_allocator(pool.malloc)

class HeightMap:
    """Core Height Map class."""

    def __init__(self, param: Parameter):
        """
        Args:
            param (heightmap_package_cupy.parameter.Parameter):
        """
        
        param.update()

        self.param                  = param
        self.data_type              = self.param.data_type
        self.resolution             = param.resolution
        self.center                 = xp.array([0, 0, 0], dtype=self.data_type)
        self.base_rotation          = xp.eye(3, dtype=self.data_type)
        self.map_length             = param.map_length

        self.cell_n                 = param.cell_n

        self.map_lock               = threading.Lock()

        self.layer_names = [
            "height",
            "variance",
            "validity",
            "traversability",
            "time",
            "steppable",
            "filter_map",
        ]

        # print Parameter
        print("\n resolution :", self.resolution)
        print("\n map_length :", self.map_length)
        print("\n cell_n :", self.cell_n)

        self.height_map             = xp.zeros((len(self.layer_names), self.cell_n, self.cell_n), dtype=self.data_type)

        # Surface Normal map
        self.normal_map             = xp.zeros((3, self.cell_n, self.cell_n), dtype=self.data_type)

        # Initial variance
        self.initial_variance       = param.initial_variance
        self.height_map[1]          += self.initial_variance
        # self.height_map[2]          += 1.0
        self.height_map[3]          += 1.0


        # Sobel gradient operator
        self.sobel_x                = cp.array([[-1, 0, 1],
                                                [-2, 0, 2],
                                                [-1, 0, 1]], dtype=self.data_type)

        self.sobel_y                = cp.array([[-1,-2,-1],
                                                [ 0, 0, 0],
                                                [ 1, 2, 1]], dtype=self.data_type)
    
        # Initial mean_error
        self.mean_error             = 0.0
        self.additive_mean_error    = 0.0

        # Use Kernels
        self.compile_kernels()
        

    def clear(self):
        """Reset all the layers of the elevation & the semantic map."""
        with self.map_lock:
            self.height_map        *= 0.0

            # Initial variance
            self.height_map[1]     += self.initial_variance

        self.mean_error             = 0.0
        self.additive_mean_error    = 0.0

    def get_position(self, position):
        """Return the position of the map center.
        """
        position[0][:]              = xp.asnumpy(self.center)
        # GPU -> CPU Data (X Y Z)

    def move_to(self, base_position, R):
        """Shift the map to an position and update the rotation."""       
        self.base_rotation          = xp.asarray(R, dtype=self.data_type)
        base_position               = xp.asarray(base_position)

        # print("=" * 50)
        # print("[Move To] Rotation Matrix (R):")
        # print(self.base_rotation)
        # print("[Move To] Target Position (x, y, z):")
        # print(position)

        # position from map to base (Inertia Frame)
        position_vector             = base_position - self.center
        position_pixel              = xp.around(position_vector[:2] / self.resolution)

        position_vector_x           = position_pixel[0] * self.resolution
        position_vector_y           = position_pixel[1] * self.resolution

        self.center[0]             += position_vector_x
        self.center[1]             += position_vector_y
        self.center[2]             += position_vector[2]

        self.shift_map_xy(-position_pixel)
        self.shift_map_z(-position_vector[2])

    def pad_value(self, x, shift_value, idx=None, value=0.0):
        """Create a padding of the map along x,y-axis according to amount that has shifted.
        """
        if idx is None:
            if shift_value[0] > 0:
                x[:, : shift_value[0], :] = value
            elif shift_value[0] < 0:
                x[:, shift_value[0] :, :] = value
            if shift_value[1] > 0:
                x[:, :, : shift_value[1]] = value
            elif shift_value[1] < 0:
                x[:, :, shift_value[1] :] = value
        else:
            if shift_value[0] > 0:
                x[idx, : shift_value[0], :] = value
            elif shift_value[0] < 0:
                x[idx, shift_value[0] :, :] = value
            if shift_value[1] > 0:
                x[idx, :, : shift_value[1]] = value
            elif shift_value[1] < 0:
                x[idx, :, shift_value[1] :] = value

    def shift_map_xy(self, delta_pixel):
        """Shift the map along the horizontal axes according to the input.
        """
        shift_value = delta_pixel.astype(cp.int32)

        if cp.abs(shift_value).sum() == 0:
            return
        with self.map_lock:
            self.height_map         = cp.roll(self.height_map, shift_value, axis=(1, 2))
            self.pad_value(self.height_map, shift_value, value=0.0)
            self.pad_value(self.height_map, shift_value, idx=1, value=self.initial_variance)

    def shift_map_z(self, delta_z):
        """Shift the relevant layers along the vertical axis.
        """
        with self.map_lock:
            # elevation
            self.height_map[0]      += delta_z

            # upper bound
            # self.elevation_map[5] += delta_z

    def compile_kernels(self):
        """Compile all kernels belonging to the height map."""
        """use GPU Kerel and initialize buffer (Cupy array)"""

        # create Cupy array (using GPU Memory) 
        self.new_map = cp.zeros(
            (self.height_map.shape[0], self.cell_n, self.cell_n),
            dtype=self.data_type,
        )

        self.map_updated            = cp.zeros((self.cell_n, self.cell_n), dtype=self.data_type)

        self.traversability_input   = cp.zeros((self.cell_n, self.cell_n), dtype=self.data_type)

        self.add_points_kernel = add_points_kernel(
            self.resolution,
            self.cell_n,
            self.cell_n,
            self.param.sensor_noise_weight,
            self.param.mahalanobis_threshold,
            self.param.outlier_variance,
            self.param.min_valid_distance,
            self.param.max_region_of_interest_z,
            self.param.min_region_of_interest_xy,
        )

        self.error_counting_kernel = error_counting_kernel(
            self.resolution,
            self.cell_n,
            self.cell_n,
            self.param.mahalanobis_threshold,
            self.param.traversability_inlier,
            self.param.min_valid_distance,
            self.param.max_region_of_interest_z,
            self.param.min_region_of_interest_xy,
        )

        self.average_map_kernel = average_map_kernel(
            self.cell_n, self.cell_n, self.param.max_variance, self.initial_variance
        )

        # normal map
        self.normal_estimation_kernel = normal_estimation_kernel(
            self.cell_n, self.cell_n, self.resolution
        )

        # traversability map
        self.traversability_kernel = traversability_kernel(
            self.cell_n, self.cell_n
        )

        # filtered map
        self.erosion_filter_kernel  = erosion_filter_kernel(self.cell_n, self.cell_n, self.param.erosion_size)
        self.erosion_mask_dummy     = cp.zeros((self.cell_n, self.cell_n), dtype=self.data_type)

        # self.dilation_filter_kernel = dilation_filter_kernel(self.cell_n, self.cell_n, self.param.dilation_size)
        self.dilation_map            = cp.zeros((self.cell_n, self.cell_n), dtype=self.data_type)

        # edge detection
        self.edge_filter_kernel    = edge_filter_kernel(
            self.cell_n, self.cell_n, self.param.edge_threshold
        )
        self.edge_mask      = cp.zeros((self.cell_n, self.cell_n), dtype=self.data_type)

        # edge margin
        self.esdf_kernel    = esdf_kernel(
            self.resolution, self.cell_n, self.cell_n, self.param.edge_safety_margin, self.param.edge_indicator
        )

        self.esdf_map       = cp.zeros((self.cell_n, self.cell_n), dtype=self.data_type)

        # feasibility
        self.feasibility_kernel= feasibility_kernel(
            self.cell_n, self.cell_n, self.param.feasibility_threshold, self.param.edge_indicator
        )

    def update_map_with_kernel(self, points_all, channels, R, t, position_noise, orientation_noise):
        """Update map with new measurement.
        """

        # initialize new_map (for measurement : new_map = 0 * new_map)
        self.new_map        *= 0.0
        self.map_updated    *= 0.0

        error               = cp.array([0.0], dtype=cp.float32)
        error_cnt           = cp.array([0], dtype=cp.float32)

        points              = points_all[:, :3]

        with self.map_lock:
            self.shift_translation_to_map_center(t)

            # Height Drfit Compensation
            self.error_counting_kernel(
                self.height_map,
                points,
                cp.array([0.0], dtype=self.data_type),
                cp.array([0.0], dtype=self.data_type),
                R,
                t,
                self.new_map,
                error,
                error_cnt,
                size=(points.shape[0]),
            )
            if(
                self.param.enable_drift_compensation
                and error_cnt > self.param.min_height_drift_cnt
            ):
                self.mean_error = error / error_cnt
                self.additive_mean_error += self.mean_error
                if np.abs(self.mean_error) < self.param.max_drift:
                    self.height_map[0] += self.mean_error * self.param.enable_drift_compensation_alpha
            
                # print(" \n error_cnt")
                # print(error_cnt)
                # print(" \n mean_error")
                # print(self.mean_error)
                # print(" \n self.additive_mean_error")
                # print(self.additive_mean_error)
                # print(" \n self.param.max_drift")
                # print(self.param.max_drift)
                # print(" \n self.add ")
                # print(self.mean_error * self.param.enable_drift_compensation_alpha)

            self.add_points_kernel(
                cp.array([0.0], dtype=self.data_type),
                cp.array([0.0], dtype=self.data_type),
                R,
                t,
                self.normal_map,
                points,
                self.height_map,
                self.new_map,
                self.map_updated,
                size=(points.shape[0]),
            )

            self.average_map_kernel(self.new_map, self.height_map, size=(self.cell_n * self.cell_n))
            
            self.erosion_mask_dummy *= 0.0

            self.erosion_filter_kernel(
                self.height_map[0],
                self.height_map[2],
                self.height_map[6],
                self.erosion_mask_dummy,
                size=(self.cell_n * self.cell_n)
            )

            self.update_traversability(self.height_map[6], self.height_map[2])

            self.normal_estimation_kernel(
                self.height_map[6],
                self.map_updated,
                self.height_map[2],
                self.normal_map,
                size=(self.cell_n * self.cell_n),
            )

            self.edge_filter_kernel(
                self.height_map[3],
                self.edge_mask,
                size=(self.cell_n * self.cell_n),
            )

            self.esdf_kernel(
                self.height_map,
                self.edge_mask,
                self.esdf_map,
                size=(self.cell_n * self.cell_n),
            )

            # self.feasibility_map = cp.where(
            #     ((self.height_map[3] < self.param.feasibility_threshold) & (self.esdf_map >= self.param.edge_indicator)),
            #     1,
            #     0
            # )

            self.feasibility_kernel(
                self.height_map[3],
                self.esdf_map,
                self.height_map[5],
                size=(self.cell_n * self.cell_n),
            )

    def update_variance(self):
        """Adds the time variance to the valid cells."""
        self.height_map[1] += self.param.time_variance * self.height_map[2]

    def update_time(self):
        """adds the time interval to the time layer."""
        self.height_map[4] += self.param.time_interval

    def input_pointcloud(
        self,
        raw_points: cp._core.core.ndarray,
        channels: List[str],
        R: cp._core.core.ndarray,
        t: cp._core.core.ndarray,
        position_noise: float,
        orientation_noise: float,
    ):
        """Input the point cloud and fuse the new measurements to update the elevation map.

        Args:
            raw_points (cupy._core.core.ndarray):
            channels (List[str]):
            R  (cupy._core.core.ndarray):
            t (cupy._core.core.ndarray):
            position_noise (float):
            orientation_noise (float):

        Returns:
            None:
        """
        raw_points              = cp.asarray(raw_points, dtype=self.data_type)
        additional_channels     = channels[3:]
        raw_points              = raw_points[~cp.isnan(raw_points[:, :3]).any(axis=1)]
        
        self.update_map_with_kernel(
            raw_points,
            additional_channels,
            cp.asarray(R, dtype=self.data_type),
            cp.asarray(t, dtype=self.data_type),
            position_noise,
            orientation_noise,
        )



    def shift_translation_to_map_center(self, t):
        """Deduct the map center to get the translation of a point w.r.t. the map center.
        """
        t -= self.center

    def update_traversability(self, input_map, input_mask):
        """Update gradient map with kernel.

        Args:
            input_map (cupy._core.core.ndarray)
        """        
        self.traversability_input *= 0.0

        gradient_x = convolve(input_map, self.sobel_x, mode='nearest')
        gradient_y = convolve(input_map, self.sobel_y, mode='nearest')

        self.traversability_kernel(
            gradient_x,
            gradient_y,
            input_mask,
            self.traversability_input,
            size=(self.cell_n * self.cell_n),
        )        
        # self.traversability_kernel(
        #     gradient_x,
        #     gradient_y,
        #     self.height_map[2],
        #     self.traversability_input,
        #     size=(self.cell_n * self.cell_n),
        # )

        self.height_map[3] = self.traversability_input       

    def process_map_for_publish(self, input_map, fill_nan = False, add_z = False, xp = cp):
        """Process the Map Data considering fill_nan

        Args:
            input_map (cupy._core.core.ndarray)
            fill_nan (bool)
            add_z (bool):
            xp (module):
        """ 
        m = input_map.copy()

        if fill_nan :
            m = xp.where(self.height_map[2] > 0.5, m, xp.nan)
        if add_z:
            m = m + self.center[2]
        return m[1:-1, 1:-1]

    def get_elevation(self):
        """Get the elevation layer.

        Returns:
            elevation layer

        """
        return self.process_map_for_publish(self.height_map[0], fill_nan=True, add_z=True)

    def get_variance(self):
        """Get the variance layer.

        Returns:
            variance layer
        """

        return self.process_map_for_publish(self.height_map[1], fill_nan=False, add_z=False)
    
    def get_traversability(self):
        """Get the traversability layer.

        Returns:
            traversability layer
        """    
        return self.process_map_for_publish(self.height_map[3], fill_nan=True, add_z=False)

    def get_valid(self):
        """Get the valid layer.

        Returns:
            valid layer

        """
        return self.process_map_for_publish(self.height_map[2], fill_nan=True, add_z=False)

    def get_filter_map(self):
        """Get the filtered layer.

        Returns:
            filtered layer

        """
        return self.process_map_for_publish(self.height_map[6], fill_nan=True, add_z=True)
        
    def get_feasibility_map(self):
        """Get the feasibility layer.

        Returns:
            feasibility layer

        """
        return self.process_map_for_publish(self.height_map[5], fill_nan=True, add_z=False)         

    def get_esdf_map(self):
        """Get the ESDF layer.

        Returns:
            ESDF layer

        """
        return self.process_map_for_publish(self.esdf_map, fill_nan=True, add_z=False)
    
    def copy_to_cpu(self, array, data, stream=None):
        """Transforms the data to float32 and if on gpu loads it to cpu.

        Args:
            array (cupy._core.core.ndarray):
            data (numpy.ndarray):
        """
        if type(array) == np.ndarray:
            data[...] = array.astype(np.float32)
        elif type(array) == cp.ndarray:
            if stream is not None:
                data[...] = cp.asnumpy(array.astype(np.float32), stream=stream)
            else:
                data[...] = cp.asnumpy(array.astype(np.float32))

    def exists_layer(self, name):
        """Check if the layer exists in elevation map or in the semantic map.

        Args:
            name (str): Layer name

        Returns:
            bool: Indicates if layer exists.
        """
        if name in self.layer_names:
            return True
        elif name in self.semantic_map.layer_names:
            return True
        elif name in self.plugin_manager.layer_names:
            return True
        else:
            return False

    def get_map_with_name(self, name, data):
        """Load a layer data.

        Args:
            name (str): Name of the layer.
            data (numpy.ndarray): Data structure that contains layer.

        """
        use_stream = True
        xp = cp

        with self.map_lock:
            if name == "elevation":
                m = self.get_elevation()
                use_stream = False
            elif name == "variance":
                m = self.get_variance()
            elif name == "traversability":
                m = self.get_traversability()
            elif name == "is_valid":
                m = self.get_valid()
            elif name == "filter_map":
                m = self.get_filter_map()                
            elif name == "esdf":
                m = self.get_esdf_map()  
            elif name == "feasibility":
                m = self.get_feasibility_map()  
            elif name == "normal_x":
                m = self.normal_map.copy()[0, 1:-1, 1:-1]
            elif name == "normal_y":
                m = self.normal_map.copy()[1, 1:-1, 1:-1]
            elif name == "normal_z":
                m = self.normal_map.copy()[2, 1:-1, 1:-1]
            else:
                print("Layer {} is empty".format(name))
                return
        
        # Because of kernel sequence(get_x_idx, get_y_idx)
        m = xp.flip(m, 0)
        m = xp.flip(m, 1)

        if use_stream:
            stream = cp.cuda.Stream(non_blocking=False)
        else:
            stream = None
        self.copy_to_cpu(m, data, stream=stream)

if __name__ == "__main__":
    print("pongbot_heightmap \n")

    # cudaA  = xp.random.rand(1, 3, 3)
    # shift=[0, 1] # > 0 : down, > 0 : right