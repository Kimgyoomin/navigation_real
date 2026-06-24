from dataclasses import dataclass, field
import pickle
import numpy as np
from simple_parsing.helpers import Serializable
from dataclasses import field
from typing import Tuple


@dataclass
class Parameter(Serializable):
    """
    This class holds the parameters for the elevation mapping algorithm.
    
    Attributes:
        resolution: Map Resolution [meter].
                    (Default: ``0.04``)
        data_type: The data type for Map.  
                    (Default: ``np.float32``)
        map_length: Map Size [meter]  
                    (Default: ``4.0``)
        sensor_noise_weight: The sensor_noise_weight.  
                    (Default: ``0.05``)
        mahalanobis_threshold: Points outside this value is outlier.
                    (Default: ``1.0``)
        initial_variance: Initial variance for Map Cell.  
                    (Default: ``10.0``)
    """
    resolution: float                       = 0.02          # 0.04 # resolution in meter.
    # subscriber_config: dict             = field(
    #     default_factory=lambda: 
    #     {
    #         "front_cam": 
    #         {
    #             "channels": ["rgb", "person"],
    #             "topic_name": "/elevation_mapping/pointcloud_semantic",
    #             "data_type": "pointcloud",
    #         }
    #     }
    # )
    data_type: str                          = np.float32    # data type for the map

    map_length: float                       = 2.4           # map's size in m.
    sensor_noise_weight: float              = 0.05          # point's noise is sensor_noise_factor*z^2 (z is distance from sensor).
    mahalanobis_threshold: float            = 1.0           # 2.0 points outside this distance is outlier.
    outlier_variance: float                 = 0.01          # if point is outlier, add this value to the cell.

    max_variance: float                     = 1.0           # maximum variance for each cell.
    initial_variance: float                 = 10.0          # initial variance for each cell.
    
    # traversability
    dilation_size: int                      = 2             # dilation filter size
    erosion_size: int                       = 1             # erosion filter size
    
    # ROI
    min_valid_distance: float               = 0.05          # 0.3  # points with shorter distance will be filtered out.
    max_region_of_interest_z: float         = 0.8           # 1.0 # points higher than this value from sensor will be filtered out to disable ceiling.
    min_region_of_interest_xy: float        = 0.8           # 0.8 # 0.5 // 1.0  # if z > max(d - ramped_height_range_b, 0).

    # edge detection
    edge_threshold: float                   = 0.2

    # euclidean distance field
    edge_safety_margin: int                 = 2             # 1
    edge_indicator: float                   = 1000.

    # not configurable params
    true_map_length: float                  = None      # true length of the map
    cell_n: int                             = None      # number of cells in the map
    true_cell_n: int                        = None      # true number of cells in the map

    # plane
    plane_iterations: int                   = 100       # 300
    plane_distance_thresh: float            = 0.02
    plane_normal_thresh: float              = 10.0      # 25
    plane_min_pointcnt: int                 = 50

    # feasibility
    feasibility_threshold: float            = 0.1

    # height drift compensation
    max_drift: float                        = 0.1
    enable_drift_compensation: bool         = False  # enable drift compensation
    min_height_drift_cnt: float             = 100
    enable_drift_compensation_alpha: float  = 0.1
    traversability_inlier: float            = 0.3   # 0.9

    # Function
    def get_names(self):
        """
        Get the names of the parameters.2
        
        Returns:
            list: A list of parameter names.
        """
        return list(self.__annotations__.keys())

    def get_types(self):
        """
        Get the types of the parameters.
        
        Returns:
            list: A list of parameter types.
        """
        return [v.__name__ for v in self.__annotations__.values()] 
        # Comment : self.__annotations__ : dictionary  / .values() : type (ex. int, float, string)

    def set_value(self, name, value):
        """
        Set the value of a parameter.
        
        Args:
            name (str): The name of the parameter.
            value (any): The new value for the parameter.
        """
        setattr(self, name, value)


    def get_value(self, name):
        """
        Get the value of a parameter.
        
        Args:
            name (str): The name of the parameter.
        
        Returns:
            any: The value of the parameter.
        """
        return getattr(self, name)

    def update(self):
        """
        Update the parameters related to the map size and resolution.
        """
        # +2 is a border for outside map
        self.cell_n = int(round(self.map_length / self.resolution)) + 2
        self.true_cell_n = round(self.map_length / self.resolution)
        self.true_map_length = self.true_cell_n * self.resolution


if __name__ == "__main__":
    # TEST
    param = Parameter()
    print("\n ")
    print(param)
    print("\n resolution : \n ", param.resolution)

    param.set_value("resolution", 0.1)
    print("\n resolution : \n ", param.resolution)
    print("\n values : \n ", param.get_value("mahalanobis_thresh"))

    print("\n names  : \n ", param.get_names())
    print("\n  types  : \n ", param.get_types())
