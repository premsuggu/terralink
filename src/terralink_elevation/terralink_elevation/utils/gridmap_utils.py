"""GridMap message encoding utilities."""
import numpy as np
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from terralink_elevation.elevation_map import ElevationMapCPU
from terralink_elevation.elevation_map_gpu import ElevationMapGPU
from terralink_elevation.parameter import Parameter
from rclpy.time import Time
from typing import Union


ElevationMap = Union[ElevationMapCPU, ElevationMapGPU]


def internal_to_gridmap(arr: np.ndarray) -> np.ndarray:
    """Convert internal (row=Y, col=X) to GridMap column-major convention.
    
    GridMap: column-major, Row->-X, Col->-Y
    Internal: row=Y, col=X
    """
    # Transpose: (H, W) -> (W, H) [now rows=X, cols=Y]
    arr = arr.T
    # Flip axis 0: row 0 becomes last row (X direction)
    arr = np.flip(arr, axis=0)
    # Flip axis 1: col 0 becomes last col (Y direction)
    arr = np.flip(arr, axis=1)
    return arr


def gridmap_to_internal(arr: np.ndarray) -> np.ndarray:
    """Convert GridMap column-major to internal row-major."""
    # Reverse: flip Y, flip X, transpose
    arr = np.flip(arr, axis=1)
    arr = np.flip(arr, axis=0)
    return arr.T


def layer_to_multiarray(layer: np.ndarray) -> Float32MultiArray:
    """Convert 2D layer to Float32MultiArray (column-major)."""
    H, W = layer.shape
    
    # Convert to GridMap convention
    gm_layer = internal_to_gridmap(layer)
    
    # Flatten column-major
    flat = gm_layer.ravel().astype(np.float32)
    
    msg = Float32MultiArray()
    msg.layout = MultiArrayLayout()
    msg.layout.dim = [
        MultiArrayDimension(label='column_index', size=W, stride=W*H),
        MultiArrayDimension(label='row_index', size=H, stride=H),
    ]
    msg.data = flat.tolist()
    return msg


def _extract_layer(elev_map: Union[ElevationMapCPU, ElevationMapGPU], 
                   layer_name: str, 
                   slice_: slice) -> np.ndarray:
    """Extract layer from elevation map (handles both CPU and GPU)."""
    layer_idx = elev_map.layer_names.index(layer_name)
    layer_data = elev_map.elevation_map[layer_idx, slice_, slice_]
    # Convert to numpy if GPU
    if hasattr(layer_data, 'get'):
        return layer_data.get()
    return layer_data


def elevation_map_to_gridmap(
    elev_map: Union[ElevationMapCPU, ElevationMapGPU], 
    param: Parameter, 
    stamp: Time, 
    frame_id: str = 'map'
) -> GridMap:
    """Convert ElevationMap (CPU or GPU) to GridMap message."""
    H, W = elev_map.true_cell_n, elev_map.true_cell_n
    
    # Extract valid region (exclude border)
    border = (elev_map.cell_n - elev_map.true_cell_n) // 2
    slice_ = slice(border, border + elev_map.true_cell_n)
    
    msg = GridMap()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    
    # Geometry
    msg.info.resolution = param.resolution
    msg.info.length_x = param.true_map_length
    msg.info.length_y = param.true_map_length
    msg.info.pose.position.x = elev_map.center_x
    msg.info.pose.position.y = elev_map.center_y
    msg.info.pose.position.z = 0.0
    msg.info.pose.orientation.w = 1.0
    
    # Layers to publish
    msg.layers = ['elevation', 'variance', 'is_valid', 'traversability']
    msg.basic_layers = ['elevation']
    
    # Extract and convert each layer
    for layer_name in msg.layers:
        layer_data = _extract_layer(elev_map, layer_name, slice_)
        msg.data.append(layer_to_multiarray(layer_data))
    
    return msg