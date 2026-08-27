"""Coordinate transformation utilities."""
import numpy as np
from numpy.typing import NDArray
from tf_transformations import quaternion_matrix, quaternion_from_euler


def quat_to_matrix(q: NDArray) -> NDArray:
    """Quaternion [x, y, z, w] to 4x4 matrix."""
    return quaternion_matrix(q)


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> NDArray:
    """Roll, pitch, yaw to quaternion [x, y, z, w]."""
    return quaternion_from_euler(roll, pitch, yaw)


def transform_to_matrix(transform) -> tuple:
    """geometry_msgs/TransformStamped -> (R: 3x3, t: 3)."""
    t_msg = transform.transform.translation
    q_msg = transform.transform.rotation
    
    t = np.array([t_msg.x, t_msg.y, t_msg.z], dtype=np.float32)
    mat = quaternion_matrix([q_msg.x, q_msg.y, q_msg.z, q_msg.w])
    R = mat[:3, :3].astype(np.float32)
    
    return R, t


def points_sensor_to_map(points: NDArray, R: NDArray, t: NDArray) -> NDArray:
    """Transform points from sensor frame to map frame.
    
    Args:
        points: (N, 3) in sensor frame
        R: (3, 3) rotation
        t: (3,) translation
    Returns:
        (N, 3) in map frame
    """
    return (R @ points.T + t.reshape(3, 1)).T


def world_to_grid(x: float, y: float, center_x: float, center_y: float, 
                  resolution: float, cell_n: int) -> tuple:
    """World coordinates to grid indices (row, col)."""
    col = int(round((x - center_x) / resolution + cell_n / 2))
    row = int(round((y - center_y) / resolution + cell_n / 2))
    return row, col


def grid_to_world(row: int, col: int, center_x: float, center_y: float,
                  resolution: float, cell_n: int) -> tuple:
    """Grid indices to world coordinates."""
    x = (col - cell_n / 2) * resolution + center_x
    y = (row - cell_n / 2) * resolution + center_y
    return x, y