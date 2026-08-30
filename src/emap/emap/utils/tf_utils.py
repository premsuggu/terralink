"""Small helpers for turning a `geometry_msgs/TransformStamped` (what
`tf2_ros`'s `Buffer.lookup_transform` returns) into plain NumPy math.

Why this exists instead of using `tf2_sensor_msgs.do_transform_cloud`:
step 2's verification found that helper raises an `AssertionError` on this
project's own `PointCloud2` layout (a dtype mismatch inside
`sensor_msgs_py.point_cloud2.create_cloud`, unrelated to anything we're doing
wrong - see docs/work-docs/emap/step02_depth_camera_pointcloud.md). Plain
matrix math on the `(N, 3)` array we already get from
`point_cloud2.read_points_numpy` sidesteps that library bug entirely, and is
exactly what step 2's write-up already verified against real, known ground
truth (points landing at the real ground height after transforming) - this
file is that same proven math, promoted from a one-off verification script
into real, reusable node code.
"""
from __future__ import annotations

import numpy as np


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """The standard formula for turning a unit quaternion (x, y, z, w) into
    the 3x3 rotation matrix it represents.
    """
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform_points(points_xyz: np.ndarray, transform) -> np.ndarray:
    """Apply a `TransformStamped`'s rotation + translation to every row of an
    `(N, 3)` array of points, returning a new `(N, 3)` array expressed in the
    transform's target frame (`transform.header.frame_id`).
    """
    t = transform.transform.translation
    q = transform.transform.rotation
    rotation = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    translation = np.array([t.x, t.y, t.z])
    # points @ R.T applies the same rotation to every row at once (this is
    # just "R @ point" for each point individually, rewritten as one matrix
    # multiply instead of a Python loop over N points).
    return points_xyz @ rotation.T + translation


def translation_of(transform) -> np.ndarray:
    """Just the (x, y, z) translation part of a `TransformStamped` - e.g.
    used as "where was the sensor" for `fuse_points`' distance-based noise
    model, without needing a whole separate lookup for that alone.
    """
    t = transform.transform.translation
    return np.array([t.x, t.y, t.z])
