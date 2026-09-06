"""Unit tests for emap.utils.gridmap_utils: the GridMap on-wire layer
encoding (`encode_layer_to_multiarray`) and its inverse (`decode_gridmap`).

Pure NumPy/ROS-message construction - no live ROS node or Gazebo needed to
build or inspect a `grid_map_msgs/GridMap` message. These tests exist because
`decode_gridmap` is a new consumer of the encoder's column-major convention
(added so `nav`'s `planner_node.py` has one authoritative decoder instead of
re-deriving the layout by hand) - a round-trip test is the cheapest way to
prove the two stay in lockstep.
"""
import numpy as np
import pytest
from grid_map_msgs.msg import GridMap

from emap.utils.gridmap_utils import decode_gridmap, encode_layer_to_multiarray


class TestEncodeDecodeRoundTrip:
    def test_round_trip_recovers_a_non_square_layer_exactly(self):
        # Deliberately non-square (rows != cols) so a transposed-order bug in
        # either function would show up as a shape mismatch, not just wrong
        # values in a shape that happens to still "fit".
        original = np.arange(12, dtype=np.float32).reshape(3, 4)
        msg = encode_layer_to_multiarray(original)
        decoded = np.array(msg.data, dtype=np.float32).reshape((3, 4), order="F")
        np.testing.assert_array_equal(decoded, original)

    def test_decode_gridmap_recovers_every_layer_by_name(self):
        elevation = np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)
        traversability = np.array([[1.0, 0.3], [0.0, 1.0]], dtype=np.float32)

        gm = GridMap()
        gm.layers = ["elevation", "traversability"]
        gm.data = [
            encode_layer_to_multiarray(elevation),
            encode_layer_to_multiarray(traversability),
        ]

        layers = decode_gridmap(gm)

        assert set(layers.keys()) == {"elevation", "traversability"}
        np.testing.assert_array_equal(layers["elevation"], elevation)
        np.testing.assert_array_equal(layers["traversability"], traversability)

    def test_round_trip_survives_a_realistic_map_size(self):
        # A shape closer to what elevation_mapping_node.py actually
        # publishes (200x200 @ 0.1m/cell = 20m map) - the small 2x2/3x4
        # cases above could in principle pass by coincidence on a
        # transposed-but-symmetric layout; this can't.
        rng = np.random.default_rng(0)
        original = rng.uniform(-1.0, 1.0, size=(200, 200)).astype(np.float32)
        msg = encode_layer_to_multiarray(original)
        decoded = np.array(msg.data, dtype=np.float32).reshape((200, 200), order="F")
        np.testing.assert_allclose(decoded, original)
