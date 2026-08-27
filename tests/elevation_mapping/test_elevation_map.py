"""Test elevation map data structures."""
import pytest
import numpy as np
from terralink_elevation.elevation_map import ElevationMapCPU
from terralink_elevation.parameter import Parameter


class TestElevationMap:
    @pytest.fixture
    def params(self):
        p = Parameter()
        p.resolution = 0.05
        p.map_length = 10.0
        p.update()
        return p
    
    @pytest.fixture
    def map_obj(self, params):
        return ElevationMapCPU(params)
    
    def test_initialization(self, map_obj, params):
        assert map_obj.elevation_map.shape == (7, params.cell_n, params.cell_n)
        assert map_obj.elevation_map.dtype == np.float32
        # Variance initialized to initial_variance
        assert np.all(map_obj.elevation_map[1] == params.initial_variance)
        # is_valid initialized to 0
        assert np.all(map_obj.elevation_map[2] == 0.0)
        # traversability initialized to 1.0
        assert np.all(map_obj.elevation_map[3] == 1.0)
    
    def test_layer_names(self, map_obj):
        assert map_obj.layer_names == [
            "elevation", "variance", "is_valid", 
            "traversability", "time", "upper_bound", "is_upper_bound"
        ]