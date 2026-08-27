"""Test parameter system."""
import pytest
from terralink_elevation.parameter import Parameter


class TestParameter:
    def test_default_values(self):
        p = Parameter()
        assert p.resolution == 0.05
        assert p.map_length == 20.0
        assert p.sensor_noise_factor == 0.05
    
    def test_update_computes_cell_n(self):
        p = Parameter(resolution=0.05, map_length=10.0)
        p.update()
        assert p.cell_n == int(round(10.0 / 0.05)) + 2  # 202
        assert p.true_cell_n == 200
        assert p.true_map_length == 10.0
    
    def test_yaml_serialization(self, tmp_path):
        import yaml
        p = Parameter(resolution=0.04, map_length=8.0)
        p.update()
        
        # Save
        yaml_file = tmp_path / "test_params.yaml"
        with open(yaml_file, 'w') as f:
            yaml.dump(p.__dict__, f)
        
        # Load
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)
        
        p2 = Parameter(**data)
        p2.update()
        assert p2.cell_n == p.cell_n