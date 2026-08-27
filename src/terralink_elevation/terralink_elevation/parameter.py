"""Parameter dataclass with YAML loading support and validation."""
from dataclasses import dataclass, field, fields
from typing import Any, Dict, List
import yaml
from pathlib import Path


@dataclass
class Parameter:
    """Elevation mapping parameters.
    
    All parameters have sensible defaults matching the reference implementation.
    Call update() after changing resolution or map_length to recompute derived values.
    """
    
    # Map geometry (matching reference synthetic demo)
    resolution: float = 0.04          # meters per cell (4cm)
    map_length: float = 8.0           # map size in meters (square, 8x8m)
    min_height: float = -2.0          # minimum valid height
    max_height: float = 5.0           # maximum valid height
    
    # Computed (call update() after changing resolution/map_length)
    cell_n: int = field(init=False, default=0)
    true_cell_n: int = field(init=False, default=0)
    true_map_length: float = field(init=False, default=0.0)
    
    # Sensor noise model
    sensor_noise_factor: float = 0.05
    min_valid_distance: float = 0.3
    
    # Outlier rejection
    mahalanobis_thresh: float = 2.0
    outlier_variance: float = 0.01
    
    # Drift compensation
    enable_drift_compensation: bool = True
    max_drift: float = 0.10
    drift_compensation_alpha: float = 0.1
    position_noise_thresh: float = 0.01
    orientation_noise_thresh: float = 0.01
    drift_compensation_variance_inlier: float = 0.1
    traversability_inlier: float = 0.9
    min_height_drift_cnt: int = 100
    
    # Visibility cleanup
    enable_visibility_cleanup: bool = True
    max_ray_length: float = 10.0
    cleanup_step: float = 0.05
    cleanup_cos_thresh: float = 0.3
    
    # Traversability
    max_slope: float = 0.35
    max_step: float = 0.15
    max_roughness: float = 0.05
    
    # Timing
    update_pose_fps: float = 0.0        # Pose update frequency (Hz) - 0 = fixed map (matching reference)
    update_variance_fps: float = 5.0
    publish_fps: float = 2.0
    
    # Frames (matching reference: pointcloud in base_link frame)
    map_frame: str = "map"
    base_frame: str = "base_link"
    sensor_frame: str = "base_link"
    
    # Topics (matching reference)
    pointcloud_topic: str = "/camera/depth/points"
    publish_topic: str = "/elevation_mapping_node/elevation_map"
    
    # Internal constants
    initial_variance: float = 1.0
    max_variance: float = 10.0
    
    def update(self):
        """Recompute derived parameters. Call after changing resolution or map_length."""
        self._validate_geometry()
        self.true_cell_n = round(self.map_length / self.resolution)
        self.cell_n = self.true_cell_n + 2  # +2 border for shifting
        self.true_map_length = self.true_cell_n * self.resolution
    
    def validate(self) -> List[str]:
        """Validate all parameters. Returns list of error messages (empty if valid)."""
        errors = []
        
        # Geometry validation
        if self.resolution <= 0:
            errors.append(f"resolution must be > 0, got {self.resolution}")
        if self.resolution > 1.0:
            errors.append(f"resolution should be <= 1.0 (1m/cell), got {self.resolution}")
        if self.map_length <= 0:
            errors.append(f"map_length must be > 0, got {self.map_length}")
        if self.map_length > 1000:
            errors.append(f"map_length should be <= 1000m, got {self.map_length}")
        if self.min_height >= self.max_height:
            errors.append(f"min_height ({self.min_height}) must be < max_height ({self.max_height})")
        
        # Sensor validation
        if self.sensor_noise_factor <= 0:
            errors.append(f"sensor_noise_factor must be > 0, got {self.sensor_noise_factor}")
        if self.min_valid_distance < 0:
            errors.append(f"min_valid_distance must be >= 0, got {self.min_valid_distance}")
        
        # Outlier rejection validation
        if self.mahalanobis_thresh <= 0:
            errors.append(f"mahalanobis_thresh must be > 0, got {self.mahalanobis_thresh}")
        if self.outlier_variance <= 0:
            errors.append(f"outlier_variance must be > 0, got {self.outlier_variance}")
        
        # Drift compensation validation
        if self.max_drift <= 0:
            errors.append(f"max_drift must be > 0, got {self.max_drift}")
        if not 0 <= self.drift_compensation_alpha <= 1:
            errors.append(f"drift_compensation_alpha must be in [0, 1], got {self.drift_compensation_alpha}")
        if self.position_noise_thresh < 0:
            errors.append(f"position_noise_thresh must be >= 0, got {self.position_noise_thresh}")
        if self.orientation_noise_thresh < 0:
            errors.append(f"orientation_noise_thresh must be >= 0, got {self.orientation_noise_thresh}")
        if not 0 <= self.drift_compensation_variance_inlier <= 1:
            errors.append(f"drift_compensation_variance_inlier must be in [0, 1], got {self.drift_compensation_variance_inlier}")
        if not 0 <= self.traversability_inlier <= 1:
            errors.append(f"traversability_inlier must be in [0, 1], got {self.traversability_inlier}")
        if self.min_height_drift_cnt < 0:
            errors.append(f"min_height_drift_cnt must be >= 0, got {self.min_height_drift_cnt}")
        
        # Visibility cleanup validation
        if self.max_ray_length <= 0:
            errors.append(f"max_ray_length must be > 0, got {self.max_ray_length}")
        if not 0 < self.cleanup_step <= 1:
            errors.append(f"cleanup_step must be in (0, 1], got {self.cleanup_step}")
        if not -1 <= self.cleanup_cos_thresh <= 1:
            errors.append(f"cleanup_cos_thresh must be in [-1, 1], got {self.cleanup_cos_thresh}")
        
        # Traversability validation
        if not 0 < self.max_slope <= 1.57:  # ~90 degrees in radians
            errors.append(f"max_slope must be in (0, 1.57], got {self.max_slope}")
        if self.max_step <= 0:
            errors.append(f"max_step must be > 0, got {self.max_step}")
        if self.max_roughness <= 0:
            errors.append(f"max_roughness must be > 0, got {self.max_roughness}")
        
        # Timing validation
        if self.update_pose_fps < 0:
            errors.append(f"update_pose_fps must be >= 0, got {self.update_pose_fps}")
        if self.update_variance_fps <= 0:
            errors.append(f"update_variance_fps must be > 0, got {self.update_variance_fps}")
        if self.publish_fps <= 0:
            errors.append(f"publish_fps must be > 0, got {self.publish_fps}")
        
        # Frame validation
        if not self.map_frame:
            errors.append("map_frame cannot be empty")
        if not self.base_frame:
            errors.append("base_frame cannot be empty")
        if not self.sensor_frame:
            errors.append("sensor_frame cannot be empty")
        
        # Topic validation
        if not self.pointcloud_topic:
            errors.append("pointcloud_topic cannot be empty")
        if not self.publish_topic:
            errors.append("publish_topic cannot be empty")
        
        # Internal constants
        if self.initial_variance <= 0:
            errors.append(f"initial_variance must be > 0, got {self.initial_variance}")
        if self.max_variance <= self.initial_variance:
            errors.append(f"max_variance ({self.max_variance}) must be > initial_variance ({self.initial_variance})")
        
        return errors
    
    def _validate_geometry(self):
        """Validate geometry parameters (called from update)."""
        if self.resolution <= 0:
            raise ValueError(f"resolution must be > 0, got {self.resolution}")
        if self.map_length <= 0:
            raise ValueError(f"map_length must be > 0, got {self.map_length}")
        if self.min_height >= self.max_height:
            raise ValueError(f"min_height ({self.min_height}) must be < max_height ({self.max_height})")
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for YAML serialization."""
        return {f.name: getattr(self, f.name) for f in fields(self) if f.init}
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Parameter':
        """Create Parameter from dictionary."""
        # Filter only init fields
        init_fields = {f.name for f in fields(cls) if f.init}
        filtered = {k: v for k, v in data.items() if k in init_fields}
        return cls(**filtered)
    
    @classmethod
    def from_yaml(cls, yaml_path: str) -> 'Parameter':
        """Load parameters from YAML file."""
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        # Handle nested structure (e.g., 'terralink_elevation:' key)
        if 'terralink_elevation' in data:
            data = data['terralink_elevation']
        if 'ros__parameters' in data:
            data = data['ros__parameters']
        return cls.from_dict(data)
    
    def save_yaml(self, yaml_path: str):
        """Save parameters to YAML file."""
        data = {'terralink_elevation': {'ros__parameters': self.to_dict()}}
        with open(yaml_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
    
    def __post_init__(self):
        """Auto-update and validate on creation."""
        errors = self.validate()
        if errors:
            raise ValueError("Parameter validation failed:\n" + "\n".join(errors))
        self.update()