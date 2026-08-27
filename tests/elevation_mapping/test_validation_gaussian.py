#!/usr/bin/env python3
"""
Validation test: Gaussian bump ground truth vs generated elevation map.

Uses a HORIZONTAL sensor (ground robot with lidar) looking horizontally,
which matches the elevation mapping algorithm assumptions.
"""
import sys
sys.path.insert(0, '/home/prem/terralink/install/terralink_elevation/lib/python3.10/site-packages')

import numpy as np
from terralink_elevation.parameter import Parameter
from terralink_elevation.elevation_map import ElevationMapCPU


class GaussianBumpValidator:
    """Validate elevation mapping against known Gaussian bump ground truth.
    
    Uses a HORIZONTAL sensor (ground robot with lidar) looking horizontally,
    which matches the elevation mapping algorithm assumptions.
    """
    
    def __init__(self, resolution=0.1, map_length=20.0, bump_height=1.0, bump_sigma=2.0):
        self.resolution = resolution
        self.map_length = map_length
        self.bump_height = bump_height
        self.bump_sigma = bump_sigma
        
        # Map parameters
        self.cell_n = int(map_length / resolution) + 2
        self.true_cell_n = int(map_length / resolution)
        self.center = self.cell_n // 2
        
        # Sensor parameters (ground robot with horizontal lidar at height 0.5m)
        self.sensor_height = 0.5
        self.sensor_noise_factor = 0.05
        
        # Create ground truth elevation map
        self.ground_truth = self._create_ground_truth()
        
        # Generate synthetic point cloud
        self.points = self._generate_synthetic_points()
    
    def _create_ground_truth(self):
        """Create ground truth elevation map with Gaussian bump."""
        x = np.arange(-self.map_length/2, self.map_length/2, self.resolution)
        y = np.arange(-self.map_length/2, self.map_length/2, self.resolution)
        X, Y = np.meshgrid(x, y)
        
        # Gaussian bump at center
        Z = self.bump_height * np.exp(-(X**2 + Y**2) / (2 * self.bump_sigma**2))
        
        return Z.astype(np.float32)
    
    def _generate_synthetic_points(self, num_points=20000):
        """Generate synthetic horizontal LiDAR point cloud from ground truth."""
        h, w = self.ground_truth.shape
        
        # Sample points from ground truth
        num_points = min(num_points, h * w)
        flat_indices = np.random.choice(h * w, size=num_points, replace=False)
        rows = flat_indices // w
        cols = flat_indices % w
        
        # Convert grid indices to world coordinates
        x_coords = (cols - w/2) * self.resolution
        y_coords = (rows - h/2) * self.resolution
        z_coords = self.ground_truth[rows, cols]
        
        # Add sensor noise (increases with distance from sensor)
        # Sensor at origin, height 0.5m
        sensor_pos = np.array([0.0, 0.0, 0.5], dtype=np.float32)
        dist = np.sqrt(x_coords**2 + y_coords**2 + (z_coords - 0.5)**2)
        noise_std = self.sensor_noise_factor * dist
        noise = np.random.normal(0, noise_std)
        z_coords_noisy = z_coords + noise
        
        # Points in sensor frame (sensor at origin, looking horizontally)
        # Robot at origin, looking along +X
        # LiDAR frame: X forward, Y left, Z up
        points_sensor = np.column_stack([x_coords, y_coords, z_coords_noisy])
        
        return points_sensor.astype(np.float32)
    
    def run_validation(self):
        """Run elevation mapping on synthetic data and compare with ground truth."""
        param = Parameter(resolution=0.1, map_length=20.0, 
                         sensor_noise_factor=0.05,
                         enable_visibility_cleanup=True, 
                         cleanup_step=0.1, cleanup_cos_thresh=0.3,
                         max_ray_length=20.0, min_height=-0.5, max_height=3.0,
                         mahalanobis_thresh=2.0)
        param.update()
        m = ElevationMapCPU(param)
        
        # Sensor at origin, looking horizontally
        R_mat = np.eye(3, dtype=np.float32)
        t_vec = np.zeros(3, dtype=np.float32)
        
        # Fuse synthetic point cloud
        m.fuse_pointcloud(self.points, R_mat, t_vec)
        m.update_traversability()
        
        # Extract generated elevation map (valid region only)
        border = (m.cell_n - m.true_cell_n) // 2
        s = slice(border, border + m.true_cell_n)
        generated = m.elevation_map[m.IDX_ELEVATION, s, s]
        validity = m.elevation_map[m.IDX_IS_VALID, s, s]
        variance = m.elevation_map[m.IDX_VARIANCE, s, s]
        
        # Crop ground truth to match generated map size
        gt_h, gt_w = self.ground_truth.shape
        gen_h, gen_w = generated.shape
        
        # Handle potential size differences by cropping/padding
        min_h = min(gt_h, gen_h)
        min_w = min(gt_w, gen_w)
        
        gt_cropped = self.ground_truth[:min_h, :min_w]
        gen_cropped = generated[:min_h, :min_w]
        valid_cropped = validity[:min_h, :min_w]
        var_cropped = variance[:min_h, :min_w]
        
        valid_mask = valid_cropped > 0.5
        
        if not np.any(valid_mask):
            return {"error": "No valid cells in generated map"}
        
        gt_valid = gt_cropped[valid_mask]
        gen_valid = gen_cropped[valid_mask]
        var_valid = variance[:min_h, :min_w][valid_mask]
        
        errors = gen_valid - gt_valid
        rmse = np.sqrt(np.mean(errors**2))
        mae = np.mean(np.abs(errors))
        max_error = np.max(np.abs(errors))
        
        rel_rmse = rmse / (np.max(gt_valid) - np.min(gt_valid)) if np.max(gt_valid) > np.min(gt_valid) else 0
        
        bias = np.mean(errors)
        coverage = np.sum(valid_cropped > 0.5) / valid_cropped.size
        
        return {
            "rmse": float(rmse),
            "mae": float(mae),
            "max_error": float(max_error),
            "rel_rmse": float(rel_rmse),
            "bias": float(bias),
            "coverage": float(coverage),
            "num_valid_cells": int(np.sum(valid_mask)),
            "total_cells": int(valid_cropped.size),
            "mean_variance": float(np.mean(variance[:min_h, :min_w][valid_mask])),
            "generated_shape": generated.shape,
            "gt_shape": gt_cropped.shape,
            "gt_range": (float(np.min(gt_valid)), float(np.max(gt_valid))),
            "gen_range": (float(np.min(gen_valid)), float(np.max(gen_valid)))
        }


def run_validation():
    """Run validation with different configurations."""
    print("=" * 60)
    print("ELEVATION MAP VALIDATION: GAUSSIAN BUMP (Horizontal LiDAR)")
    print("=" * 60)
    
    configs = [
        {"resolution": 0.1, "bump_height": 1.0, "bump_sigma": 2.0, "name": "Default"},
        {"resolution": 0.05, "bump_height": 1.0, "bump_sigma": 2.0, "name": "HighRes"},
        {"resolution": 0.1, "bump_height": 2.0, "bump_sigma": 1.0, "name": "SteepBump"},
        {"resolution": 0.1, "bump_height": 0.5, "bump_sigma": 3.0, "name": "ShallowWide"},
    ]
    
    results = []
    for config in configs:
        print(f"\n--- Testing: {config['name']} ---")
        print(f"  Resolution: {config['resolution']}m, Bump: {config['bump_height']}m, Sigma: {config['bump_sigma']}m")
        
        try:
            validator = GaussianBumpValidator(**{k:v for k,v in config.items() if k != 'name'})
            metrics = validator.run_validation()
            
            if "error" in metrics:
                print(f"  ERROR: {metrics['error']}")
                continue
            
            print(f"  RMSE:     {metrics['rmse']:.4f} m")
            print(f"  MAE:      {metrics['mae']:.4f} m")
            print(f"  Max Err:  {metrics['max_error']:.4f} m")
            print(f"  Rel RMSE: {metrics['rel_rmse']:.2%}")
            print(f"  Bias:     {metrics['bias']:.4f} m")
            print(f"  Coverage: {metrics['coverage']:.2%}")
            print(f"  Valid Cells: {metrics['num_valid_cells']}/{metrics['total_cells']}")
            print(f"  Mean Var: {metrics['mean_variance']:.4f}")
            
            metrics['config'] = config['name']
            results.append(metrics)
            
        except Exception as e:
            print(f"  ERROR: {e}")
            import traceback
            traceback.print_exc()
    
    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    for r in results:
        print(f"{r['config']:12s} | RMSE: {r['rmse']:.4f}m | MAE: {r['mae']:.4f}m | Cov: {r['coverage']:.1%} | Bias: {r['bias']:.4f}m")
    
    return results


if __name__ == "__main__":
    import sys
    sys.path.insert(0, '/home/prem/terralink/install/terralink_elevation/lib/python3.10/site-packages')
    from terralink_elevation.parameter import Parameter
    from terralink_elevation.elevation_map import ElevationMapCPU
    run_validation()