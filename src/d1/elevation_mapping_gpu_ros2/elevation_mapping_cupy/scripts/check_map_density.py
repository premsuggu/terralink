#!/usr/bin/env python3
"""
Map Density Checker for Elevation Mapping.

Subscribes to GridMap topic and reports density statistics.
"""

import rclpy
from rclpy.node import Node
from grid_map_msgs.msg import GridMap
import numpy as np


class MapDensityChecker(Node):
    def __init__(self):
        super().__init__('map_density_checker')
        self.sub = self.create_subscription(
            GridMap, '/elevation_mapping_node/elevation_map', 
            self.callback, 10)
        self.get_logger().info('Map Density Checker started. Waiting for /elevation_mapping_node/elevation_map...')
        
    def callback(self, msg):
        # Find elevation layer
        try:
            idx = msg.layers.index('elevation')
        except ValueError:
            self.get_logger().warn('No elevation layer found in GridMap')
            return
            
        data = np.array(msg.data[idx].data, dtype=np.float32)
        
        # Get dimensions from layout
        if len(msg.data[idx].layout.dim) >= 2:
            rows = msg.data[idx].layout.dim[0].size
            cols = msg.data[idx].layout.dim[1].size
        else:
            size = len(data)
            rows = cols = int(np.sqrt(size))
            
        data = data.reshape(rows, cols)
        
        total = data.size
        valid = np.count_nonzero(np.isfinite(data))
        pct = 100.0 * valid / total if total > 0 else 0.0
        
        valid_data = data[np.isfinite(data)]
        if len(valid_data) > 0:
            elev_min = np.min(valid_data)
            elev_max = np.max(valid_data)
            elev_mean = np.mean(valid_data)
            elev_std = np.std(valid_data)
        else:
            elev_min = elev_max = elev_mean = elev_std = 0.0
        
        # Get variance layer if available
        var_pct = 0.0
        try:
            var_idx = msg.layers.index('variance')
            var_data = np.array(msg.data[var_idx].data, dtype=np.float32).reshape(rows, cols)
            var_valid = np.count_nonzero(np.isfinite(var_data))
            var_pct = 100.0 * var_valid / total if total > 0 else 0.0
        except ValueError:
            pass
            
        # Get traversability layer if available
        trav_pct = 0.0
        try:
            trav_idx = msg.layers.index('traversability')
            trav_data = np.array(msg.data[trav_idx].data, dtype=np.float32).reshape(rows, cols)
            trav_valid = np.count_nonzero(np.isfinite(trav_data))
            trav_pct = 100.0 * trav_valid / total if total > 0 else 0.0
        except ValueError:
            pass
        
        print(f"\n{'='*60}")
        print(f"MAP DENSITY REPORT")
        print(f"{'='*60}")
        print(f"Grid: {rows}x{cols} = {total:,} cells")
        print(f"Elevation layer: {valid:,} valid ({pct:.1f}%)")
        print(f"  Range: {elev_min:.3f} to {elev_max:.3f} m")
        print(f"  Mean:  {elev_mean:.3f} m, Std: {elev_std:.3f} m")
        print(f"Variance layer:  {var_pct:.1f}% valid")
        print(f"Traversability:  {trav_pct:.1f}% valid")
        
        if pct >= 90:
            print("✅ DENSE MAP ACHIEVED (>90%)")
        elif pct >= 50:
            print("⚠️  PARTIAL MAP (50-90%) - needs more loops")
        elif pct >= 25:
            print("⚠️  SPARSE MAP (25-50%) - needs more loops")
        else:
            print("❌ VERY SPARSE MAP (<25%) - check sensor config")
        
        print(f"{'='*60}\n")


def main():
    rclpy.init()
    node = MapDensityChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()