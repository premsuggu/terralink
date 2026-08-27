"""Elevation Mapping ROS 2 Node - Main entry point."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2
from grid_map_msgs.msg import GridMap
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
from tf2_ros import TransformException

import numpy as np
import ros2_numpy as rnp
from typing import Optional, Tuple

from terralink_elevation.parameter import Parameter
from terralink_elevation.elevation_map import ElevationMapCPU
from terralink_elevation.utils.gridmap_utils import elevation_map_to_gridmap


class ElevationMappingNode(Node):
    """ROS 2 Node for GPU-accelerated elevation mapping."""
    
    def __init__(self):
        super().__init__('elevation_mapping_node')
        
        # Load parameters
        self.param = self._load_parameters()
        
        # Initialize elevation map (CPU to avoid CuPy kernel compilation issues)
        self.elevation_map = self._create_elevation_map(self.param)
        self.get_logger().info('Elevation map backend: CPU (NumPy)')
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Get pointcloud topic from parameters (with fallback)
        pc_topic = getattr(self.param, 'pointcloud_topic', '/my_uav/camera/depth/points')
        
        # Subscriber: PointCloud2
        self.pc_sub = self.create_subscription(
            PointCloud2,
            pc_topic,
            self.pointcloud_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )
        
        # Publisher: GridMap - use default QoS (RELIABLE) to match subscriber
        pub_topic = getattr(self.param, 'publish_topic', '/elevation_mapping_node/elevation_map')
        self.get_logger().info(f'Creating publisher for topic: {pub_topic}')
        self.map_pub = self.create_publisher(
            GridMap,
            pub_topic,
            10
        )
        
        # Timers
        if self.param.update_pose_fps > 0:
            self.pose_timer = self.create_timer(
                1.0 / self.param.update_pose_fps, self.pose_update
            )
        else:
            self.pose_timer = None
            self.get_logger().info('Pose update disabled (update_pose_fps=0)')
        
        self.publish_timer = self.create_timer(
            1.0 / self.param.publish_fps, self.publish_map
        )
        
        self.get_logger().info('ElevationMappingNode initialized')
        self.get_logger().info(f'Map: {self.param.map_length}x{self.param.map_length}m, '
                               f'{self.param.resolution}m resolution, '
                               f'{self.param.cell_n}x{self.param.cell_n} cells')
    
    def _load_parameters(self) -> Parameter:
        """Use Parameter class defaults (matching reference implementation)."""
        param = Parameter()
        self.get_logger().info('Using Parameter class defaults (matching reference)')
        self.get_logger().info(f'Base frame: {param.base_frame}')
        self.get_logger().info(f'Sensor frame: {param.sensor_frame}')
        self.get_logger().info(f'Map frame: {param.map_frame}')
        return param
    
    def _create_elevation_map(self, param: Parameter):
        """Create elevation map (force CPU to avoid CuPy kernel compilation issues)."""
        from terralink_elevation.elevation_map import ElevationMapCPU
        return ElevationMapCPU(param)
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Process incoming PointCloud2."""
        try:
            # Convert to numpy (CPU) then transfer to GPU if needed
            points_np = self._pointcloud2_to_xyz(msg)
            
            if len(points_np) == 0:
                return
            
            # Get transform: sensor_frame -> map_frame
            transform = self._lookup_transform(
                self.param.map_frame, 
                msg.header.frame_id, 
                msg.header.stamp
            )
            if transform is None:
                return
            
            R, t = self._transform_to_matrix(transform)
            
            # Convert to GPU arrays if using GPU elevation map
            from terralink_elevation.elevation_map_gpu import ElevationMapGPU, CUPY_AVAILABLE
            if CUPY_AVAILABLE and isinstance(self.elevation_map, ElevationMapGPU):
                import cupy as cp
                points_np = cp.asarray(points_np)
                R = cp.asarray(R)
                t = cp.asarray(t)
            
            # Fuse into elevation map (handles CPU/GPU automatically)
            self.elevation_map.fuse_pointcloud(points_np, R, t)
            
            # Update traversability
            self.elevation_map.update_traversability()
            
        except Exception as e:
            self.get_logger().error(f'PointCloud processing failed: {e}')
    
    def _pointcloud2_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        """Extract XYZ from PointCloud2 (fast path)."""
        points = rnp.numpify(msg)
        self.get_logger().info(f'PointCloud type: {type(points)}, keys: {points.keys() if isinstance(points, dict) else "N/A"}')
        if isinstance(points, dict):
            if 'xyz' in points:
                # ros2_numpy returns dict with 'xyz' (N,3) and 'rgb'
                xyz = points['xyz']
                return xyz.astype(np.float32)
            elif 'x' in points:
                # Standard format
                return np.column_stack([points['x'], points['y'], points['z']]).astype(np.float32)
        else:
            return np.column_stack([points['x'], points['y'], points['z']]).astype(np.float32)
    
    def _lookup_transform(self, target: str, source: str, stamp) -> Optional[TransformStamped]:
        """Safe TF lookup with timeout and extrapolation fallback (matching reference)."""
        try:
            return self.tf_buffer.lookup_transform(
                target, source, stamp, 
                timeout=Duration(seconds=0.1)
            )
        except TransformException as e:
            # Handle extrapolation by falling back to latest available transform
            if "extrapolation" in str(e).lower() or "extrapolating" in str(e).lower():
                try:
                    return self.tf_buffer.lookup_transform(
                        target, source, rclpy.time.Time()
                    )
                except TransformException as e2:
                    self.get_logger().warn(f'TF {source} -> {target} (fallback): {e2}')
                    return None
            self.get_logger().warn(f'TF {source} -> {target}: {e}')
            return None
    
    def _transform_to_matrix(self, transform: TransformStamped) -> Tuple[np.ndarray, np.ndarray]:
        """Convert TransformStamped to (R, t) matrices."""
        t_msg = transform.transform.translation
        q_msg = transform.transform.rotation
        
        t = np.array([t_msg.x, t_msg.y, t_msg.z], dtype=np.float32)
        
        # Quaternion to rotation matrix
        from tf_transformations import quaternion_matrix
        mat = quaternion_matrix([q_msg.x, q_msg.y, q_msg.z, q_msg.w])
        R = mat[:3, :3].astype(np.float32)
        
        return R, t
    
    def pose_update(self):
        """Update map position to follow UAV (10 Hz)."""
        transform = self._lookup_transform(
            self.param.map_frame, 
            self.param.base_frame, 
            rclpy.time.Time()
        )
        if transform is None:
            return
        
        R, t = self._transform_to_matrix(transform)
        position = np.array([t[0], t[1], t[2]], dtype=np.float32)
        
        self.elevation_map.move_to(position, R)
    
    def publish_map(self):
        """Publish GridMap message."""
        try:
            msg = elevation_map_to_gridmap(
                self.elevation_map, 
                self.param, 
                self.get_clock().now().to_msg(),
                self.param.map_frame
            )
            self.map_pub.publish(msg)
            self.get_logger().info(f'Published GridMap: {msg.layers}')
        except Exception as e:
            self.get_logger().error(f'GridMap publishing failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ElevationMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()